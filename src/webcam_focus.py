# Control focus on webcam using printer's XY coordinates,
# and/or control other webcam settings using gcode macros.
#
# Copyright (C) 2022 Colin MacKenzie <colin@flyingeinstein.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import subprocess
import numpy as np

UPDATE_TIME  = 1.0

def v4l2_query():
    try:
	lines=subprocess.check_output(['v4l2-ctl','-kl']).split('\n')
	controls = []
        valueof = lambda x: int(x) if x.isdigit() else x
	for line in lines:
            try:
		left, right = line.split(':')
		prop_name, prop_id, dtype = left.split()
		properties = [p.split('=') for p in right.split()]
                props = {p[0]:valueof(p[1]) for p in properties}
		obj = {
			'name': prop_name,
			'id': prop_id,
			'type': dtype
		}
                obj.update(props)
		controls.append(obj)
            except:
                pass
	return controls
    except:
        return None


def v4l2_query_single(prop_name):
    props = v4l2_query()
    if props is not None:
        for p in props:
            if p['name'] == prop_name:
                return p['value']
    return None


######################################################################
# WebCam Focus
######################################################################

class WebcamFocus:
    cmd_v4l2_help = 'Set settings on your webcam'
    cmd_clear_points_help = 'Clear any remembered focal points'
    cmd_add_point_help = 'Add the current focus and position into the focus mapper'
    cmd_focus_mapper_help = 'Show the points in the focus mapper'
    cmd_focus_calibrate_help = 'Calibrate focus automatically using a focus pattern on the toolhead'

    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.gcode   = self.printer.lookup_object('gcode')
        #self.printer.load_object(config, "display_status")
        #self.min_focus = config.getfloat('min_focus', 0.)
        #self.max_focus = config.getfloat('max_focus', 255.)
        self.position = [0.0, 0.0, 0.0]
        self.focus = 50
        self.stepperTimer     = None
        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.gcode.register_command('WEBCAM_SETTINGS',
                                    self.cmd_v4l2,
                                    desc=self.cmd_v4l2_help)
        self.gcode.register_command('WEBCAM_FOCUS_CLEAR',
                                    self.cmd_clear_points,
                                    desc=self.cmd_clear_points_help)
        self.gcode.register_command('WEBCAM_FOCUS_SAVE',
                                    self.cmd_save_point,
                                    desc=self.cmd_add_point_help)
        self.gcode.register_command('WEBCAM_FOCUS_MAPPER',
                                    self.cmd_focus_mapper_show,
                                    desc=self.cmd_add_point_help)
        self.gcode.register_command('WEBCAM_FOCUS_CALIBRATE',
                                    self.cmd_focus_calibrate,
                                    desc=self.cmd_focus_calibrate_help)
        self.shutdown = False

        self.distances = []
        self.focals = []
        self.focus_mappings = '60.0:220, 80.0:150, 100.0:110, 120.0:100, 140.0:90, 160.0:80, 180.0:80, 200.0:80, 220.0:70, 240.0:70, 260.0:70, 280.0:60, 300.0:60, 320.0:60, 340.0:60, 360.0:60, 380.0:60, 400.0:60, 420.0:60, 440.0:60, 460.0:60'


    def parse_focus_mappings(self, fm):
        axes_max = self.kin.axes_max
        axes_min = self.kin.axes_min
        dist_mapper = lambda x,y: (y - axes_min[1]) / (axes_max[1] - axes_min[1])
        try:
            mappings = fm.split(',')
            distances = []
            focals = []
            for m in mappings:
                d, f = m.split(':')
                distances.append(dist_mapper(0, float(d)))
                focals.append(int(f))

            self.distances = distances
            self.focals = focals
            self.build_focus_mapper()
        except:
            self.gcode.respond_raw('cannot parse the focus mappings')


    def build_focus_mapper(self):
        # using polyline
        #self.focus_mapper_coeffs = np.polyfit(self.distances, self.focals, 2)
        #self.focus_mapper = np.poly1d(self.focus_mapper_coeffs)

        # using linear interp
        focals = sorted([[d, f] for d,f in zip(self.distances, self.focals)], key=lambda x: x[0])
        def _mapper(dist):
            prev = focals[0]
            for f in focals:
                if dist < f[0]:
                    t = f[0] - prev[0]
                    if t == 0:
                        return f[1]
                    else:
                        r = (dist - prev[0]) / t
                        fp = prev[1] + r * (f[1] - prev[1])
                        return int(fp)
                prev = f
            #return int(prev) if prev else 80
        self.focus_mapper = _mapper


    def _handle_ready(self):
        self.shutdown = False
        self.reactor = self.printer.get_reactor()
        self.printer.register_event_handler('klippy:shutdown', 
                                            self._handle_shutdown)

        # machine position from kinematics driver
        self.position = [0.0, 0.0, 0.0]
        self.toolhead = self.printer.lookup_object('toolhead')
        self.kin = self.toolhead.get_kinematics()

        if self.focus_mappings:
            self.parse_focus_mappings(self.focus_mappings)
            #self.build_focus_mapper()

        # register update timers
        #self.displayStatus = self.printer.lookup_object('display_status')
        self.updateTimer = self.reactor.register_timer(
                                self._updateFocus, 
                                self.reactor.NOW)
        self.stepperTimer = self.reactor.register_timer(
                                self._pollStepper,
                                self.reactor.NOW)

    def _handle_shutdown(self):
        self.shutdown = True
        pass

    def _pollStepper(self, eventtime):
        # get raw stepper counts (target position)
        kin_spos = {s.get_name(): s.get_commanded_position()
                    for s in self.kin.get_steppers()}
       
        # covert stepper counts to machine coordinates
        pos = self.kin.calc_position(kin_spos)
        
        for i in range(3):
            if pos[i] >= self.kin.axes_min[i] and pos[i] <= self.kin.axes_max[i]:
                self.position[i] = (pos[i] - self.kin.axes_min[i]) / (self.kin.axes_max[i] - self.kin.axes_min[i])
        return eventtime + 0.5

    def _updateFocus(self, eventtime):
        if self.focus_mapper is not None:
            dist = self.distance()
            focus = self.focus_mapper(dist)
            focus = max(0, min(250, focus))
            if self.focus != focus:
                self.focus = focus
                self.gcode.respond_raw('Focus {0} <= {1}'.format(focus, dist))
                self._focus(focus)

        return eventtime + UPDATE_TIME

    def distance(self):
        y = self.position[1]
        return y

    def distance_to_mm(self, d):
        range = self.kin.axes_max[1] - self.kin.axes_min[1]
        return d * range

    def cmd_v4l2(self, gcmd):
        focus_abs=gcmd.get_float('FOCUS_ABSOLUTE', None)
        focus_auto=gcmd.get_int('FOCUS_AUTO', None)
        if focus_auto is not None:
            self.gcode.respond_raw('enabling auto-focus' if focus_auto else 'disabling auto-focus')
            self._control('focus_auto', focus_auto)
        if focus_abs is not None:
            self.last_manual_focus = focus_abs
            self.gcode.respond_raw('Setting focus to {0}'.format(focus_abs))
            self._focus(focus_abs)

    def cmd_clear_points(self, gcmd):
        self.distances = []
        self.focals = []
        self.focus_mapper_coeffs = None
        self.focus_mapper = None

    def cmd_save_point(self, gcmd):
        focus_abs=gcmd.get_float('FOCUS_ABSOLUTE', self.last_manual_focus)
        dist=gcmd.get_float('D', self.distance())
        if focus_abs is None:
            self.gcode.respond_raw('Missing FOCUS_ABSOLUTE parameter and no previous setting has been made')
        elif dist is None:
            self.gcode.respond_raw('Missing D parameter (probably you have not homed yet)')
        else:
            self.distances.append(dist)
            self.focals.append(focus_abs)
            if len(self.distances) > 1:
                self.build_focus_mapper()

    def cmd_focus_mapper_show(self, gcmd):
        save_graph=gcmd.get_int('GRAPH', 0)
        points=[]
        for d,f in zip(self.distances, self.focals):
            mm = self.distance_to_mm(d)
            points.append('{}:{}'.format(mm, f))
        self.gcode.respond_raw('focus_mappings: ' + ', '.join(points))

        if save_graph > 0:
            try:
                import matplotlib
                matplotlib.use('Agg')
                import matplotlib.pyplot as plt
            except Exception as e:
                self.gcode.respond_raw('graphing requires the matplotlib (pyplot) python library to be installed: ' + str(e))
                return
            
            polyline = np.linspace(0, 1, 255)
            plt.rcParams["figure.figsize"] = [7.50, 3.50]
            plt.rcParams["figure.autolayout"] = True
            plt.scatter(self.distances, self.focals)
            plt.plot(polyline, self.focus_mapper(polyline), color='blue')
            plt.savefig('/home/pi/klipper_config/focus_mapper.png')

        
    def cmd_focus_calibrate(self, gcmd):
        ymin=gcmd.get_int('Y_MIN', 60)
        ymax=gcmd.get_int('Y_MAX', 480)
        ystep=gcmd.get_int('Y_STEP', 20)
        move_speed=gcmd.get_int('MOVE_SPEED', 600)

        toolhead = self.printer.lookup_object('toolhead')
        if not toolhead:
            self.gcode.respond_raw('cannot get control of the toolhead')
            return

        # make a mapper for machine coord to [0..1] distance
        # todo: for now just using the coord
        axes_max = self.kin.axes_max
        axes_min = self.kin.axes_min
        dist_mapper = lambda x,y: (y - axes_min[1]) / (axes_max[1] - axes_min[1])

        # enable auto-focus
        self.gcode.respond_raw('enabling auto-focus')
        self._control('contrast', 255)
        self._control('exposure_absolute', 200)
        self._focus_auto(True)

        # todo: turn exposure down, and contrast way up as a kind of thresholding filter

        # clear existing calibration points
        self.distances = []
        self.focals = []
        self.focus_mapper_coeffs = None
        self.focus_mapper = None
       
        for p in range(ymin, ymax, 20):
            # extent position should be far away from test position
            extent_y = 100 if p > 250 else 450

            # move to extent to wipe focus
            point = [250, extent_y]
            toolhead.manual_move(point, move_speed)
            toolhead.wait_moves()
            toolhead.dwell(1.0)

            # move to new test position
            point = [250, p]
            toolhead.manual_move(point, move_speed)
            toolhead.wait_moves()
            toolhead.dwell(8.0)

            # record focus
            focus = v4l2_query_single('focus_absolute')
            if focus is not None:
                d = dist_mapper(point[0], point[1])
                self.gcode.respond_raw('  focus {}:{}'.format(d, focus))
                self.distances.append(d)
                self.focals.append(focus)


        if len(self.distances) > 1:
            self.build_focus_mapper()
        self.gcode.respond_raw('finished auto-focus calibration')



    def _control(self, key, value):
        subprocess.call(['v4l2-ctl -d /dev/video0 -c {}={}'.format(key, str(value))], shell=True)

    def _focus(self, value):
        self._control('focus_absolute', value)

    def _focus_auto(self, enable):
        self._control('focus_auto', 1 if enable else 0)



def test():
    try:
      import matplotlib
      matplotlib.use('Agg')
      import matplotlib.pyplot as plt
    except Exception as e:
      print('graphing requires the matplotlib (pyplot) python library to be installed' + str(e))
      return
            
    distances = [0.16, 0.34, 0.5, .65, 0.8, 1.0]
    focals = [250,  80, 77, 55, 53, 50]
    focus_mapper_coeffs = np.polyfit(distances, focals, 2)
    focus_mapper = np.poly1d(focus_mapper_coeffs)
    polyline = np.linspace(0, 1, 255)
    plt.rcParams["figure.figsize"] = [7.50, 3.50]
    plt.rcParams["figure.autolayout"] = True
    plt.scatter(distances, focals)
    plt.plot(polyline, focus_mapper(polyline), color='blue')
    plt.savefig('/home/pi/klipper_config/focus_mapper.png')


def test_interp():
    distances_in = [0.5, 0.16, 0.8, 0.34, .65, 1.0]
    focals_in = [77, 250,  53, 80, 55, 50]
    # zip and sort the distance:focal calibration points
    focals = sorted([[d, f] for d,f in zip(distances_in, focals_in)], key=lambda x: x[0])
    for f in focals:
        print('{}:{}'.format(f[0], f[1]))
    def _mapper(dist):
        prev = focals[0]
        for f in focals:
            if dist < f[0]:
                t = f[0] - prev[0]
                if t == 0:
                    return f[1]
                else:
                    r = (dist - prev[0]) / t
                    fp = prev[1] + r * (f[1] - prev[1])
                    return fp
            prev = f

    for d in range(0, 10):
        print('{} => {}'.format(d/10.0, _mapper(d/10.0)))


#test_interp()
#props = v4l2_query()
#for p in props:
#    print('{}: {}'.format(p['name'], p['value']))

#print('focus: {}'.format(v4l2_query_single('focus_absolute')))

def load_config(config):
    return WebcamFocus(config)


#def load_config_prefix(config):
#    return WebcamFocus(config)
