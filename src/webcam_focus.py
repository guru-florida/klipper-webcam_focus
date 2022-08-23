# Control focus on webcam using printer's XY coordinates,
# and/or control other webcam settings using gcode macros.
#
# Copyright (C) 2022 Colin MacKenzie <colin@flyingeinstein.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import subprocess
import math
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


def parse_focal(s):
    d, f = s.split(':')
    return float(d), int(f)

def ensure_xyz(tup):
    return tup + (0.0, 0.0, 0.0)[0:3 - len(tup)]



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

        self.video_dev = config.get('device', '/dev/video0')
        self.min_focus = config.getint('min_focus', 0)
        self.max_focus = config.getint('max_focus', 255)

        # read camera position from config and ensure it is 4 values (x,y,z)
        self.camera_position = ensure_xyz(config.getintlist('camera_position'))

        # read what axis will contribute to focus
        # we could use all 3, but some axis that are say parallel to the camera don't
        # actually affect focus so they can be set to 0
        self.focal_axis = ensure_xyz(config.getfloatlist('focal_axis', [1.0, 1.0, 1.0]))

        self.distances = []
        self.focals = []
        fmlist = config.getlists('focus_mappings', [], seps=(',',), count=None, parser=parse_focal, note_valid=True)
        for x in fmlist:
            self.distances.append(x[0])
            self.focals.append(x[1])


        self.position = [0.0, 0.0, 0.0]
        self.focus = 50
        self.enable_focus_control = False
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
        self.printer.register_event_handler("homing:home_rails_end",
                                            self.handle_home_rails_end)

        # machine position from kinematics driver
        self.position = [0.0, 0.0, 0.0]
        self.toolhead = self.printer.lookup_object('toolhead')
        self.kin = self.toolhead.get_kinematics()

        if len(self.distances) > 1:
            self.build_focus_mapper()

        # default to auto-focus on while we are not homed
        self._focus_auto(True)

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

    def handle_home_rails_end(self, homing_state, rails):
        self.try_enable_focus_control()

    def home_status(self):
        # note: we only care about axis that are involved in the focal_axis setting
        curtime = self.printer.get_reactor().monotonic()
        kin_status = self.toolhead.get_kinematics().get_status(curtime)
        home_status = kin_status['homed_axes']
        x_ready = (self.focal_axis[0] == 0.0) or ('x' in home_status)
        y_ready = (self.focal_axis[1] == 0.0) or ('y' in home_status)
        z_ready = (self.focal_axis[2] == 0.0) or ('z' in home_status)
        return x_ready and y_ready and z_ready

    def try_enable_focus_control(self):
        if self.focus_mapper is None:
            return False

        if self.home_status():
            # we have the components we need for position derived auto-focus
            self.enable_focus_control = True
            self.gcode.respond_raw('enabling position derived webcam focus')
            self._focus_auto(False)
            return True
        else:
            return False

    def _pollStepper(self, eventtime):
        # get raw stepper counts (target position)
        kin_spos = {s.get_name(): s.get_commanded_position()
                    for s in self.kin.get_steppers()}
       
        # covert stepper counts to machine coordinates
        self.position = self.kin.calc_position(kin_spos)
        
        return eventtime + 0.5

    def _updateFocus(self, eventtime):
        if self.enable_focus_control and self.focus_mapper is not None:
            dist = self.distance()
            focus = self.focus_mapper(dist)
            focus = max(self.min_focus, min(self.max_focus, focus))
            if self.focus != focus:
                self.focus = focus
                #self.gcode.respond_raw('Focus {0} <= {1}'.format(focus, dist))
                self._focus(focus)

        return eventtime + UPDATE_TIME

    def distance(self, position = None):
        if position is None:
            position = self.position
        while len(position) < 3:
            position.append(0.0)

        # compute distance vector
        x = (position[0] - self.camera_position[0]) * self.focal_axis[0]
        y = (position[1] - self.camera_position[1]) * self.focal_axis[1]
        z = (position[2] - self.camera_position[2]) * self.focal_axis[2]

        # use pythagoreans theorum to get the distance to camera
        dist = math.sqrt(x*x + y*y + z*z)
        return dist

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
        dist=gcmd.get_float('D', None)
        if dist is not None:
            # only show mapping for given distance
            f = self.focus_mapper(dist)
            self.gcode.respond_raw('focal distance %f value %d' % (dist, f))
            return

        points=[]
        for d,f in zip(self.distances, self.focals):
            points.append('{}:{}'.format(d, f))
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
        axes_max = self.kin.axes_max
        axes_min = self.kin.axes_min
        ymin=gcmd.get_int('Y_MIN', axes_min[1])
        ymax=gcmd.get_int('Y_MAX', axes_max[1])
        ystep=gcmd.get_int('Y_STEP', (ymax - ymin) / 10)
        move_speed=gcmd.get_int('MOVE_SPEED', 300)

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
                d = self.distance(point)
                self.gcode.respond_raw('  focus {}:{}'.format(d, focus))
                self.distances.append(d)
                self.focals.append(focus)


        if len(self.distances) > 1:
            self.build_focus_mapper()
            self.try_enable_focus_control()

        self.gcode.respond_raw('finished auto-focus calibration')



    def _control(self, key, value):
        subprocess.call(['v4l2-ctl -d {} -c {}={}'.format(self.video_dev, key, str(value))], shell=True)

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
