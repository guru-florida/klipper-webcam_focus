#!/bin/bash
KLIPPER_PATH="${HOME}/klipper"
SYSTEMDDIR="/etc/systemd/system"

# Step 1:  Verify Klipper has been installed
check_klipper()
{
    if [ "$(sudo systemctl list-units --full -all -t service --no-legend | grep -F "klipper.service")" ]; then
        echo "Klipper service found!"
    else
        echo "Klipper service not found, please install Klipper first"
        exit -1
    fi

}

# Step 2: link extension to Klipper
link_extension()
{
    echo "Linking extension to Klipper..."
    ln -sf "${SRCDIR}/webcam_focus.py" "${KLIPPER_PATH}/klippy/extras/webcam_focus.py"
}

# Step 3: Add updater
# webcamd to moonraker.conf
echo -e "Adding update manager to moonraker.conf"

update_section=$(grep -c '\[update_manager webcam_focus\]' \
${HOME}/klipper_config/moonraker.conf || true)
if [ "${update_section}" -eq 0 ]; then
  echo -e "\n" >> ${HOME}/klipper_config/moonraker.conf
  while read -r line; do
    echo -e "${line}" >> ${HOME}/klipper_config/moonraker.conf
  done < "$PWD/file_templates/moonraker_update.txt"
  echo -e "\n" >> ${HOME}/klipper_config/moonraker.conf
else
  echo -e "[update_manager webcam_focus] already exist in moonraker.conf [SKIPPED]"
fi



# Step 4: restarting Klipper
restart_klipper()
{
    echo "Restarting Klipper..."
    sudo systemctl restart klipper
}

# Helper functions
verify_ready()
{
    if [ "$EUID" -eq 0 ]; then
        echo "This script must not run as root"
        exit -1
    fi
}

# Force script to exit if an error occurs
set -e

# Find SRCDIR from the pathname of this script
SRCDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/src/ && pwd )"

# Parse command line arguments
while getopts "k:" arg; do
    case $arg in
        k) KLIPPER_PATH=$OPTARG;;
    esac
done

# Run steps
verify_ready
link_extension
restart_klipper
