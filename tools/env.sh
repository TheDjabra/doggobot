# Source this to get a working ROS2 environment inside the Doggobot container
# from a NON-interactive shell (scripts, launchers, systemd units).
#
# Why this file exists instead of just calling `source_ros2`: that is a shell
# function defined in the image's /home/scripts/bashrc_docker.sh, and shell
# functions and aliases do not exist in a child script. Sourcing that file
# instead would work but it ends with `export ROS_DOMAIN_ID=96`, which would
# silently move this project onto the wrong domain.
#
# Usage:  source /home/projects/ros2_ws/src/doggobot/tools/env.sh

export ROS_DOMAIN_ID=66

export _DOGGOBOT_ROS_DISTRO="${ROS_DISTRO:-jazzy}"
source "/opt/ros/${_DOGGOBOT_ROS_DISTRO}/setup.bash"

# Driver workspaces. Skipping these is the classic failure: the packages build
# and the nodes run, but `multi_cam` and the VESC packages are invisible because
# only ros2_ws was ever sourced.
for ws in \
    /home/projects/sensor2_ws/src/cameras/oakd/install/setup.bash \
    /home/projects/sensor2_ws/src/imu/artemis_openlog/install/setup.bash \
    /home/projects/sensor2_ws/src/vesc/install/setup.bash \
    /home/projects/sensor2_ws/src/lidars/ld06/ros2/install/setup.bash \
    /home/projects/rosboard_ws/install/setup.bash ; do
    [ -f "$ws" ] && source "$ws"
done

# Our own workspace last, so it wins on any name collision.
source /home/projects/ros2_ws/install/setup.bash
