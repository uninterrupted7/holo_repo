#!/bin/bash
# vessel_teleop_wrapper.sh
# ========================
# Wrapper to launch vessel_teleop in a new gnome-terminal window
# so it gets a real TTY for keyboard input.
#
# All arguments are the full command line from ros2 launch:
#   <executable_path> --ros-args -r __node:=vessel_teleop ...
#
# This script is called as: bash vessel_teleop_wrapper.sh <cmd> <args...>
# It opens a new terminal and runs the command there.

export LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0

CMD="$@"

LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0 gnome-terminal -- bash -c "
    export LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0
    exec $CMD
"
