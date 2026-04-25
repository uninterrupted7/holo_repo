#!/bin/bash
# rviz2_wrapper.sh
# ================
# Wrapper script for RViz2 to fix snap core20 library conflict.
# The snap core20 library has an incompatible libpthread.so.0 that
# causes: "undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE"
#
# This script forces the system libpthread to be loaded first.

export LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0
exec /opt/ros/humble/bin/rviz2 "$@"
