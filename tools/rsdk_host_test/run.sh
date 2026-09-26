#!/bin/sh
# Host tests for the DJI R SDK backend (no ESP32 needed). Run from anywhere:
#   sh tools/rsdk_host_test/run.sh
set -e
cd "$(dirname "$0")"
SRC=../../src
g++ -std=gnu++17 -Wall -o /tmp/rsdk_protocol_test protocol_test.cpp $SRC/dji_rsdk_protocol.cpp
/tmp/rsdk_protocol_test
g++ -std=gnu++17 -Wno-format -Istub -I$SRC -o /tmp/rsdk_sim_test sim_test.cpp \
    $SRC/dji_rsdk_camera.cpp $SRC/dji_rsdk_protocol.cpp $SRC/dji_duml_transport.cpp
/tmp/rsdk_sim_test | grep -v '^\['
