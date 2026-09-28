#!/bin/sh
# Host test for the OSD element formatter (no ESP32 needed). Checks that
# src/osd_format.cpp, docs/osd-format.js and the copy embedded in
# src/web_assets.h render identical Custom Message text. Run from anywhere:
#   sh tools/osd_host_test/run.sh        (needs g++ and node)
set -e
cd "$(dirname "$0")"
SRC=../../src
g++ -std=gnu++17 -Wall -Wno-format-truncation -I../rsdk_host_test/stub -I$SRC -o /tmp/osd_format_test osd_test.cpp $SRC/osd_format.cpp
node compare.js /tmp/osd_format_test
