#!/bin/sh -e
#
# Set up OpenTRV Arduino IDE environment and verify compilation.
# Exits 0 on success.
# Refer to https://github.com/arduino/Arduino/blob/master/build/shared/manpage.adoc for arduino CLI docs.
#
# Script assumes that required libraries (OTRadioLink and OTAESGCM) are already installed in $HOME/Arduino/libs.
# Dependencies:
#   firmware:
#       OpenTRV-Arduino-V0p2 : https://github.com/opentrv/OpenTRV-Arduino-V0p2
#   board config:
#       OpenTRV V0p2 board: https://github.com/opentrv/OpenTRV-Config/tree/master/Arduino
#   libs:
#       OTRadioLink: https://github.com/opentrv/OTRadioLink
#       OTAESGCM: https://github.com/opentrv/OTAESGCM
#
#
# NOTE!!! Arduino IDE requires an X server to run, even in CLI mode.
# See https://github.com/arduino/Arduino/blob/master/build/shared/manpage.adoc#bugs for instructions on how to set up a dummy X server in Linux.

echo Test compilation of 2-way comms demo.

BUILD_TARGET=opentrv:avr:opentrv_v0p2  # Target board to build for.
SKETCH_PATH=$PWD  # Path to sketch directory

# List of hardware configurations to be tested.
# Partial paths to the individual .ino files under $SKETCH_PATH.
PARTPATHS="
    OT2WayGateway/OT2WayGateway.ino
    OT2WayTRV/OT2WayTRV.ino
    "

# Loop through and test each configuration.
# Note which one is being tested to make clear which one has failed, if any.
for pp in $PARTPATHS;
do
    echo "@@@@@@" Testing $SKETCH_PATH/$pp
    arduino --verify --board $BUILD_TARGET $SKETCH_PATH/$pp || exit 2
done

exit 0
