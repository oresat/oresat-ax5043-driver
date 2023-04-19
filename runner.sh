#!/bin/sh

set -e
scp $1 pi@raspberrypi.local:
ssh -t pi@raspberrypi.local /home/pi/$(basename $1)
