#!/bin/bash

MICRO_ID=`xinput | grep MicroTouch | cut -f 2 | cut -d'=' -f2`
WACOM_ID=`xinput | grep "Finger touch" | cut -f 2 | cut -d'=' -f2`

xinput --map-to-output $MICRO_ID VGA1
xinput --map-to-output $WACOM_ID LVDS1

