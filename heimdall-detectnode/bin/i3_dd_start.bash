#!/usr/bin/env bash

MYDIR=`dirname $0`
PKGDIR=`dirname $MYDIR`

ps cax | grep i3status > /dev/null
if [ $? -ne 0 ]; then
    echo "You should be running the i3 Window Manager to use this script!"
    exit 1
fi

#Append i3 layout:

i3-msg "workspace 4; append_layout ${PKGDIR}/i3-layouts/dd_image_views_wt.json"
sleep 1

#Launch some handy terminals (use -c to automatically execute a command):
#gnome-terminal -t "dd_terminal_0" &> /dev/null &
#gnome-terminal -t "dd_terminal_1" &> /dev/null &

#Launch:
roslaunch heimdall_detectnode dd_image_views.launch
