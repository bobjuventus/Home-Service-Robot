#!/bin/sh
xterm -e " rosrun pick_objects pick_objects" &
sleep 1
xterm -e " rosrun home_service home_service"