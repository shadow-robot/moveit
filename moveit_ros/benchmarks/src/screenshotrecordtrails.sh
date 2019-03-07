#!/bin/bash
# Usage/ ./stickOneAtATime.sh

# This shows that the trail could not be enabled/disabled from any c++ script, so process to get it will be to replace record with screenshot and will unfortunately be as time-costly
# https://answers.ros.org/question/231731/how-to-update-a-robot-visual-enable-check-box-in-rviz-from-a-running-node-c/

# I choose shutter for its .svg/pdf output ability
# https://www.cyberciti.biz/open-source/linux-screenshot-program-tool/
# https://unix.stackexchange.com/questions/174168/shutter-scripts
# https://askubuntu.com/questions/260226/how-to-fix-shutter
# https://askubuntu.com/questions/862377/when-i-capture-a-screenshot-with-shutter-it-opens-automatically
# NOTE THAT Without specifying the window borders it will always open a GUI and bother the process
# NOTE THAT in my shutter software version, fixing the x axis window origin is buggy so I take the whole width of the screen unfortunately 

if [[ $# -eq 0 ]] ; then #no arg provided
	echo 'Require to pass an extra filename when calling this bash'
	exit 0
fi
# $1=name of the file, stands for the arg passed when executing './videorecordmoves.sh arg'
OUTPUTNAME=$1

######################## SETTINGS ##########################
EXTENSION=".pdf"
DIR="/tmp/trails/"
# axis x = from top left pointing to the right
SHIFT_TOP_LEFT_POS_x=1 #it doesnt accept 0 !!!
# axis y = from top left pointing to the bottom
SHIFT_TOP_LEFT_POS_y=390
WIDTHINPIX=1920
HEIGHTINPIX=660
############################################################

mkdir -p $DIR

OUTPUT="$DIR$OUTPUTNAME$EXTENSION"

######################## COMMANDS ##########################

shutter -s=[$SHIFT_TOP_LEFT_POS_x,$SHIFT_TOP_LEFT_POS_y,$WIDTHINPIX,$HEIGHTINPIX] -e -o $OUTPUT &
