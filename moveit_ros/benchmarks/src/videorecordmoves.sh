#!/bin/bash
if [[ $# -eq 0 ]] ; then #no arg provided
	echo 'Require to pass an extra filename when calling this bash'
	exit 0
fi
# $1=name of the file, stands for the arg passed when executing './videorecordmoves.sh arg'
OUTPUTNAME=$1

######################## SETTINGS ##########################
FRAMERATE=$2 #default 60 fps
EXTENSION=".mp4"
DIR="/tmp/"
# axis x = from top left pointing to the right
SHIFT_TOP_LEFT_POS_x=0
# axis y = from top left pointing to the bottom
SHIFT_TOP_LEFT_POS_y=0
WIDTHINPIX=1920
HEIGHTINPIX=1080
############################################################

OUTPUT="$DIR$OUTPUTNAME$EXTENSION"
LISTENER="${DIR}stop" #don't care about the extension #https://stackoverflow.com/questions/9722624/how-to-stop-ffmpeg-remotely
LOG="${DIR}ShellOut.log"

# Optional:
if [[ -e $LOG ]]; then
	rm $LOG
fi

# Non optional I think:
if [[ -e $LISTENER ]]; then
	rm $LISTENER
fi

######################## COMMANDS ##########################
echo $OUTPUT
touch $LISTENER
<$LISTENER ffmpeg -y -video_size ${WIDTHINPIX}x$HEIGHTINPIX -framerate ${FRAMERATE:=60} -f x11grab -i :0.0+$SHIFT_TOP_LEFT_POS_x,$SHIFT_TOP_LEFT_POS_y $OUTPUT #>/dev/null 2>>${DIR}Capture.log &
############################################################
# https://trac.ffmpeg.org/wiki/Capture/Desktop


