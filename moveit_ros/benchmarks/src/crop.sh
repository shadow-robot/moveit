#!/bin/bash
# Usage: ./crop.sh
# https://video.stackexchange.com/questions/19413/crop-selected-area-in-all-videos
# https://video.stackexchange.com/questions/4563/how-can-i-crop-a-video-with-ffmpeg
# https://ffmpeg.org/ffmpeg-filters.html#crop

######################## SETTINGS ##########################
# $1=folder of the outputs, stands for the arg passed when executing './videorecordmoves.sh arg'
OUTPUTFOLDER=$1 #default cropped

# https://ezgif.com/crop-video to figure out the params!
LEFT=$2 #default 620pix
WIDTH=$3 #default 792pix
TOP=$4 #default 74pix
HEIGHT=$5 #default 976pix
############################################################

mkdir -p ${OUTPUTFOLDER:=cropped}
for file in *.mp4;
do
  ffmpeg -y -i "${file}" -vf crop=${WIDTH:=792}:${HEIGHT:=976}:${LEFT:=620}:0${TOP:=74} -c:a copy "./$OUTPUTFOLDER/$file"
done
