#!/bin/bash
# Usage: ./stick.sh
# https://unix.stackexchange.com/questions/86722/how-do-i-loop-through-only-directories-in-bash

OUTPUTFOLDER=$1 #default sticked
mkdir -p ${OUTPUTFOLDER:=/home/exs/Videos/merge/sticked/}

for dir1 in merge/*/ ; do
	( cd "$dir1" && pwd && for dir2 in */ ; do ( cd "$dir2" && pwd && FILEARRAY=(*.mp4) && ffmpeg -y -i "${FILEARRAY[0]}" -i "${FILEARRAY[1]}" -filter_complex hstack -c:v libx264 -crf 0 "${OUTPUTFOLDER}${FILEARRAY[0]}" &) ; done)
done
