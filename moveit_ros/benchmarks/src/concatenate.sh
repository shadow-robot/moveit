#!/bin/bash
# Usage/ ./concatenate.sh
# https://stackoverflow.com/questions/7333232/how-to-concatenate-two-mp4-files-using-ffmpeg
# https://unix.stackexchange.com/questions/427237/what-happens-if-we-use-touch-command-while-the-file-already-exists
# https://stackoverflow.com/questions/1848415/remove-slash-from-the-end-of-a-variable

OUTPUTFOLDER=${1:-$(pwd)/../concatenated/} #default concatenated/
mkdir -p ${OUTPUTFOLDER}

clear
echo
echo
echo
echo
echo
clear
echo "OUTPUT FOLDER = $OUTPUTFOLDER"

for dir1 in */ ; do
		echo $dir1 && \
		cd "$dir1" && pwd && FILEARRAY=(*.mp4) && \
		if [ -f ./mylist.txt ]; then \
    		rm ./mylist.txt ; \
		fi && \
		touch mylist.txt && \
		for i in "${FILEARRAY[@]}" ; do \
			echo "file '$i'" >> ./mylist.txt ;
		done && \
		cat mylist.txt && \
		echo "${OUTPUTFOLDER}${dir1%/}.mp4" && \
		ffmpeg -y -f concat -i mylist.txt -c copy "${OUTPUTFOLDER}${dir1%/}.mp4" && \
		cd ../ && pwd ;
done
