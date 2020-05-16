#!/bin/bash
# super sophisticated control script for nearly everything - here creating nice videos from recordings
# written by Marcel Ebbrecht <marcel.ebbrecht@googlemail.com>

# preamble
LANG=C
PATH=$PATH:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games

### functions
# hello world
function superSophisticatedHello {
    echo
    echo "Welcome to our super sophisticated video conversion script"
    echo
}

# help message
function helpMain {
    echo "usage: stitch.sh help"
    echo "                 run PATH_WITH_RECORDINGS"
}

### execution
superSophisticatedHello

# exit when no parameter is given
if [[ $# -le 0 ]]; then
    helpMain
    echo
    exit 1
fi

# first parameter
case $1 in
    run)
        # check for files
        if ! test -f $2/cam.avi; then
            echo "File $2/cam.avi not existant, please check given path"
            echo
            exit 1
        fi

        if ! test -f $2/rviz.avi; then
            echo "File $2/rviz.avi not existant, please check given path"
            echo
            exit 1
        fi

	# calculate factor
	CAMLENGTH=$(ffmpeg -i $2/cam.avi 2>&1 | grep "Duration"| cut -d ' ' -f 4 | sed s/,// | awk '{ split($1, A, ":"); split(A[3], B, "."); print 360000*A[1] + 6000*A[2] + 100*B[1] + B[2]}')
	RVIZLENGTH=$(ffmpeg -i $2/rviz.avi 2>&1 | grep "Duration"| cut -d ' ' -f 4 | sed s/,// | awk '{ split($1, A, ":"); split(A[3], B, "."); print 360000*A[1] + 6000*A[2] + 100*B[1] + B[2] }')
	NEWLENGTH=$(python -c "print($RVIZLENGTH*1.0/$CAMLENGTH*1.0)" | cut -b -6)

	# convert videos
	ffmpeg -i $2/cam.avi -filter:v "setpts=$NEWLENGTH*PTS" -c:v libx264 $2/cam.mp4
	ffmpeg -i $2/rviz.avi -c:v libx264 $2/rviz.mp4

        # stitch videos
        ffmpeg \
          -i $2/cam.mp4 \
          -i $2/rviz.mp4 \
          -filter_complex '[0:v]pad=iw*2:ih[int];[int][1:v]overlay=W/2:0[vid]' \
          -map [vid] \
          -c:v libx264 \
          -crf 23 \
          -preset veryfast \
          $2/stitch.mp4
    ;;
    *)
        helpMain
    ;;
esac
