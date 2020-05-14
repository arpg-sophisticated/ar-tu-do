#!/bin/bash
# super sophisticated control script for nearly everything - here creating nice graphs with latex
# written by Marcel Ebbrecht <marcel.ebbrecht@googlemail.com>

# preamble
LANG=C
PATH=$PATH:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games

### settings
# set to 1 after successfull configuration
CONFIGURED="0"

# Provide data files seperated by spaces or linebreaks
# Format: FILE;COLOR;TITLE;DESCR
#     FILE  Filename to include
#     COLOR red,blue,green,yellow,black,purple or anything, that latex supports
#     TITLE Title to use in legend (use _ as spaces)
#     DESCR Description used in Captions and TOCs (use _ as spaces)
DATAFILES="
sim-current_20200514-155846/speed_over_time.dat;green;WF5;Wallfollowing_5_(aktuelle_Entwicklungsversion)
sim-old_20200514-155328/speed_over_time.dat;blue;WF2;Wallfollowing_2_(alte_Version)
"
# Timesets to use for plots seperated by spaces or linebreaks
# Please ensure, that the data files hold enough datasets
# Format: COUNT;DIVT;DIVD;SMOO
#     COUNT Number of datasets to use on Plots
#     DIVT  Steps for plots by time
#     DIVD  Steps for plots by distance
#     SMOO  Smoothing: 0 for no, higher values smoothes stronger
SETS="
800;80;80;5
400;40;40;3
200;20;20;1
100;10;10;0
"

### functions
# hello world
function superSophisticatedHello {
    echo
    echo "Welcome to our super sophisticated latex plot script"
    echo
}

# help message
function helpMain {
    echo "usage: latex.sh help"
    echo "                build"
    echo
    echo "Please check configuration in this script prior running build"
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
    build)
        # check if files exist
        for DATAFILE in $DATAFILES; do
            if ! test -f $(echo $DATAFILE | cut -d ";" -f 1); then
                echo "File $(echo $DATAFILE | cut -d ";" -f 1) not found"
                echo
                exit 1
            fi  
        done

        # check if files contain enough datasets
        for DATAFILE in $DATAFILES; do
            for SET in $SETS; do
                if [[ $(($(wc -l $(echo $DATAFILE | cut -d ";" -f 1) | cut -d " " -f 1))) -le $(echo $SET | cut -d ";" -f 1) ]]; then
                    echo "File $(echo $DATAFILE | cut -d ";" -f 1): not enough data, max available: $(($(wc -l $(echo $DATAFILE | cut -d ";" -f 1) | cut -d " " -f 1)-1))"
                    echo
                    exit 1
                fi        
            done      
        done      

        # create datafiles
        rm latex-data*.dat > /dev/null 2>&1
        for DATAFILE in $DATAFILES; do
            FILENAME=$(echo $DATAFILE | cut -d ";" -f 1)
            TITLE=$(echo $DATAFILE | cut -d ";" -f 3)
            for SET in $SETS; do
                SIZE=$(echo $SET | cut -d ";" -f 1)
                head -$(($SIZE+1)) $FILENAME > latex-data-$TITLE-$SIZE.dat
            done      
        done  
        

        
#head -$(($4+1)) $2 > new.dat
        #head -$(($4+1)) $3 > old.dat
        #pdflatex report.tex > /dev/null 2>&1
    ;;
    *)
        helpMain
    ;;
esac
