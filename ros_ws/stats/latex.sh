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
# Format: COUNT;DIVT;DIVD
#     COUNT Number of datasets to use on Plots
#     DIVT  Steps for plots by time
#     DIVD  Steps for plots by distance
SETS="
800;8;40
400;4;20
200;2;10
100;1;5
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
        
        # create variables for main
        PH_INCLUDES=""
        PH_DATAITEMS="\\n"
        PH_SETS="\\n"

        # create list for datasets
        for DATAFILE in $DATAFILES; do
            FILENAME="$(echo $DATAFILE | cut -d ";" -f 1)"
            TITLE="$(echo $DATAFILE | cut -d ";" -f 3 | sed 's/_/ /g')"
            DESC="$(echo $DATAFILE | cut -d ";" -f 4 | sed 's/_/ /g')"
            PH_DATAITEMS=$PH_DATAITEMS"\\\item \\\textbf{$TITLE:} $DESC\\n"
        done

        # create list for setsizes
        for SET in $SETS; do
            SIZE="$(echo $SET | cut -d ";" -f 1)"
            PH_SETS=$PH_SETS"\\\item $SIZE Messwerte\\n"
        done
        
        # create subsections from single
        rm *.tex > /dev/null 2>&1
        for DATAFILE in $DATAFILES; do
            TITLEFILE="$(echo $DATAFILE | cut -d ";" -f 3)"
            TITLE="$(echo $DATAFILE | cut -d ";" -f 3 | sed 's/_/ /g')"
            DESC="$(echo $DATAFILE | cut -d ";" -f 4 | sed 's/_/ /g')"
            for SET in $SETS; do
                SIZE=$(echo $SET | cut -d ";" -f 1)
                FILENAME="latex-data-$TITLEFILE-$SIZE"
                DIVT=$(echo $SET | cut -d ";" -f 2)
                DIVD=$(echo $SET | cut -d ";" -f 3)
                cat tex/section.tex \
                    | sed "s&___PH_DESC___&$DESC&g" \
                    | sed "s&___PH_SIZE___&$SIZE&g" \
                    | sed "s&___PH_DIVD___&$DIVD&g" \
                    | sed "s&___PH_DIVT___&$DIVT&g" \
                    | sed "s&___PH_FILENAME___&$FILENAME&g" \
                    | sed "s&___PH_TITLE___&$TITLE&g" \
                    > section-$TITLEFILE-$SIZE.tex
                PH_INCLUDES=$PH_INCLUDES"\\\section {$DESC - $SIZE}\\n\\\include{section-$TITLEFILE-$SIZE}\\n"
            done      
        done
        
        # create main tex and replace variables
        cat tex/main.tex | sed "s&___PH_DATAITEMS___&$PH_DATAITEMS&" | sed "s&___PH_SETS___&$PH_SETS&" | sed "s&___PH_INCLUDES___&$PH_INCLUDES&" > report.tex
        #pdflatex report.tex > /dev/null 2>&1
        pdflatex report.tex
    ;;
    *)
        helpMain
    ;;
esac
