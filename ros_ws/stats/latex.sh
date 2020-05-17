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
gazebo_20200517-wf2/speed_over_time.dat;blue;WF2;Wallfollowing_2_(alte_Version)
gazebo_20200517-wf5/speed_over_time.dat;green;WF5;Wallfollowing_5_(aktuelle_Entwicklungsversion)
"
#sim-current-20200515/speed_over_time.dat;green;WF5;Wallfollowing_5_(aktuelle_Entwicklungsversion)
#sim-old-20200515/speed_over_time.dat;blue;WF2;Wallfollowing_2_(alte_Version)

# Timesets to use for plots seperated by spaces or linebreaks
# Please ensure, that the data files hold enough datasets
# Format: COUNT;DIVT;DIVD
#     COUNT Number of datasets to use on Plots
#     DIVT  Steps for plots by time
#     DIVD  Steps for plots by distance
#2000;20;100;20
#1000;10;50;10
SETS="
500;5;25;5
200;2;10;2
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
            SMOOTH="$(echo $SET | cut -d ";" -f 4)"
            PH_SETS=$PH_SETS"\\\item $SIZE Messwerte, Glättung: $SMOOTH\\n"
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
                SMOOTH=$(echo $SET | cut -d ";" -f 4)
		AVERAGE=$(head -2 $FILENAME.dat | tail -1 | cut -d " " -f 3)

                cat tex/section.tex \
                    | sed "s&___PH_DESC___&$DESC&g" \
                    | sed "s&___PH_SIZE___&$SIZE&g" \
                    | sed "s&___PH_DIVD___&$DIVD&g" \
                    | sed "s&___PH_DIVT___&$DIVT&g" \
                    | sed "s&___PH_SMOOTH___&$SMOOTH&g" \
                    | sed "s&___PH_AVERAGE___&$AVERAGE&g" \
                    | sed "s&___PH_FILENAME___&$FILENAME&g" \
                    | sed "s&___PH_TITLE___&$TITLE&g" \
                    > section-$TITLEFILE-$SIZE.tex
                PH_INCLUDES=$PH_INCLUDES"\\\include{section-$TITLEFILE-$SIZE}\\n"
            done      
        done

        # create comparisons
        for SET in $SETS; do
            PH_PLOT1=""
            PH_LEGEND1=""
            PH_PLOT2=""
            PH_LEGEND2=""
            PH_PLOT3=""
            PH_LEGEND3=""
            PH_PLOT4=""
            PH_LEGEND4=""
            PH_PLOT5=""
            PH_LEGEND5=""
            PH_PLOT6=""
            PH_LEGEND6=""
            PH_PLOT7=""
            PH_LEGEND7=""
            PH_PLOT8=""
            PH_LEGEND8=""

            SIZE=$(echo $SET | cut -d ";" -f 1)
            DIVT=$(echo $SET | cut -d ";" -f 2)
            DIVD=$(echo $SET | cut -d ";" -f 3)
            SMOOTH=$(echo $SET | cut -d ";" -f 4)
            for DATAFILE in $DATAFILES; do
                COLOR="$(echo $DATAFILE | cut -d ";" -f 2)"
                TITLEFILE="$(echo $DATAFILE | cut -d ";" -f 3)"
                TITLE="$(echo $DATAFILE | cut -d ";" -f 3 | sed 's/_/ /g')"
                DESC="$(echo $DATAFILE | cut -d ";" -f 4 | sed 's/_/ /g')"
                FILENAME="latex-data-$TITLEFILE-$SIZE"
		AVERAGE=$(head -2 $FILENAME.dat | tail -1 | cut -d " " -f 3)

                PH_PLOT1=$PH_PLOT1"\\\addplot[smooth,$COLOR,solid] table [y=speed,x=distance]{$FILENAME.dat};\\n"
                PH_LEGEND1=$PH_LEGEND1"\\\addlegendentry{$TITLE \$v_{cur}(d)\$}\\n"

                PH_PLOT5=$PH_PLOT5"\\\addplot[smooth,$COLOR,solid,each nth point=___PH_SMOOTH___, filter discard warning=false, unbounded coords=discard] table [y=speed,x=distance]{$FILENAME.dat};\\n"
                PH_LEGEND5=$PH_LEGEND5"\\\addlegendentry{$TITLE \$v_{cur}(d)\$}\\n"

                PH_PLOT2=$PH_PLOT2"\\\addplot[smooth,$COLOR,solid] table [y=speed,x=time]{$FILENAME.dat};\\n"
                PH_LEGEND2=$PH_LEGEND2"\\\addlegendentry{$TITLE \$v_{cur}(t)\$}\\n"

                PH_PLOT6=$PH_PLOT6"\\\addplot[smooth,$COLOR,solid,each nth point=___PH_SMOOTH___, filter discard warning=false, unbounded coords=discard] table [y=speed,x=time]{$FILENAME.dat};\\n"
                PH_LEGEND6=$PH_LEGEND6"\\\addlegendentry{$TITLE \$v_{cur}(t)\$}\\n"

                PH_PLOT3=$PH_PLOT3"\\\addplot[smooth,$COLOR,solid] table [y=acceleration,x=distance]{$FILENAME.dat};\\n"
                PH_LEGEND3=$PH_LEGEND3"\\\addlegendentry{$TITLE \$a(d)\$}\\n"

                PH_PLOT7=$PH_PLOT7"\\\addplot[smooth,$COLOR,solid,each nth point=___PH_SMOOTH___, filter discard warning=false, unbounded coords=discard] table [y=acceleration,x=distance]{$FILENAME.dat};\\n"
                PH_LEGEND7=$PH_LEGEND7"\\\addlegendentry{$TITLE \$a(d)\$}\\n"

                PH_PLOT4=$PH_PLOT4"\\\addplot[smooth,$COLOR,solid] table [y=acceleration,x=time]{$FILENAME.dat};\\n"
                PH_LEGEND4=$PH_LEGEND4"\\\addlegendentry{$TITLE \$a(t)\$}\\n"

                PH_PLOT8=$PH_PLOT8"\\\addplot[smooth,$COLOR,solid,each nth point=___PH_SMOOTH___, filter discard warning=false, unbounded coords=discard] table [y=acceleration,x=time]{$FILENAME.dat};\\n"
                PH_LEGEND8=$PH_LEGEND8"\\\addlegendentry{$TITLE \$a(t)\$}\\n"
            done  

            cat tex/compare.tex \
                | sed "s&___PH_DESC___&$DESC&g" \
                | sed "s&___PH_AVERAGE___&$AVERAGE&g" \
                | sed "s&___PH_SIZE___&$SIZE&g" \
                | sed "s&___PH_DIVD___&$DIVD&g" \
                | sed "s&___PH_DIVT___&$DIVT&g" \
                | sed "s&___PH_FILENAME___&$FILENAME&g" \
                | sed "s&___PH_TITLE___&$TITLE&g" \
                | sed "s&___PH_PLOT1___&$PH_PLOT1&g" \
                | sed "s&___PH_LEGEND1___&$PH_LEGEND1&g" \
                | sed "s&___PH_PLOT2___&$PH_PLOT2&g" \
                | sed "s&___PH_LEGEND2___&$PH_LEGEND2&g" \
                | sed "s&___PH_PLOT3___&$PH_PLOT3&g" \
                | sed "s&___PH_LEGEND3___&$PH_LEGEND3&g" \
                | sed "s&___PH_PLOT4___&$PH_PLOT4&g" \
                | sed "s&___PH_LEGEND4___&$PH_LEGEND4&g" \
                | sed "s&___PH_PLOT5___&$PH_PLOT5&g" \
                | sed "s&___PH_LEGEND5___&$PH_LEGEND5&g" \
                | sed "s&___PH_PLOT6___&$PH_PLOT6&g" \
                | sed "s&___PH_LEGEND6___&$PH_LEGEND6&g" \
                | sed "s&___PH_PLOT7___&$PH_PLOT7&g" \
                | sed "s&___PH_LEGEND7___&$PH_LEGEND7&g" \
                | sed "s&___PH_PLOT8___&$PH_PLOT8&g" \
                | sed "s&___PH_LEGEND8___&$PH_LEGEND8&g" \
                | sed "s&___PH_SMOOTH___&$SMOOTH&g" \
                > compare-$SIZE.tex
            PH_INCLUDES=$PH_INCLUDES"\\\include{compare-$SIZE}\\n" 
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
