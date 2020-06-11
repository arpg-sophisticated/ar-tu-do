#!/bin/bash
# super sophisticated control script for nearly everything - main
# written by Marcel Ebbrecht <marcel.ebbrecht@googlemail.com>

# preamble
LANG=C
PATH=$PATH:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games

# functions
. ./toad.functions


# execution
superSophisticatedHello

# exit when no config exists
if ! test -f ./toad.settings; then
    echo "no settings file found, exiting"
    echo
    exit 1
fi

# load settings
. ./toad.settings

# exit when no config exists
if [[ $CONFIGURED != "1" ]]; then
    echo "settings not configured, exiting"
    echo
    exit 1
fi

# exit when no parameter is given
if [[ $# -le 0 ]]; then
    toadHelpMain
    echo
    exit 1
fi

# main mode switch
case $1 in
    system)
        # exit when no second parameter is given
        if [[ $# -le 1 ]]; then
            toadHelpSystem
            echo
            exit 1
        fi
        case $2 in
            sshkeys)
                echo "Replacing local SSH keys, please wait ..."
                echo
                OLDBRANCH=$(getActiveBranch)
                git checkout $BRANCHSSH
                echo
                cd ../ssh && \
                    git pull && \
                    cat ../ssh/*.pub > ~/.ssh/authorized_keys
                echo
                git checkout $OLDBRANCH
            ;;
            rebuild)
                echo "Starting rebuild, please wait ..."
                echo
                OLDBRANCH=$(getActiveBranch)
                git checkout $BRANCHBUILD
                echo
                rm -fr ./build
                source $PATHROS
                source $PATHSETUP
                catkin_make --make-args -j $(($(cat /proc/cpuinfo  | grep processor | wc -l) + 1))
                echo
                git checkout $OLDBRANCH
            ;;
            resetbuild)
                if [[ "$3" != "cron" ]]; then
                    toadConfirmationRequest "This will delete and rebuild"
                    toadConfirmationRequest "WARNING: This will reset branch and remove any changes"
                fi
                echo "Starting rebuild, please wait ..."
                echo
                OLDBRANCH=$(getActiveBranch)
                git checkout $BRANCHBUILD
                git reset --hard
                git pull
                echo
                rm -fr ./build
                source $PATHROS
                source $PATHSETUP
                catkin_make --make-args -j $(($(cat /proc/cpuinfo  | grep processor | wc -l) + 1))
                echo
                git checkout $OLDBRANCH
            ;;
            run)
                if [[ $(ps aux | grep gdm | grep session | grep -v grep | grep $(whoami) | wc -l) -le 0 ]]; then
                    echo
                    echo "No running gdm-x-session process found, exiting"
                    echo
                    exit 1
                fi
                export DISPLAY=$(getActiveDisplay)
                ARGUMENTS="world:=$LAUNCHTRACK"
                if [[ "$3" =~ "nogui" ]]; then
                    ARGUMENTS="$ARGUMENTS gui:=false "
                fi
                if [[ "$3" =~ "fast" ]]; then
                    ARGUMENTS="$ARGUMENTS fast:=true "
                fi
                if [[ "$3" =~ "drive" ]]; then
                    ARGUMENTS="$ARGUMENTS mode_override:=2 "
                fi
                if [[ "$3" =~ "record" ]]; then
                    ARGUMENTS="$ARGUMENTS record:=true "
                fi
                if [[ "$3" =~ "videohd" ]]; then
                    ARGUMENTS="$ARGUMENTS videohd:=true "
                fi
                if [[ "$3" =~ "manual" ]]; then
                    ARGUMENTS="$ARGUMENTS mode_override:=1 "
                fi
                if [[ "$3" =~ "customtrack" ]]; then
                    if [[ "$4" =~ [a-z] ]]; then
                        ARGUMENTS="$ARGUMENTS world:=$4 "
                    else
                        ARGUMENTS="$ARGUMENTS world:=$LAUNCHTRACK "
                    fi
                fi
                source $PATHROS
                source $PATHSETUP
		RECORDTIME=$(date +%Y%m%d-%H%M%S)
		mkdir -p ../data/videos/$RECORDTIME
                roslaunch launch/$LAUNCHBUILD use_gpu:=$USEGPU $ARGUMENTS
		mv ~/.ros/output.avi ../data/videos/$RECORDTIME/rviz.avi > /dev/null 2>&1
		mv ~/.ros/output-cam.avi ../data/videos/$RECORDTIME/cam.avi > /dev/null 2>&1
		mv ./output.avi ../data/videos/$RECORDTIME/rviz.avi > /dev/null 2>&1
		mv ./output-cam.avi ../data/videos/$RECORDTIME/cam.avi > /dev/null 2>&1
            ;;
            *)
                toadHelpSystem
            ;;
        esac
    ;;
    car)
        # exit when no second parameter is given
        if [[ $# -le 1 ]]; then
            toadHelpCar
            echo
            exit 1
        fi
        case $2 in
            sshkeys)
                echo "Replacing local SSH keys, please wait ..."
                echo
                OLDBRANCH=$(getActiveBranch)
                git checkout $BRANCHSSH
                echo
                cd ../ssh && \
                    git pull && \
                    cat ../ssh/*.pub > ~/.ssh/authorized_keys
                echo
                git checkout $OLDBRANCH
            ;;
            rebuild)
                echo "Starting rebuild, please wait ..."
                echo
                OLDBRANCH=$(getActiveBranch)
                git checkout $BRANCHCAR
                git reset --hard
                echo
                rm -fr ./build
                source $PATHROS
                source $PATHSETUP
                catkin_make --make-args -j $(($(cat /proc/cpuinfo  | grep processor | wc -l) + 1))
                echo
                git checkout $OLDBRANCH
            ;;
            resetbuild)
                if [[ "$3" != "cron" ]]; then
                    toadConfirmationRequest "This will delete and rebuild"
                    toadConfirmationRequest "WARNING: This will reset branch and remove any changes"
                fi
                echo "Starting rebuild, please wait ..."
                echo
                OLDBRANCH=$(getActiveBranch)
                git checkout $BRANCHCAR
                git reset --hard
                git pull
                echo
                rm -fr ./build
                source $PATHROS
                source $PATHSETUP
                catkin_make --make-args -j $(($(cat /proc/cpuinfo  | grep processor | wc -l) + 1))
                echo
                git checkout $OLDBRANCH
            ;;
            run)
                source $PATHROS
                source $PATHSETUP
                MAINIPADDRESS=$(getAddressByInterface $CARINTERFACE)
		export ROS_IP=$MAINIPADDRESS
		export ROS_HOSTNAME=$MAINIPADDRESS
		export ROS_MASTER_URI="http://$MAINIPADDRESS:11311"
                ARGUMENTS=""
                if [[ "$3" =~ "record" ]]; then
                    ARGUMENTS="$ARGUMENTS record:=true "
                fi
                if [[ "$3" =~ "videohd" ]]; then
                    ARGUMENTS="$ARGUMENTS videohd:=true "
                fi
                if [[ "$3" =~ "drive" ]]; then
                    if [[ "$LAUNCHCARINSANE" == "1" ]]; then
                        ARGUMENTS="$ARGUMENTS mode_override:=2 "
                    else
			echo
                        echo "Autonomous mode is disabled in configuration, press enter to proceed without or CRTL+C to exit"
			read
		    fi
                fi
                if [[ "$3" =~ "manual" ]]; then
                    ARGUMENTS="$ARGUMENTS mode_override:=1 "
                fi
		RECORDTIME=$(date +%Y%m%d-%H%M%S)
		mkdir -p ../data/videos/$RECORDTIME
                roslaunch launch/$LAUNCHCAR $ARGUMENTS
		mv ~/.ros/output-cam.avi ../data/videos/$RECORDTIME/cam.avi > /dev/null 2>&1
		mv ~/.ros/output.avi ../data/videos/$RECORDTIME/rviz.avi > /dev/null 2>&1
            ;;
            remote)
                source $PATHROS
                source $PATHSETUP
                MAINIPADDRESS=$(getAddressByInterface $CARINTERFACE)
		export ROS_IP=$MAINIPADDRESS
		export ROS_HOSTNAME=$MAINIPADDRESS
		export ROS_MASTER_URI="http://$MAINIPADDRESS:11311"
                ARGUMENTS="show_rviz:=0"
                if [[ "$3" =~ "record" ]]; then
                    ARGUMENTS="$ARGUMENTS record:=true "
                fi
                if [[ "$3" =~ "videohd" ]]; then
                    ARGUMENTS="$ARGUMENTS videohd:=true "
                fi
                if [[ "$3" =~ "drive" ]]; then
                    if [[ "$LAUNCHCARINSANE" == "1" ]]; then
                        ARGUMENTS="$ARGUMENTS mode_override:=2 "
                    else
			echo
                        echo "Autonomous mode is disabled in configuration, press enter to proceed without or CRTL+C to exit"
			read
		    fi
                fi
                if [[ "$3" =~ "manual" ]]; then
                    ARGUMENTS="$ARGUMENTS mode_override:=1 "
                fi
		RECORDTIME=$(date +%Y%m%d-%H%M%S)
		mkdir -p ../data/videos/$RECORDTIME
                roslaunch launch/$LAUNCHCAR $ARGUMENTS
		mv ~/.ros/output-cam.avi ../data/videos/$RECORDTIME/cam.avi > /dev/null 2>&1
		mv ~/.ros/output.avi ../data/videos/$RECORDTIME/rviz.avi > /dev/null 2>&1
		mv ./output.avi ../data/videos/$RECORDTIME/rviz.avi > /dev/null 2>&1
		mv ./output-cam.avi ../data/videos/$RECORDTIME/cam.avi > /dev/null 2>&1
            ;;

            control)
                source $PATHROS
                source $PATHSETUP
		export ROS_IP="$CARIP"
		export ROS_HOSTNAME="$CARIP"
		export ROS_MASTER_URI="http://$CARIP:11311"
		RECORDTIME=$(date +%Y%m%d-%H%M%S)
		mkdir -p ../data/videos/$RECORDTIME
                rviz -d src/car_control/launch/car.rviz
		mv ~/.ros/output-cam.avi ../data/videos/$RECORDTIME/cam.avi > /dev/null 2>&1
		mv ~/.ros/output.avi ../data/videos/$RECORDTIME/rviz.avi > /dev/null 2>&1
		mv ./output.avi ../data/videos/$RECORDTIME/rviz.avi > /dev/null 2>&1
		mv ./output-cam.avi ../data/videos/$RECORDTIME/cam.avi > /dev/null 2>&1
            ;;
            *)
                toadHelpCar
            ;;
        esac
    ;;
    video)
        # exit when no second parameter is given
        if [[ $# -le 1 ]]; then
            toadHelpVideo
            echo
            exit 1
        fi
        case $2 in
            list)
                toadVideosList
            ;;
            cam)
                # exit when no third parameter is given
                if [[ $# -le 2 ]]; then
                    toadHelpVideo
                    echo
                    exit 1
                fi
                toadVideosCam $3
            ;;
            rviz)
                # exit when no third parameter is given
                if [[ $# -le 2 ]]; then
                    toadHelpVideo
                    echo
                    exit 1
                fi
                toadVideosRviz $3
            ;;
            stitch)
                # exit when no third parameter is given
                if [[ $# -le 2 ]]; then
                    toadHelpVideo
                    echo
                    exit 1
                fi
                toadVideosStitch $3
            ;;
            move)
                toadVideosMove
            ;;
            *)
                toadHelpVideo
            ;;
        esac
    ;;
    telemetry)
        # exit when no second parameter is given
        if [[ $# -le 1 ]]; then
            toadHelpTelemetry
            echo
            exit 1
        fi
        case $2 in
            list)
                # exit when not configured
                if [[ $REPORTCONFIGURED != "1" ]]; then
                    echo "Report not configured, exiting"
                    echo
                    exit 1
                fi
                toadTelemetryList
            ;;
            move)
                toadTelemetryMove
            ;;
            report)
                # exit when not configured
                if [[ $REPORTCONFIGURED != "1" ]]; then
                    echo "Report not configured, exiting"
                    echo
                    exit 1
                fi
                toadTelemetryReport
            ;;
            *)
                toadHelpTelemetry
            ;;
        esac
    ;;
    *)
        toadHelpMain
        echo
        exit 1
    ;;
esac

echo
exit 0
