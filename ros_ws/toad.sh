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
                if [[ "$3" != "cron" ]]; then
                    toadConfirmationRequest "This will replace your ssh keys"
                fi
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
                if [[ "$3" != "cron" ]]; then
                    toadConfirmationRequest "This will delete and rebuild"
                fi
                echo "Starting rebuild, please wait ..."
                echo
                OLDBRANCH=$(getActiveBranch)
                git checkout $BRANCHBUILD
                echo
                rm -fr ./build
                source $PATHROS
                source $PATHSETUP
                catkin_make
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
                echo
                rm -fr ./build
                source $PATHROS
                source $PATHSETUP
                catkin_make
                echo
                git checkout $OLDBRANCH
            ;;
            run)
                toadConfirmationRequest "Please ensure a running X Server"
                export DISPLAY=$(getActiveDisplay)
                NOGUI=""
                if [[ "$3" == "nogui" ]]; then
                    NOGUI="gui:=false"
                fi
                source $PATHROS
                source $PATHSETUP
                roslaunch launch/gazebo.launch $NOGUI
            ;;
            *)
                toadHelpSystem
            ;;
        esac
    ;;
    car)
        # exit when no second parameter is given
        if [[ $# -le 1 ]]; then
            toadHelpSystem
            echo
            exit 1
        fi
        case $2 in
            sshkeys)
                if [[ "$3" != "cron" ]]; then
                    toadConfirmationRequest "This will replace your ssh keys"
                fi
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
                if [[ "$3" != "cron" ]]; then
                    toadConfirmationRequest "This will delete and rebuild"
                fi
                echo "Starting rebuild, please wait ..."
                echo
                OLDBRANCH=$(getActiveBranch)
                git checkout $BRANCHCAR
                git reset --hard
                echo
                rm -fr ./build
                source $PATHROS
                source $PATHSETUP
                catkin_make
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
                #git reset --hard
                echo
                rm -fr ./build
                source $PATHROS
                source $PATHSETUP
                catkin_make
                echo
                git checkout $OLDBRANCH
            ;;
            run)
                source $PATHROS
                source $PATHSETUP
                roslaunch launch/car_wallfollowing.launch
            ;;
            *)
                toadHelpCar
            ;;
        esac
    ;;
    init)
        echo "To be implemented"
    ;;
    *)
        toadHelp
        echo
        exit 1
    ;;
esac

echo
exit 0
