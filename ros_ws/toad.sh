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
                    toadConfirmationRequest "WARNING: This will gain access to this system to all group members - dont try this at home"
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
                if [[ $SLACK -ge 1 ]] && [[ $SLACKSYSTEMSSH -ge 1 ]]; then
                    echo
                    sendSlackMessage ssh
                fi
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
                if [[ $SLACK -ge 1 ]] && [[ $SLACKSYSTEMREBUILD -ge 1 ]]; then
                    echo
                    sendSlackMessage rebuild
                fi
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
                catkin_make
                echo
                git checkout $OLDBRANCH
                if [[ $SLACK -ge 1 ]] && [[ $SLACKSYSTEMRESETBUILD -ge 1 ]]; then
                    echo
                    sendSlackMessage resetbuild
                fi
            ;;
            run)
                toadConfirmationRequest "Please ensure a running and logged in X Session with GDM3"
                if [[ $(ps aux | grep gdm | grep session | grep -v grep | grep $(whoami) | wc -l) -le 0 ]]; then
                    echo
                    echo "No running gdm-x-session process found, exiting"
                    echo
                    exit 1
                fi
                export DISPLAY=$(getActiveDisplay)
                ARGUMENTS=""
                if [[ "$3" =~ "nogui" ]]; then
                    ARGUMENTS="gui:=false "
                fi
                if [[ "$3" =~ "fast" ]]; then
                    ARGUMENTS="$ARGUMENTS fast:=true "
                fi
                if [[ "$3" =~ "drive" ]]; then
                    ARGUMENTS="$ARGUMENTS mode_override:=2 "
                fi
                if [[ "$3" =~ "manual" ]]; then
                    ARGUMENTS="$ARGUMENTS mode_override:=1 "
                fi
                if [[ "$3" =~ "obstacle" ]]; then
                    ARGUMENTS="$ARGUMENTS world:=$LAUNCHTRACKOBSTACLE "
                fi
                source $PATHROS
                source $PATHSETUP
                roslaunch launch/$LAUNCHBUILD use_gpu:=$USEGPU $ARGUMENTS
                if [[ $SLACK -ge 1 ]] && [[ $SLACKSYSTEMRUN -ge 1 ]]; then
                    echo
                    sendSlackMessage custom "Starting simulation with following arguments: $ARGUMENTS"
                fi
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
                if [[ "$3" != "cron" ]]; then
                    toadConfirmationRequest "This will replace your ssh keys"
                    toadConfirmationRequest "WARNING: This will gain access to this system to all group members - dont try this at home"
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
                if [[ $SLACK -ge 1 ]] && [[ $SLACKCARSSH -ge 1 ]]; then
                    echo
                    sendSlackMessage ssh
                fi
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
                if [[ $SLACK -ge 1 ]] && [[ $SLACKCARREBUILD -ge 1 ]]; then
                    echo
                    sendSlackMessage rebuild
                fi
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
                catkin_make
                echo
                git checkout $OLDBRANCH
                if [[ $SLACK -ge 1 ]] && [[ $SLACKCARRESETBUILD -ge 1 ]]; then
                    echo
                    sendSlackMessage resetbuild
                fi
            ;;
            run)
                source $PATHROS
                source $PATHSETUP
                MAINIPADDRESS=$(getAddressByInterface $CARINTERFACE)
		export ROS_IP=$MAINIPADDRESS
		export ROS_HOSTNAME=$MAINIPADDRESS
		export ROS_MASTER_URI="http://$MAINIPADDRESS:11311"
                roslaunch launch/$LAUNCHCAR
                if [[ $SLACK -ge 1 ]] && [[ $SLACKCARRUN -ge 1 ]]; then
                    echo
                    sendSlackMessage custom "Watch you feet, I'm on the road"
                fi
            ;;
            remote)
                source $PATHROS
                source $PATHSETUP
                MAINIPADDRESS=$(getAddressByInterface $CARINTERFACE)
		export ROS_IP=$MAINIPADDRESS
		export ROS_HOSTNAME=$MAINIPADDRESS
		export ROS_MASTER_URI="http://$MAINIPADDRESS:11311"
                roslaunch launch/$LAUNCHCAR show_rviz:=0
                if [[ $SLACK -ge 1 ]] && [[ $SLACKCARRUN -ge 1 ]]; then
                    echo
                    sendSlackMessage custom "Watch you feet, I'm on the road (remote controlled)"
                fi
            ;;
            control)
                source $PATHROS
                source $PATHSETUP
		export ROS_IP="$CARIP"
		export ROS_HOSTNAME="$CARIP"
		export ROS_MASTER_URI="http://$CARIP:11311"
                rviz -d src/car_control/launch/car.rviz
            ;;
            *)
                toadHelpCar
            ;;
        esac
    ;;
    init)
        # exit when no second parameter is given
        if [[ $# -le 1 ]]; then
            toadHelpInit
            echo
            exit 1
        fi
        DISTRIBUTION=$(lsb_release -i | awk {'print $3'})
        VERSION=$(lsb_release -r | awk {'print $2'})
        CODENAME=$(lsb_release -c | awk {'print $2'})
        FORCED="yes"
        if [[ $3 != 'force' ]]; then
            if [[ $DISTRIBUTION != 'Ubuntu' ]]; then
                echo "Your distribution ($DISTRIBUTION) is not supported"
                echo "You may use the force argument, but be warned!"
                echo
                exit 1
            fi
            if [[ $VERSION != '16.04' ]] && [[ $VERSION != '18.04' ]]; then
                echo "Your Ubuntu version ($VERSION) is not supported"
                echo "You may use the force argument, but be warned!"
                echo
                exit 1
            fi
            FORCED="no"
        fi

        case $2 in
            system)
                toadInitParameters
                toadConfirmationRequest "This stuff is hardly untested, please report results or supply patches"
                toadConfirmationRequest "This will install all required system packages"
                if [[ $VERSION == '16.04' ]]; then
                    source $PATHROS
                    RESULT=""
                    while [[ $RESULT != 's' && $RESULT != 'p' ]]; do
                        toadConfirmationEnter "Update packages and upgrade system"
                        read RESULT
                    done
                    if [[ $RESULT == 'p' ]]; then
                        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
                        sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
                        sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
                        wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
                        sudo apt-get update -qq
                        sudo apt-get upgrade -y
                    else
                        echo "Skipping"
                    fi

                    RESULT=""
                    while [[ $RESULT != 's' && $RESULT != 'p' ]]; do
                        toadConfirmationEnter "Now we install OS Packages"
                        read RESULT
                    done
                    if [[ $RESULT == 'p' ]]; then
                        sudo apt-get install -y python-catkin-tools libsdl2-dev ros-kinetic-ackermann-msgs ros-melodic-serial ros-kinetic-desktop-full gazebo7 libgazebo7-dev ros-kinetic-gazebo-ros-control ros-kinetic-joy ros-kinetic-map-server ros-kinetic-move-base mplayer ffmpeg mencoder netcat ros-kinetic-rviz-imu-plugin
                        sudo apt-get install -y libignition-math2-dev
                        sudo apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
                    else
                        echo "Skipping"
                    fi

                    RESULT=""
                    while [[ $RESULT != 's' && $RESULT != 'p' ]]; do
                        toadConfirmationEnter "Now we init ROS"
                        read RESULT
                    done
                    if [[ $RESULT == 'p' ]]; then
                        sudo rosdep init
                    else
                        echo "Skipping"
                    fi

                    RESULT=""
                    while [[ $RESULT != 's' && $RESULT != 'p' ]]; do
                        toadConfirmationEnter "Now we reset pip and install python packages"
                        read RESULT
                    done
                    if [[ $RESULT == 'p' ]]; then
                        source $PATHROS
                        sudo python -m pip uninstall -y pip
                        sudo apt-get install -y python-pip
                        sudo apt-get install -y libsdl2-dev clang-format python-pyqtgraph
                        sudo python2 -m pip install --upgrade pip --force
                        sudo python2 -m pip install --no-cache-dir torch autopep8 cython circle-fit slack-cli
                    else
                        echo "Skipping"
                    fi

                    RESULT=""
                    while [[ $RESULT != 's' && $RESULT != 'p' ]]; do
                        toadConfirmationEnter "Now we install range_libc"
                        read RESULT
                    done
                    if [[ $RESULT == 'p' ]]; then
                        cd ../.. && git clone http://github.com/kctess5/range_libc
                        cd ../../range_libc/pywrapper && ./compile.sh
                    else
                        echo "Skipping"
                    fi
                else
                    source $PATHROS
                    RESULT=""
                    while [[ $RESULT != 's' && $RESULT != 'p' ]]; do
                        toadConfirmationEnter "Update packages and upgrade system"
                        read RESULT
                    done
                    if [[ $RESULT == 'p' ]]; then
                        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
                        sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
                        sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
                        wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
                        sudo apt-get update -qq
                        sudo apt-get upgrade -y
                    fi
                    RESULT=""
                    while [[ $RESULT != 's' && $RESULT != 'p' ]]; do
                        toadConfirmationEnter "Now we reset pip and install python packages"
                        read RESULT
                    done
                    if [[ $RESULT == 'p' ]]; then
                        sudo apt-get install -y python-catkin-tools libsdl2-dev ros-melodic-ackermann-msgs ros-melodic-serial ros-melodic-desktop-full gazebo9 libgazebo9-dev ros-melodic-gazebo-ros-control mplayer ffmpeg mencoder netcat ros-melodic-rviz-imu-plugin
                        sudo apt-get install -y libignition-math2-dev
                        sudo apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
                    else
                        echo "Skipping"
                    fi

                    RESULT=""
                    while [[ $RESULT != 's' && $RESULT != 'p' ]]; do
                        toadConfirmationEnter "Now we init ROS"
                        read RESULT
                    done
                    if [[ $RESULT == 'p' ]]; then
                        sudo rosdep init
                    else
                        echo "Skipping"
                    fi

                    RESULT=""
                    while [[ $RESULT != 's' && $RESULT != 'p' ]]; do
                        toadConfirmationEnter "Now we reset pip and install python packages"
                        read RESULT
                    done
                    if [[ $RESULT == 'p' ]]; then
                        source $PATHROS
                        sudo python -m pip uninstall -y pip
                        sudo apt-get install -y python-pip
                        sudo apt-get install -y libsdl2-dev clang-format python-pyqtgraph
                        sudo python2 -m pip install --upgrade pip --force
                        sudo python2 -m pip install --no-cache-dir torch autopep8 cython circle-fit vpython slack-cli
                    else
                        echo "Skipping"
                    fi

                    RESULT=""
                    while [[ $RESULT != 's' && $RESULT != 'p' ]]; do
                        toadConfirmationEnter "Now we install range_libc"
                        read RESULT
                    done
                    if [[ $RESULT == 'p' ]]; then
                        cd ../.. && git clone http://github.com/kctess5/range_libc
                        cd ../../range_libc/pywrapper && ./compile.sh
                    else
                        echo "Skipping"
                    fi
                fi
            ;;
            ros)
                toadInitParameters
                toadConfirmationRequest "This stuff is hardly untested, please report results or supply patches"
                toadConfirmationRequest "This will install all required ros packages"
                if [[ $VERSION == '16.04' ]]; then
                    source $PATHROS
                    cd $WORKDIR/src/external_packages/ && git clone https://github.com/KristofRobot/razor_imu_9dof.git
                    cd $WORKDIR/.. && git submodule init
                    cd $WORKDIR/.. && git submodule update --recursive
                    cd $WORKDIR/.. && rosdep update
                    cd $WORKDIR && rosdep install -y --from-paths ./src --ignore-src --rosdistro kinetic 
                    catkin_make
                else
                    source $PATHROS
                    cd $WORKDIR/src/external_packages/ && git clone https://github.com/KristofRobot/razor_imu_9dof.git
                    cd $WORKDIR/.. && git submodule init
                    cd $WORKDIR/.. && git submodule update --recursive
                    cd $WORKDIR/.. && rosdep update
                    cd $WORKDIR && rosdep install -y --from-paths ./src --ignore-src --rosdistro melodic 
                    catkin_make
                fi
            ;;
            camstream)
                toadInitParameters
                toadConfirmationRequest "This stuff is hardly untested, please report results or supply patches"
                toadConfirmationRequest "This will install all required packages for camera streaming server and client"
                if [[ $VERSION == '16.04' ]]; then
                    apt-get install mplayer mencoder ffmpeg netcat
                else
                    apt-get install mplayer mencoder ffmpeg netcat
                fi
            ;;
            ide)
                toadInitParameters
                toadConfirmationRequest "This will install all required ide packages"
                if [[ $VERSION == '16.04' ]]; then
                    RESULT=""
                    while [[ $RESULT != 's' && $RESULT != 'p' ]]; do
                        toadConfirmationEnter "This will install Visual Studio Code"
                        read RESULT
                    done
                    if [[ $RESULT == 'p' ]]; then
                        curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
                        sudo install -o root -g root -m 644 packages.microsoft.gpg /usr/share/keyrings/
                        sudo sh -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'
                        sudo apt-get install apt-transport-https
                        sudo apt-get update
                        sudo apt-get install code
                        rm packages.microsoft.gpg
                        echo
                        echo
                        toadConfirmationRequest "To install plugin, follow instructions from" "https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros"
                    else
                        echo "Skipping"
                    fi
                    RESULT=""
                    while [[ $RESULT != 's' && $RESULT != 'p' ]]; do
                        toadConfirmationEnter "This will install Netbeans"
                        read RESULT
                    done
                    if [[ $RESULT == 'p' ]]; then
                        curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
                        sudo install -o root -g root -m 644 packages.microsoft.gpg /usr/share/keyrings/
                        sudo sh -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'
                        sudo apt-get install apt-transport-https
                        sudo apt-get update
                        sudo apt-get install code
                        echo
                        echo
                        toadConfirmationRequest "To install plugin, follow instructions from" "http://plugins.netbeans.org/plugin/60486/netbeans-ros-pack"
                    else
                        echo "Skipping"
                    fi
                else
                    RESULT=""
                    while [[ $RESULT != 's' && $RESULT != 'p' ]]; do
                        toadConfirmationEnter "This will install Visual Studio Code"
                        read RESULT
                    done
                    if [[ $RESULT == 'p' ]]; then
                        curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
                        sudo install -o root -g root -m 644 packages.microsoft.gpg /usr/share/keyrings/
                        sudo sh -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'
                        sudo apt-get install apt-transport-https
                        sudo apt-get update
                        sudo apt-get install code
                        rm packages.microsoft.gpg
                        echo
                        echo
                        toadConfirmationRequest "To install plugin, follow instructions from" "https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros"
                    else
                        echo "Skipping"
                    fi
                    RESULT=""
                    while [[ $RESULT != 's' && $RESULT != 'p' ]]; do
                        toadConfirmationEnter "This will install Netbeans"
                        read RESULT
                    done
                    if [[ $RESULT == 'p' ]]; then
                        sudo apt-get install netbeans
                        echo
                        echo
                        toadConfirmationRequest "To install plugin, follow instructions from" "http://plugins.netbeans.org/plugin/60486/netbeans-ros-pack"
                    else
                        echo "Skipping"
                    fi
                fi
            ;;
            env)
                toadInitParameters
                toadConfirmationRequest "This will update your environment settings"
                if [[ $VERSION == '16.04' ]]; then
                    source $PATHROS
                    source $PATHSETUP
                    MAINIPADDRESS=$(getAddressByInterface $CARINTERFACE)
                    export ROS_IP=$MAINIPADDRESS
                    export ROS_HOSTNAME=$MAINIPADDRESS
                    export ROS_MASTER_URI="http://$MAINIPADDRESS:11311"
                else
                    source $PATHROS
                    source $PATHSETUP
                    MAINIPADDRESS=$(getAddressByInterface $CARINTERFACE)
                    export ROS_IP=$MAINIPADDRESS
                    export ROS_HOSTNAME=$MAINIPADDRESS
                    export ROS_MASTER_URI="http://$MAINIPADDRESS:11311"
                fi
                echo "Done:"
                echo
                env | grep ROS
            ;;
            *)
                toadHelpInit
            ;;
        esac
    ;;
    slack)
        # exit when no second parameter is given
        if [[ $# -le 1 ]]; then
            toadHelpSlack
            echo
            exit 1
        fi
        case $2 in
            status)
                sendSlackMessage status
            ;;
            custom)
                sendSlackMessage custom "$3"
            ;;
            rebuild)
                sendSlackMessage rebuild
            ;;
            resetbuild)
                sendSlackMessage resetbuild
            ;;
            ssh)
                sendSlackMessage ssh
            ;;
            *)
                toadHelpSlack
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
