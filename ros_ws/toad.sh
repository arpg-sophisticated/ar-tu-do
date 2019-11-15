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
                roslaunch launch/$LAUNCHBUILD $NOGUI
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
                roslaunch launch/$LAUNCHCAR
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
                toadConfirmationRequest "This will install all required system packages"
                if [[ $VERSION == '16.04' ]]; then
                    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
                    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
                    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
                    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
                    sudo apt-get update -qq
                    sudo apt-get upgrade -y
                    sudo apt-get install -y python-catkin-tools libsdl2-dev ros-kinetic-ackermann-msgs ros-melodic-serial ros-kinetic-desktop-full gazebo7 libgazebo7-dev ros-kinetic-gazebo-ros-control ros-kinetic-joy ros-kinetic-map-server ros-kinetic-move-base
                    sudo apt-get install -y libignition-math2-dev
                    sudo rosdep init
                    source $PATHROS
                    sudo apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
                    sudo python -m pip uninstall -y pip
                    sudo apt-get install -y python-pip
                    sudo apt-get install -y libsdl2-dev clang-format python-pyqtgraph
                    sudo python2 -m pip install --upgrade pip --force
                    sudo python2 -m pip install --no-cache-dir torch autopep8 cython circle-fit
                    cd ../.. && git clone http://github.com/kctess5/range_libc
                    cd ../range_libc/pywrapper && ./compile.sh
                fi
                if [[ $VERSION == '18.04' ]]; then
                    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
                    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
                    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
                    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
                    sudo apt-get update -qq
                    sudo apt-get upgrade -y
                    sudo apt-get install -y python-catkin-tools libsdl2-dev ros-melodic-ackermann-msgs ros-melodic-serial ros-melodic-desktop-full gazebo9 libgazebo9-dev ros-melodic-gazebo-ros-control
                    sudo apt-get install -y libignition-math2-dev
                    sudo rosdep init
                    source $PATHROS
                    sudo apt-get install -y python-visual python-rosinstall python-rosinstall-generator python-wstool build-essential
                    sudo python -m pip uninstall -y pip
                    sudo apt-get install -y python-pip
                    sudo apt-get install -y libsdl2-dev clang-format python-pyqtgraph
                    sudo python2 -m pip install --upgrade pip --force
                    sudo python2 -m pip install --no-cache-dir torch autopep8 cython circle-fit
                    cd ../.. && git clone http://github.com/kctess5/range_libc
                    cd ../range_libc/pywrapper && ./compile.sh
                fi
            ;;
            ros)
                toadInitParameters
                toadConfirmationRequest "This will install all required ros packages"
                if [[ $VERSION == '16.04' ]]; then
                    cd .. && git submodule init
                    cd .. && git submodule update --recursive
                    cd .. && rosdep update
                    rosdep install -y --from-paths src --ignore-src --rosdistro kinetic 
                    catkin_make
                fi
                if [[ $VERSION == '18.04' ]]; then
                    cd .. && git submodule init
                    cd .. && git submodule update --recursive
                    cd .. && rosdep update
                    rosdep install -y --from-paths src --ignore-src --rosdistro kinetic 
                    catkin_make
                fi
            ;;
            *)
                toadHelpInit
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
