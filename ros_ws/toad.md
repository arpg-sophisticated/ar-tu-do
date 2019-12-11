# About toad.sh

This CLI tool contains the most important commands for userfull tasks. It must be called from inside its directory due relative paths.

To use it, copy toad.example to toad.settings and configure that file as needed. Then run

    toad.sh

to get some help about the usage.

## Prequisites

For sure, you need a properly configured system with Ubuntu 16.04 or 18.04 running. Also you need 3D support enabled.

**Important: To run the simulation via ssh on the remote system's X session, you'll need gdm3 and a gnome session running (and therefore a, no, ONE gdm-x-session process running), so that the system can guess the DISPLAY to use correctly. Sorry folks, Ubunutu/Wayland/??? massacre.**

**It could work with other display managers if you issue 'export DISPLAY=:0' prior starting the simulation.

This was tested with Ubuntu 18.04 from local machine and remote system. If the can't recognize the DISPLAY correctly, please send me the output of ```w -sh```, the output of ```env``` and the output of the script.

## toad.sh system

Commands to use on a normal computer or on the build server. Branch is configured in BRANCHBUILD.

### toad.sh system sshkeys [cron]

This updates the local and current user's authorized_keys file with the keys located in the ssh directory of the repository from the configured branch. Please don' use this on your own machine, because all group member will gain access to your system and your own authorized_keys file is lost. You have been warned.

Arguments:
* cron - this removes the confirmation (usefull for cron)

### toad.sh system rebuild [cron]

This removes the build directory and rebuild the whole project from the configured build branch. 

Arguments:
* cron - this removes the confirmation (usefull for cron)

### toad.sh system resetbuild [cron]

This will reset the working dir to the current state of the configured branch and proceed like rebuild explained before. This reset will lead to a loss of all unpushed changes. You habe been warned.

Arguments:
* cron - this removes the confirmation (usefull for cron)

### toad.sh system run [nogui,fast,drive,manual,obstacle]

This will run the gazebo simuation.

Arguments:
* nogui - this disables the local gazebo client (more stable and performant on our systems)
* fast - enforce fast mode
* drive - enforce autonomous driving
* manual -  enforce manual driving
* obstacle -  use track with obstacle

## toad.sh car

Commands to use on the car. Branch is configured in BRANCHCAR.

### toad.sh car sshkeys [cron]

This updates the local and current user's authorized_keys file with the keys located in the ssh directory of the repository from the configured branch. Please don' use this on your own machine, because all group member will gain access to your system and your own authorized_keys file is lost. You have been warned.

Arguments:
* cron - this removes the confirmation (usefull for cron)

### toad.sh car rebuild [cron]

This removes the build directory and rebuild the whole project from the configured build branch. 

Arguments:
* cron - this removes the confirmation (usefull for cron)

### toad.sh car resetbuild [cron]

This will reset the working dir to the current state of the configured branch and proceed like rebuild explained before. This reset will lead to a loss of all unpushed changes. You habe been warned.

Arguments:
* cron - this removes the confirmation (usefull for cron)

### toad.sh car run

This will run the software on the car.

## toad.sh init

**WARNING:** This stuff is hardly untested and may damage your system. Please report results and problems or supply patches to marcel.ebbrecht@googlemail.com.

This could be used to initialize the whole system. Please checkout the repo before. This should only be used on Ubuntu 16.04 or 18.04. It is unusable on the car.

### toad.sh init env

This will initialize the environment based on the current working directory.

### toad.sh init system [force]

This will install everything you need on system side. Please ensure, that you're allowed to sudo.

Arguments:
* force - this will ommit the check of the correct system and force installation

### toad.sh init ros [force]

This will install all needed stuff regarding the software, like:
* Libraries
* Subprojects
* ROS

Please ensure, that you're allowed to sudo.

Arguments:
* force - this will ommit the check of the correct system and force installation in Ubuntu 18.04 mode

### toad.sh init ide

This will install IDEs and provide informations how to get the plugins for ROS
* Visual Studio Code
* Netbeans

## toad.sh slack

This is for sending messages to slack channels configured in settings.

### toad.sh status

Sends a preconfigured status message (look at the code).

### toad.sh custom 'MESSAGE'

Sends a custom message given in parantheses.

Arguments:
* MESSAGE - the message to send. Must be a string in parantheses.
