# Backing up the CAR

To create an image of the car, please install the L4T Driver Package first:
* [[https://developer.nvidia.com/embedded/linux-tegra]]
* [[https://elinux.org/Jetson/TX2_Cloning]]

## Create an image
Change into the directory containing the L4T installation package on the host PC. The command below will save the TX2's eMMC image to the specified file on the host.

    sudo ./flash.sh -r -k APP -G <DESTINATIONFILENAME> jetson-tx2 mmcblk0p1

## Restoring an image
Change into the directory containing the L4T installation package on the host PC.

    sudo cp <BACKUPFILENAME> bootloader/system.img
    sudo ./flash.sh -r -k APP jetson-tx2 mmcblk0p1
