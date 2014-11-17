==========================
How to train your picopter
==========================

*Setting up a raspberry-pi powered hexacopter from a clean flash of raspbian*


---------------------------------------------
1.  Update repositories and upgrade system
---------------------------------------------

    sudo apt-get update
    sudo apt-get upgrade
    

Also, if git isn't installed, get git.

    sudo apt-get install git-core


---------------------------------------------
2.  Enable support for camera board
---------------------------------------------

You will need to configure support for the raspberry pi camera.  Run the following command:

    sudo raspi-config

Select "4 Enable Camera" and follow prompts.

Now wouldn't be a bad time to also select the time zone and keyboard from the "3 Internationalisaton Options" menu.


---------------------------------------------
3.  Install dwm
---------------------------------------------

dwm (dynamic window manager) is a simple tiling desktop environment.  It runs an X window server, so images can be displayed through OpenCV.  Installation of dwm is not essential, but is more convenient than using LXDE.

    sudo apt-get install xterm dwm

See http://dwm.suckless.org for more information.


---------------------------------------------
3.  Install boost and ncurses
---------------------------------------------

boost is a library of tools that extends the c++ standard libraries.  In this project, we use it  largely for multithreading.

nCurses is a library used for printing text to the screen in a fashionable way.


    sudo apt-get install libboost-all-dev
    sudo apt-get install libncurses5-dev

See http://www.boost.org for more information about the boost libraries and https://www.gnu.org/software/ncurses/ or http://tldp.org/HOWTO/NCURSES-Programming-HOWTO/ for more information on ncurses.


---------------------------------------------
4.  Install cmake
---------------------------------------------

Cmake is used to build the raspicam userland tools later on.

    sudo apt-get install cmake

See http://www.cmake.org for more information.


---------------------------------------------
5.  Make libraries directory
---------------------------------------------

We'll put all of the hexacopter relevant libraries and drivers here.

    mkdir ~/lib


---------------------------------------------
6.  Install wiringPi
---------------------------------------------

wiringPi is the library used to interact with the pi's gpio pins.

    cd ~/lib
    git clone git://git.drogon.net/wiringPi
    cd wiringPi
    ./build

See https://projects.drogon.net/raspberry-pi/wiringpi/ for more information.


---------------------------------------------
7.  Install ServoBlaster
---------------------------------------------

ServoBlaster is the program used to output pwm on some of the pi's gpio pins.  These are used as inputs to the flight board.

    cd ~/lib
    git clone git://github.com/richardghirst/PiBits.git
    cd PiBits/ServoBlaster/user
    make servod

See https://github.com/richardghirst/PiBits/tree/master/ServoBlaster for more information.


---------------------------------------------
8.  Install OpenCV
---------------------------------------------

OpenCV is the library used for all image processing.

    sudo apt-get install libopencv-dev

See http://docs.opencv.org/trunk/modules/core/doc/intro.html for more information.


---------------------------------------------
9.  Install userland
---------------------------------------------

userland is the collection of development files used by raspivid_cv to interface with the GPU and thus the camera board.

    cd ~/lib
    git clone git://github.com/raspberrypi/userland.git
    cd userland
    ./buildme

See https://github.com/raspberrypi/userland for more information.


---------------------------------------------
10.  Install raspivid_cv
---------------------------------------------

raspivid_cv is a library which allows us to use OpenCV on the pi's camera.

    cd ~/lib
    git clone git://github.com/robidouille/robidouille.git
    cd robidouille/raspicam_cv
    mkdir objs

One line needs to be changed in the make file for this to work:

    nano Makefile

On line 6, remove "/git/raspberrypi" and replace it with "/lib".  The line should now read:

    USERLAND_ROOT = $(HOME)/lib/userland
    
Ctrl-x to exit nano (y, return).

Then make raspicam_cv;

    make


See https://github.com/robidouille/robidouille/tree/master/raspicam_cv for more information.


---------------------------------------------
11.  Install Xsens libraries
---------------------------------------------

The hexacopter used an Xsens MTi IMU (accelerometers and magnetometers).  It is used largely as a compass.  I've put the libraries up on my git, since they aren't available anymore (too old).  Alternatively, they should be on the Xsens usb memory stick.  Provided you can find the box.

    cd ~/lib
    git clone git://github.com/TGLogic/Xsens.git
    cd Xsens
    make

The compiler usually complains about some 'cdecl' attribute.  It's fine to ignore it, as long as you don't compile with -Werror.


---------------------------------------------
12.  Install picopter
---------------------------------------------
Yo! Champ in the making.  Almost there.  There's just a few more things to do before your very own picopter is out there, stalking unsuspecting red bucket lids.

    cd ~
    git clone git://github.com/crazyoldmans/picopter.git


*12.1   Build the project*

Nowadays, there's a nice makefile that calls all the other make files.  All you have to do is start it off and make a cuppa tea.

    cd ~/picopter
    make


*12.2   Create logs directory*

Logs will magically appear here.  See logger.h to change the path.

    cd ~
    mkdir logs
    
    
---------------------------------------------
13.  Have fun!
---------------------------------------------

Yeah, that one's a joke.


