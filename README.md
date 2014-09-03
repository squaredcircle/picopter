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

    sudo apt-get git install core


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

dwm (dynamic window manager) is a simple tiling desktop environment.  It runs an X window server, so images can be displayed through OpenCV.  Installation of dwm is not essential, but is more conventient than using LXDE.

    sudo apt-get install xterm dwm

See http://dwm.suckless.org for more information.


---------------------------------------------
3.  Install boost and ncurses
---------------------------------------------

boost is a library of tools that extends the c++ standard libraries.  In this project, we use it  largely for multithreading.

nCurses is a library used for printing text to the screen in a fasionable way.


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
5.  Install wiringPi
---------------------------------------------

wiringPi is the library used to interact with the pi's gpio pins.

    cd ~
    git clone git://git.drogon.net/wiringPi
    cd wiringPi
    ./build

See https://projects.drogon.net/raspberry-pi/wiringpi/ for more information.


---------------------------------------------
6.  Install ServoBlaster
---------------------------------------------

ServoBlaster is the program used to output pwm on some of the pi's gpio pins.  These are used as inputs to the flight board.

    cd ~
    git clone git://github.com/richardghirst/PiBits.git
    cd PiBits/ServoBlaster/user
    make servod

See https://github.com/richardghirst/PiBits/tree/master/ServoBlaster for more information.


---------------------------------------------
7.  Install OpenCV
---------------------------------------------

OpenCV is the library used for all image processing.

    sudo apt-get install libopencv-dev

See http://docs.opencv.org/trunk/modules/core/doc/intro.html for more information.


---------------------------------------------
8.  Install userland
---------------------------------------------

userland is the collection of development files used by raspivid_cv to interface with the GPU and thus the camera board.

    cd ~
    git clone git://github.com/raspberrypi/userland.git
    cd userland
    ./buildme

See https://github.com/raspberrypi/userland for more information.


---------------------------------------------
9.  Install raspivid_cv
---------------------------------------------

raspivid_cv is a library which allows us to use OpenCV on the pi's camera.

    cd ~
    git clone git://github.com/robidouille/robidouille.git
    cd robidouille/raspicam_cv
    mkdir objs

One line needs to be changed in the make file for this to work:

    nano Makefile

On line 6, remove "/git/raspberrypi".  ctrl-x to exit nano (y, return).

Then make raspicam_cv;

    make


See https://github.com/robidouille/robidouille/tree/master/raspicam_cv for more information.


---------------------------------------------
10.  Install picopter
---------------------------------------------
Yo! Champ in the making.  Almost there.  There's just a few more things to do before you're very own picopter is out there, stalking unsuspecting red bucket lids.

    cd ~
    git clone git://github.com/crazyoldmans/picopter.git


*10.1   Building Xsens drivers*

The Xsens is the IMU (incl. compass) we use on the copter.

    cd ~/picopter/Xsens
    make


*10.2   Building base systems*

This set of classes forms the low-level system and wraps many of the previously introduced programs.  The low-level systems are multi-threaded, so they gather data in the background, while a higher level control system runs in the foreground.  Test programs (examples) are also included in the bin folder.

    cd ~/picopter/picopter-base
    mkdir obj bin logs
    make


*10.3   Building controller applications*

This set of programs controls the picopter as it sets to follow waypoints and track red objects.

    cd ~/picopter/picopter-objects
    mkdir obj bin logs photos
    make
    

---------------------------------------------
11.  Have fun!
---------------------------------------------

Yeah, that one's a joke.


