#!/bin/bash

#g++ RaspiCamTest.cpp -o RaspiCamTest -I/home/pi/raspicam_cv -L/home/pi/raspicam_cv -lraspicamcv -L/home/pi/userland/build/lib -lmmal_core -lmmal -lmmal_util -lvcos -lbcm_host `pkg-config --cflags --libs opencv`

#g++ threshold_hsv.cpp -o threshold_hsv -I/home/pi/raspicam_cv -L/home/pi/raspicam_cv -lraspicamcv -L/home/pi/userland/build/lib -lmmal_core -lmmal -lmmal_util -lvcos -lbcm_host `pkg-config --cflags --libs opencv`

#g++ threshold_rgb.cpp -o threshold_rgb -I/home/pi/raspicam_cv -L/home/pi/raspicam_cv -lraspicamcv -L/home/pi/userland/build/lib -lmmal_core -lmmal -lmmal_util -lvcos -lbcm_host `pkg-config --cflags --libs opencv`

#g++ bad_effects.cpp -o bad_effects -I/home/pi/raspicam_cv -L/home/pi/raspicam_cv -lraspicamcv -L/home/pi/userland/build/lib -lmmal_core -lmmal -lmmal_util -lvcos -lbcm_host `pkg-config --cflags --libs opencv`

#g++ quick_threshold.cpp -o quick_threshold -I/home/pi/raspicam_cv -L/home/pi/raspicam_cv -lraspicamcv -L/home/pi/userland/build/lib -lmmal_core -lmmal -lmmal_util -lvcos -lbcm_host `pkg-config --cflags --libs opencv`

#g++ detect_red_object_cross.cpp -o detect_red_object_cross -I/home/pi/raspicam_cv -L/home/pi/raspicam_cv -lraspicamcv -L/home/pi/userland/build/lib -lmmal_core -lmmal -lmmal_util -lvcos -lbcm_host `pkg-config --cflags --libs opencv`

g++ RaspiCamTest2.cpp -o RaspiCamTest2 -I/home/pi/lib/robidouille/raspicam_cv -L/home/pi/lib/robidouille/raspicam_cv -lraspicamcv -L/home/pi/lib/userland/build/lib -lmmal_core -lmmal -lmmal_util -lvcos -lbcm_host `pkg-config --cflags --libs opencv`
