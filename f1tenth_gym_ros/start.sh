#!/bin/bash
rocker --device=/dev/input/event14 --nvidia --x11 --volume .:/sim_ws/src/f1tenth_gym_ros -- f1tenth_gym_ros 
