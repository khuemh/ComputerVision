#!/bin/bash
#mkdir release
g++ $1 -o runfile `pkg-config --cflags --libs opencv`
