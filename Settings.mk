CC = arm-linux-gnueabihf-gcc
CXX = arm-linux-gnueabihf-g++
LINK.o := $(LINK.cc)
CPPFLAGS = --sysroot=/home/andrasj/projects/reach/base/ -pipe -O2 -Wall -W $(INCLUDES)
LDFLAGS = ../icm20948driver/icm20948driver.a ../Fusion/Fusion.a
