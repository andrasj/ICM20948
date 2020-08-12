all:
include ../Settings.mk
include ../Rules.mk

INCLUDES = -I ../
APPS = $(patsubst %.o,%,$(OBJ_FILES))
LIBS = ../Fusion/Fusion.a ../icm20948driver/icm20948driver.a
$(info $(APPS))

icmfusion: icmfusion.o
	$(CXX) icmfusion.o $(LIBS) -o icmfusion

icmcal: icmcal.o
	$(CXX) icmcal.o $(LIBS) -o icmcal

all: $(APPS)

