all:
include ../Settings.mk
include ../Rules.mk

INCLUDES = -I ../
APPS = $(patsubst %.o,%,$(OBJ_FILES))
LIBS = ../Fusion/Fusion.a ../icm20948driver/icm20948driver.a
$(info $(APPS))

icmfusion: icmfusion.o
	$(CXX) $(OBJ_FILES) $(LIBS) -o icmfusion

all: $(APPS)

