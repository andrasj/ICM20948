all:
include ../Settings.mk
include ../Rules.mk

INCLUDES = -I ../
APPS = $(patsubst %.o,%,$(OBJ_FILES))
LIBS = ../Fusion/Fusion.a ../icm20948driver/icm20948driver.a
#$(info $(APPS))

icmfusion: icmfusion.cpp $(wildcard ../icm20948driver/*.cpp) $(wildcard ../Fusion/*.c) $(wildcard ../icm20948driver/*.h) $(wildcard ../Fusion/*.h)
	$(CC) $(wildcard ../Fusion/*.c) -c -I ../
	ar rcs Fusion.a $(notdir $(patsubst %.c,%.o,$(wildcard ../Fusion/*.c)))
	$(CXX) icmfusion.cpp $(wildcard ../icm20948driver/*.cpp) Fusion.a -o icmfusion -I ../

icmcal: icmcal.o $(LIBS)
	$(CXX) icmcal.o $(LIBS) -o icmcal

icmtest: icmtest.cpp $(wildcard ../icm20948driver/*.cpp) $(wildcard ../icm20948driver/*.h)
	$(CXX) icmtest.cpp $(wildcard ../icm20948driver/*.cpp) -o icmtest -I ../

all: $(APPS)

clean:
	rm $(APPS) *.a *.o

