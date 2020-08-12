TARGET := $(notdir $(PWD)).a

all: $(TARGET)

include ../Settings.mk
include ../Rules.mk

all: $(TARGET)

$(TARGET) : $(OBJ_FILES)
	ar rcs $(TARGET) $(OBJ_FILES)
