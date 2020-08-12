TARGET := $(notdir $(PWD)).a

all: $(TARGET)

include ../Settings.mk
include ../Rules.mk

$(TARGET) : $(OBJ_FILES) $(CXX_SRC_FILES)
	ar rcs $(TARGET) $(OBJ_FILES)
