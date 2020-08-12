DEPS = $(wildcard *.h)
CC_SRC_FILES := $(wildcard *.c)
CC_OBJ_FILES := $(patsubst %.c,%.o,$(CC_SRC_FILES))
CXX_SRC_FILES := $(wildcard *.cpp)
CXX_OBJ_FILES := $(patsubst %.cpp,%.o,$(CXX_SRC_FILES))
OBJ_FILES = $(CC_OBJ_FILES) $(CXX_OBJ_FILES)

$(OBJ_FILES) : $(DEPS)

all:

clean:
	rm $(TARGET) $(OBJ_FILES)

cleanall:
	rm *.a *.o

