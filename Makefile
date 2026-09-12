FUNC := g++
copt := -c 
OBJ_DIR := ./bin/
NATIVE ?= 0
OPENMP ?= 1
EXACT_TRIG ?= 0
OPT ?= 3
ARCH_FLAGS :=
THREAD_FLAGS :=
QUALITY_FLAGS :=
ifeq ($(NATIVE),1)
ARCH_FLAGS += -march=native
endif
ifeq ($(OPENMP),1)
THREAD_FLAGS += -fopenmp
endif
ifeq ($(EXACT_TRIG),1)
QUALITY_FLAGS += -DRAY_EXACT_TRIG
endif
FLAGS := -O$(OPT) -flto -DNDEBUG $(ARCH_FLAGS) $(THREAD_FLAGS) $(QUALITY_FLAGS) -lm -g -Werror

CPP_FILES := $(wildcard src/*.cpp)
OBJ_FILES := $(addprefix $(OBJ_DIR),$(notdir $(CPP_FILES:.cpp=.obj)))

TEXTURE_CPP_FILES := $(wildcard src/Textures/*.cpp)
TEXTURE_OBJ_FILES := $(addprefix $(OBJ_DIR)Textures/,$(notdir $(TEXTURE_CPP_FILES:.cpp=.obj)))

all:
	cd ./src && $(MAKE)
	$(FUNC) ./main.cpp -o ./main.exe ./src/*.obj ./src/Textures/*.obj $(FLAGS)

clean:
	cd ./src && $(MAKE) clean
	rm -f ./*.exe
	rm -f ./*.obj
