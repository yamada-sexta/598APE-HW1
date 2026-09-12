FUNC := g++
.DEFAULT_GOAL := all
copt := -c
output :=-o 
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
FLAGS := -O$(OPT) -flto -DNDEBUG -MMD -MP $(ARCH_FLAGS) $(THREAD_FLAGS) $(QUALITY_FLAGS) -g -Werror
OBJ_DIR :=./

CPP_FILES := $(wildcard *.cpp)
OBJ_FILES := $(addprefix $(OBJ_DIR),$(notdir $(CPP_FILES:.cpp=.obj)))
DEP_FILES := $(OBJ_FILES:.obj=.d)

-include $(DEP_FILES)

all: $(OBJ_FILES)

$(OBJ_DIR)texture.obj: texture.cpp texture.h
	$(FUNC) $(output)$(OBJ_DIR)texture.obj $(copt) texture.cpp $(FLAGS)

$(OBJ_DIR)mandeltexture.obj: mandeltexture.cpp mandeltexture.h $(OBJ_DIR)texture.obj
	$(FUNC) $(output)$(OBJ_DIR)mandeltexture.obj $(copt) mandeltexture.cpp $(FLAGS)

$(OBJ_DIR)fractalnoise.obj: fractalnoise.cpp fractalnoise.h $(OBJ_DIR)texture.obj
	$(FUNC) $(output)$(OBJ_DIR)fractalnoise.obj $(copt) fractalnoise.cpp $(FLAGS)

$(OBJ_DIR)imagetexture.obj: imagetexture.cpp imagetexture.h $(OBJ_DIR)texture.obj
	$(FUNC) $(output)$(OBJ_DIR)imagetexture.obj $(copt) imagetexture.cpp $(FLAGS)

$(OBJ_DIR)colortexture.obj: colortexture.cpp colortexture.h $(OBJ_DIR)texture.obj
	$(FUNC) $(output)$(OBJ_DIR)colortexture.obj $(copt) colortexture.cpp $(FLAGS)

$(OBJ_DIR)functiontexture.obj: functiontexture.cpp functiontexture.h $(OBJ_DIR)texture.obj
	$(FUNC) $(output)$(OBJ_DIR)functiontexture.obj $(copt) functiontexture.cpp $(FLAGS)

clean:
	rm -f $(OBJ_DIR)*.obj $(OBJ_DIR)*.d
