.DEFAULT_GOAL := all
include mk/cpu-config.mk

CPU_BUILD ?= build/cpu
CPU_OUTPUT ?= main.exe
CPU_SOURCES := $(wildcard src/*.cpp src/Textures/*.cpp)
CPU_OBJECTS := $(patsubst %.cpp,$(CPU_BUILD)/%.o,$(CPU_SOURCES))
CPU_MAIN := $(CPU_BUILD)/main.o
CPU_CONFIG := $(CPU_BUILD)/build-config.txt
CPU_TEST_NAMES := bvh4 screen_bounds texture_tiles acceleration_lifecycle transparent_shadow
CPU_TESTS := $(addprefix $(CPU_BUILD)/tests/,$(CPU_TEST_NAMES))
cpu_quote = '$(subst ','"'"',$(1))'

.PHONY: all cpu-build cpu-objects cpu-textures clean check-cpu FORCE
all cpu-build: $(CPU_OUTPUT)
cpu-objects: $(CPU_OBJECTS)
cpu-textures: $(filter $(CPU_BUILD)/src/Textures/%,$(CPU_OBJECTS))
FORCE:

$(CPU_CONFIG): FORCE
	@mkdir -p $(dir $@)
	@printf '%s\n' $(call cpu_quote,CXX=$(CXX)) $(call cpu_quote,$(shell $(CXX) --version | head -1)) $(call cpu_quote,COMPILE=$(CPU_COMPILE_FLAGS)) $(call cpu_quote,LINK=$(CPU_LINK_FLAGS) $(LDLIBS)) > $@.tmp
	@cmp -s $@.tmp $@ || mv $@.tmp $@
	@rm -f $@.tmp

$(CPU_BUILD)/%.o: %.cpp $(CPU_CONFIG) Makefile mk/cpu-config.mk
	@mkdir -p $(dir $@)
	$(CXX) $(CPU_COMPILE_FLAGS) -MMD -MP -c $< -o $@

$(CPU_OUTPUT): $(CPU_MAIN) $(CPU_OBJECTS) $(CPU_CONFIG)
	@mkdir -p $(dir $@)
	$(CXX) $(CPU_MAIN) $(CPU_OBJECTS) $(CPU_LINK_FLAGS) $(LDLIBS) -lm -o $@

$(CPU_BUILD)/tests/%: tests/%.cpp $(CPU_OBJECTS) $(CPU_CONFIG) main.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CPU_COMPILE_FLAGS) -I. $< $(CPU_OBJECTS) $(CPU_LINK_FLAGS) $(LDLIBS) -lm -o $@

check-cpu: $(CPU_TESTS)
	@set -e; for test in $(CPU_TESTS); do OMP_NUM_THREADS=4 $$test; done

clean:
	rm -f $(CPU_OUTPUT) $(CPU_MAIN) $(CPU_OBJECTS) $(CPU_MAIN:.o=.d) $(CPU_OBJECTS:.o=.d) $(CPU_CONFIG) $(CPU_TESTS)
	rm -f src/*.obj src/*.d src/Textures/*.obj src/Textures/*.d

-include $(CPU_MAIN:.o=.d) $(CPU_OBJECTS:.o=.d)

