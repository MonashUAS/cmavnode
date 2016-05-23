BIN = bin/cmavnode
CPP_FILES := $(wildcard src/*.cpp)
OBJ_FILES := $(addprefix obj/,$(notdir $(CPP_FILES:.cpp=.o)))
LD_FLAGS := -std=c++11  -lboost_program_options -lpthread -lboost_thread -lboost_system
CC_FLAGS := -std=c++11  -DELPP_NO_DEFAULT_LOG_FILE -DELPP_STACKTRACE_ON_CRASH -DELPP_THREAD_SAFE

all: CC_FLAGS += -DNDEBUG
all: target = master
all: $(BIN)

muasmav: target = muasmav
muasmav: CC_FLAGS+= -DNDEBUG
muasmav: $(BIN)

debug: target = master
debug: CC_FLAGS += -ggdb
debug: LD_FLAGS += -ggdb
debug: $(BIN)

pre-build:
	git --git-dir=include/mavlink/.git --work-tree=include/mavlink/ checkout $(target)

$(BIN): $(OBJ_FILES)
	g++ -o $@ $^ $(LD_FLAGS)

obj/%.o: src/%.cpp pre-build
	g++ -c -o $@ $< $(CC_FLAGS)

clean:
	rm -f $(BIN) $(OBJ_FILES) 
