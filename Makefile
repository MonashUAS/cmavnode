BIN = bin/cmavnode
CPP_FILES := $(wildcard src/*.cpp)
OBJ_FILES := $(addprefix obj/,$(notdir $(CPP_FILES:.cpp=.o)))
LD_FLAGS := -std=c++14 -ggdb -lboost_program_options -lpthread -lboost_thread -lboost_system
CC_FLAGS := -std=c++14 -ggdb -DELPP_NO_DEFAULT_LOG_FILE -DELPP_STACKTRACE_ON_CRASH -DELPP_THREAD_SAFE

all: $(BIN)

$(BIN): $(OBJ_FILES)
	g++ $(LD_FLAGS) -o $@ $^

obj/%.o: src/%.cpp
	g++ $(CC_FLAGS) -c -o $@ $<

clean:
	rm -f $(BIN) $(OBJ_FILES) 
