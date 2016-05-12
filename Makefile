BIN = bin/cmavnode
CPP_FILES := $(wildcard src/*.cpp)
OBJ_FILES := $(addprefix obj/,$(notdir $(CPP_FILES:.cpp=.o)))
LD_FLAGS := -std=c++14 -ggdb -lboost_program_options -lpthread -lboost_thread -lboost_system
CC_FLAGS := -std=c++14 -ggdb -DELPP_NO_DEFAULT_LOG_FILE -DELPP_STACKTRACE_ON_CRASH -DELPP_THREAD_SAFE

all: CC_FLAGS += -DNDEBUG
all: $(BIN)

debug: $(BIN)

$(BIN): $(OBJ_FILES)
	g++ -o $@ $^ $(LD_FLAGS)

obj/%.o: src/%.cpp
	g++ -c -o $@ $< $(CC_FLAGS)

clean:
	rm -f $(BIN) $(OBJ_FILES) 
