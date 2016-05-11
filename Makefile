SRC = $(shell find . -name *.cpp)

OBJ = $(SRC:%.cpp=%.o)

BIN = bin/cmavnode


all: $(BIN)

$(BIN): $(OBJ)
	g++ -std=c++14 -ggdb $(OBJ) -o $(BIN) -lboost_program_options -lpthread -lboost_thread -lboost_system


%.o: %.cpp
	g++ -std=c++14 -c -ggdb  $< -o $@ -Iinclude/mavlink

clean:
	$(RM) $(BIN) src/*.o *~

