# lab1: lab1.cpp
# 	g++ -g -c lab1.cpp utils.cpp render.cpp
# 	g++ lab1.o utils.o render.o -o lab1 -lsfml-graphics -lsfml-window -lsfml-system

# Define object files
OBJ = lab1.o utils.o render.o

# Define the final executable target
lab1: $(OBJ)
	g++ -g -o lab1 $(OBJ) -lsfml-graphics -lsfml-window -lsfml-system

# Compile object files separately
lab1.o: lab1.cpp
	g++ -g -c lab1.cpp

utils.o: utils.cpp
	g++ -g -c utils.cpp

render.o: render.cpp
	g++ -g -c render.cpp

debug_app: lab1.cpp
	g++ -g -O0 -fsanitize=address,undefined -c lab1.cpp  utils.cpp render.cpp
	g++ -g -O0 -fsanitize=address,undefined lab1.o utils.o render.o -o debug_app -lsfml-graphics -lsfml-window -lsfml-system

clean:
	rm *.o lab1

