# lab2: lab2.cpp
# 	g++ -g -c lab2.cpp utils.cpp render.cpp
# 	g++ lab2.o utils.o render.o -o lab2 -lsfml-graphics -lsfml-window -lsfml-system

# debug_app: lab2.cpp
# 	g++ -g -O0 -fsanitize=address,undefined -c lab2.cpp  utils.cpp render.cpp
# 	g++ -g -O0 -fsanitize=address,undefined lab2.o utils.o render.o -o debug_app -lsfml-graphics -lsfml-window -lsfml-system

# Define object files
OBJ = lab2.o utils.o render.o

# Define the final executable target
lab2: $(OBJ)
	g++ -g -o lab2 $(OBJ) -lsfml-graphics -lsfml-window -lsfml-system

# Compile object files separately
lab2.o: lab2.cpp
	g++ -g -c lab2.cpp

utils.o: utils.cpp
	g++ -g -c utils.cpp

render.o: render.cpp
	g++ -g -c render.cpp

clean:
	rm *.o lab2

