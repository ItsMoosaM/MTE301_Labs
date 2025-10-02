# Define object files
OBJ = lab4.o utils.o render.o

# Define the final executable target
lab4: $(OBJ)
	g++ -g -o lab4 $(OBJ) -lsfml-graphics -lsfml-window -lsfml-system

# Compile object files separately
lab4.o: lab4.cpp
	g++ -g -c lab4.cpp

utils.o: utils.cpp
	g++ -g -c utils.cpp

render.o: render.cpp
	g++ -g -c render.cpp

clean:
	rm *.o lab4

