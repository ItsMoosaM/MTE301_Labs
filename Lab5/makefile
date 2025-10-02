# Define object files
OBJ = lab5.o utils.o render.o

# Define the final executable target
lab5: $(OBJ)
	g++ -g -o lab5 $(OBJ) -lsfml-graphics -lsfml-window -lsfml-system

# Compile object files separately
lab5.o: lab5.cpp
	g++ -g -c lab5.cpp
	
utils.o: utils.cpp
	g++ -g -c utils.cpp

render.o: render.cpp
	g++ -g -c render.cpp

clean:
	rm *.o lab5

