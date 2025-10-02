# Define object files
OBJ = lab3.o utils.o render.o
OBJ_temp = lab3_template.o utils.o render.o
OBJ_sln = lab3_sln.o utils.o render.o

# Define the final executable target
lab3: $(OBJ)
	g++ -g -o lab3 $(OBJ) -lsfml-graphics -lsfml-window -lsfml-system

lab3_template: $(OBJ_temp)
	g++ -g -o lab3_template $(OBJ_temp) -lsfml-graphics -lsfml-window -lsfml-system

lab3_sln: $(OBJ_sln)
	g++ -g -o lab3_sln $(OBJ_sln) -lsfml-graphics -lsfml-window -lsfml-system

# Compile object files separately
lab3.o: lab3.cpp
	g++ -g -c lab3.cpp

lab3_template.o: lab3_template.cpp
	g++ -g -c lab3_template.cpp

lab3_sln.o: lab3_sln.cpp
	g++ -g -c lab3_sln.cpp

utils.o: utils.cpp
	g++ -g -c utils.cpp

render.o: render.cpp
	g++ -g -c render.cpp

debug_app: lab3.cpp
	g++ -g -O0 -fsanitize=address,undefined -c lab3.cpp  utils.cpp render.cpp
	g++ -g -O0 -fsanitize=address,undefined lab3.o utils.o render.o -o debug_app -lsfml-graphics -lsfml-window -lsfml-system

debug_grid: robot_grid_eff.cpp
	g++ -g -O0 -fsanitize=address,undefined -c robot_grid_eff.cpp  utils.cpp render.cpp
	g++ -g -O0 -fsanitize=address,undefined robot_grid_eff.o utils.o render.o -o debug_grid -lsfml-graphics -lsfml-window -lsfml-system

clean:
	rm *.o lab3

