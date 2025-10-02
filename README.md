# Labs for MTE301: Programming for Mechatronics
## Table of Contents

[Workspace setup](https://github.com/TakShimoda/MTE301_Lab?tab=readme-ov-file#workspace-setup)<br />
[Lab 1: Elementary Robotic Navigation](https://github.com/TakShimoda/MTE301_Lab?tab=readme-ov-file#lab-1-elementary-robotic-navigation)<br /> 
[Lab 2: Obstacle Avoidance](https://github.com/TakShimoda/MTE301_Lab?tab=readme-ov-file#lab-2-obstacle-avoidance)<br /> 
[Lab 3: Mapping and Wall Following](https://github.com/TakShimoda/MTE301_Lab?tab=readme-ov-file#lab-3-mapping-and-wall-following)<br />
[Lab 4: Mapping, Wall Following, and Sweeping](https://github.com/TakShimoda/MTE301_Lab/blob/master/README.md#lab-4-mapping-wall-following-and-sweeping)<br />
[Lab 5: Mapping, Wall Following, and Sweeping with Obstacles](https://github.com/TakShimoda/MTE301_Lab/blob/master/README.md#lab-5-mapping-wall-following-and-sweeping-with-obstacles)


## Workspace setup and Information
### Basic setup
Follow the same steps discussed in the lab manuals. What you do may be different based on whether it's the first time cloning, or you did parts of lab 1 and want to re-clone the repository for lab 2 while keeping the changes to lab1.cpp. In that case, follow the lab 2 manual closely.

In general, you clone the repository:
```
git clone https://github.com/TakShimoda/MTE301_Lab
```
Then, you go to your workspace and open Visual Studio Code with the following:
```
cd ~/MTE301_Lab/Lab1 #assuming you cloned the repository in the root directory, and you're working on lab 1
code . # open VS code
```

### Extra functionalities
**Pause and rewind/fast forward**: at any time the SFML window is open when you execute your code, you can pause it by pressing ```p``` on your keyboard. Press ```p``` again to unpause it. While it's paused, you can also rewind or fast-forward by pressing the left and right arrow keys to carefully observe how your robot is moving each iteration.

### Debugger
Optionally, if you want to use the debugger to observe the stack trace and keep track of variables to observe if your code is working as intended, go to the [VSCode_Files](https://github.com/TakShimoda/MTE301_Lab/tree/master/VSCode_Files) folder and follow the steps there.
## Lab 1: Elementary Robotic Navigation


https://github.com/user-attachments/assets/d398575e-7e02-49e8-b974-e67d268c7cd2
 - **Task 1**: detect wall collision. Robot above goes up until it hits top wall and exits with failure when it does.

https://github.com/user-attachments/assets/62cf708c-e786-4f71-bcaf-82167b891e60
- **Task 2**: detect goal collision. Robot above goes up until it hits goal and exits with success when it does. (Note: in the above scenario, it just happened the goal was directly above the robot, but you should demonstrated this with task 3/4 navigation methods that ensure the robot goes towards the goal.)

https://github.com/user-attachments/assets/2217eca1-e557-4d3e-a85f-c2e60817cbe6
 - **Task 4**: robot goes up, then horizontally towards the goal. Task 3 is the same but in opposite order.

https://github.com/user-attachments/assets/af54f692-4a1b-4d45-a296-7cb2ec9a2790
- **Task 5**: case 1 (1<abs(del_y/del_x)<2). Move diagonally 1 pixel at a time in x and y, then after del_x = 0, move in the y direction the rest of the way. 


https://github.com/user-attachments/assets/46eade8e-574b-47ef-aaca-8b5725ed6df2
- **Task 5**: case 2 (1<abs(del_x/del_xy)<2). Move diagonally 1 pixel at a time in x and y, then after del_y = 0, move in the x direction the rest of the way. 

https://github.com/user-attachments/assets/ef3a64a3-c080-4cef-8ee1-0eef8e8caf0b
- **Task 6**: robot goes in diagonal motion, then vertical towards the center of the goal. The diagonal motion is fast, because abs(del_y/del_x) rounds down to 3, so it moves 3 pixels at a time in the y direction.

https://github.com/user-attachments/assets/f9a064b1-2790-462f-888d-170d1cf10110
- **Task 7**: robot moves towards the corner with the shortest distance. In this case, it's the bottom right corner.

After you finish your code (assuming you cloned this repository to the root directory):

```cd ~/MTE301_Lab/Lab1```

```make lab1```

```./lab1```

## Lab 2: Obstacle Avoidance



https://github.com/user-attachments/assets/6b80ef97-2bbb-4bfc-9e9f-6d047793460f
- **task 3**(bonus): obstacle avoidance that always results in shortest path to goal.

https://github.com/user-attachments/assets/35c35620-7641-449a-8424-d62ef37bc5e9
- Example obstacle avoidance with bonus task 3 (obstacle avoidance resulting in shortest path to goal) as well as mechanisms to avoid the robot getting stuck (note: your robot may get stuck at times even with tasks 2 and 3, but fixing that is beyond the scope of the lab.)

After you finish your code (assuming you cloned this repository to the root directory):

```cd ~/MTE301_Lab/Lab2```

```make lab2```

```./lab2```


## Lab 3: Mapping and Wall Following


https://github.com/user-attachments/assets/6788a339-72bd-4ee5-8c9a-6e1ea28d06f5

## Lab 4: Mapping, Wall Following, and Sweeping



https://github.com/user-attachments/assets/4c89cb2d-89fd-4d07-bb34-1239c4088c5c

- **Tasks 1 and 2, Environment 1**: robot follows the walls while mapping (task 1). Following task 1, the robot initiates the sweeping algorithm and maps the interior of the walls. Rendered at 3x the speed.


https://github.com/user-attachments/assets/be91701d-bdd8-4727-8e93-c59a1177f68f

- **Task 1, Environment 2**: robot follows the walls while mapping. Rendered at 4x the speed.


https://github.com/user-attachments/assets/6a1b68a9-fce4-4291-b47b-c2a1f5df7ae6

- **Task 2, Environment 2**: following task 1, robot initiates the sweeping algorithm and maps the interior of the walls. Rendered at 4x the speed.



https://github.com/user-attachments/assets/d4c64972-949f-4e56-a739-c0347e0f749d

- **Tasks 1 and 2, Environment 3**: robot follows the walls while mapping. Following task 1, robot initiates the sweeping algorithm and maps the interior of the walls. Rendered at 3x the speed.

## Lab 5: Mapping, Wall Following, and Sweeping with Obstacles


https://github.com/user-attachments/assets/176cb120-7ab9-41f2-be74-52a30dddb0fb

- **Task 1**: robot sweeps the inner environment after wall following, while avoiding obstacles. Rendered at 3x the speed.

https://github.com/user-attachments/assets/40cd6f7e-6b59-477d-bcea-d7f431238b16

- **Task 2**: robot navigates to the goal, while avoiding obstacles. Rendered at 3x the speed.



