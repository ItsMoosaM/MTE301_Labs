# Debugging in VS Code for WSL
In this folder, you'll learn how to setup your VS code in WSL to debug your code.

## Prerequisites
First, install gdb in your WSL ubuntu environment:
 ```
sudo apt-get install gdb
 ```
- press Y if prompted [Y/n]
  
Now, install the WSL extension for code. This follows the same steps from the top part of [this link](https://code.visualstudio.com/docs/remote/wsl). Then open VS code in your WSL environment with ```code .``` and then in VS code, go to the extensions tab, which is the bottom icon on the left side, and search for **WSL**, and install it:

<img src="https://github.com/user-attachments/assets/7163a52a-4579-4970-a67d-4ffb21017407" width="400">

Now close VS code and restart it. It may install some new features before opening again. Then install the C/C++ extension. Note: you should see this extension under **WSL: UBUNTU**, and install it. Once it's installed, you should see it installed under **WSL: UBUNTU - INSTALLED**: 
![Installed packages](https://github.com/user-attachments/assets/18577cbb-6489-4ffd-940d-e12fe8f5a105)<br/>
You should also see at the bottom left of VS Code that it's working in the WSL environment:
![WSL installed](https://github.com/user-attachments/assets/5b983957-f65a-4e44-a61a-91f0965850e9)

## Set up the workspace for debugging

Finish the prerequisite steps before coming here. This setup is done for each workspace folder (Lab1 and Lab2 separately). You may notice in each of your Lab1/Lab2 workspace folders that you have a **.vscode** folder with files, such as **settings.json**. If not, simple create it by right-clicking up top and creating a new folder (it has to be the exact name **.vscode**!). (Side note: although the folders for .vscode could have been included in the repository, students may have already set up their own configuration files, so these weren't included in the Lab1 and Lab2 folders to automatically overwrite them. Instead, this is setup to manually create a brand new configuration). Create files called **launch.json** and **tasks.json**. After creating them in Lab1 it should look like the following:<br/>
<img src="https://github.com/user-attachments/assets/71040296-fea2-450e-b063-5c8b6cb8ca5a" width="400"><br/>
Now, copy and paste the contents of **launch.json** and **tasks.json** from this folder under Lab1 onto the respective files you created in the previous step: <br/>
<img src="https://github.com/user-attachments/assets/7d536eb7-d7ab-4ad6-997f-6026cb6e6c11" width="500"><br/>
For example, **tasks.json** should look like: 
<img src="https://github.com/user-attachments/assets/b375eede-b9d1-487f-8763-0a2c55d9044b" width="800"><br/>
The same steps can be done for Lab2 with their respective json files. 
Now you can debug by clicking the debug icon (fourth from the top) and pressing the green play button:<br/>
<img src="https://github.com/user-attachments/assets/c35780ba-6798-4044-9361-ccf71b0e3869" width="500"><br/>

## Example debugging.

The most basic example is simply to run the debugger by pressing the play button. Below is an example of running the debugger on the lab1.cpp template code and pausing it. On the top left side under **VARIABLES** and **Locals**, you can see the variables in the local scope when it's paused, including vectors and integers used.
![Basic debug](https://github.com/user-attachments/assets/3665181a-3b73-4f25-ac86-12d47be8bea9)

A good way to keep track of your variables is to use breakpoints. To make a breakpoint, go to the left side of your line where you want to add a breakpoint and you should see a red circle. Click on it and you'll create a breakpoint. Below is an example where breakpoints were added to keep track of the integers ```robot.x``` and ```count``` as they are incremented step by step. The blue play/pause button is used to step over until the next breakpoint or run the code indefinitely. You can see the values of these integers being incremented each step:
![Breakpoints](https://github.com/user-attachments/assets/6e792d6d-cc77-44e9-9281-b19a1d24c1ef)<br/>
It is useful to add breakpoints within function calls so users can keep track of variables as they're passed around their code. 

These are the most basic examples. For more information on debugging, you can visit https://code.visualstudio.com/docs/cpp/cpp-debug
