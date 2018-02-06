----------------------
CMSC691 Assignment 4
----------------------
Xiaoxia Zheng / CE83376


----------------------
Command Line Arguments
----------------------
Example: ./main input.json output-%02d-%05d.obj

	"input.json" is the file that we need to input
	"output-%02d-%05d.obj" is the format that we output the files. For this project, you'll have 30 output files.

-----------------
Project sturcture
-----------------
1. Read all input files.
2. Loop all springs and store all node i and node j in one spring's force into a temp Mass structure.
3. Loop all mass nodes then update all nodes' force, velocity and new position.
4. Output files.


------------------------------------    
Problems I met and resource I use
------------------------------------
1. I read some papers online and some videos in youtube for helping me solve the project.
2. The most problem I met in this project is, I first didn't calculate the force from j to i in one spring. 
	I just calculete the force from i to j. That make the output totally not right. 

