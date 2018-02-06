-------------------
Spring Mass System
-------------------
For this project, there is a C++ program that will perform a 3D spring mass simulations. The object will fall under gravity (starting with zero initial velocity, gravity in the z direction) and collide with a plane at z=0. Collisions is constraints. That is, after updating positions set masspoint.z = max(masspoint.z, 0.0). Just use symplectic Euler to update the simulation through time.

v(t+delta_t) = v(t) + a(t) * delta_t
x(t+delta_t) = x(t) + v(t+delta_t) * delta_t

For force use

f_ij = stiffness * (mag(p_j-p_i) / l - 1) * (p_j-p_i) / mag(p_j-p_i) + damping * (v_j -v_i)

for the force on node i when there is a spring between nodes i and j.

Each timestep I zero out force accumulators, then loop over springs applying forces, then update velocities and positions of mass points. If it has been more than 1/30 of a second since the last output frame, write a new obj file with the current positions.



------------------
Input File Format
------------------
The main input file will be in the .json format. This file will specify the main simulation parameters and one or more spring-mass geometry files. 
Each line will begin with with either a m, s, or t. m is for mass-point, s is for spring, t is for triangle. Each m line will include the x, y, z initial position of the mass-point, and the mass of the mass-point.

m x y z volume
Each s line will give the index of node i, the index of node j, and rest length for the spring.

s i j l
Each t line will give the indices into the mass-point array of a surface triangle (which will be used for output as in the previous assignment).

t a b c
Unlike the obj format, all indices will be given assuming zero-offsets.



----------------------
Command Line Arguments
----------------------
Example: ./main input.json output-%02d-%05d.obj

	"input.json" is the file that we need to input
	"output-%02d-%05d.obj" is the format that we output the files. For this project, you'll have 30 
	output files.



