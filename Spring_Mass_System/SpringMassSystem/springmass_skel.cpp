#include <iostream>
#include <fstream>
#include <vector>
#include "slVector.H"
#include "json.h"
#include <math.h>
#include <algorithm>


struct Mass {
	SlVector3 pos, vel, frc;
	double vol;
};

struct Spring {
	double l;
	int i, j;
};

class SlTri {
public:
	int indices[3];
	inline int &operator[](const unsigned int &i) { return indices[i];};
	inline int operator[](const unsigned int &i) const { return indices[i];};
	inline void set(int x, int y, int z) {indices[0] = x; indices[1] = y; indices[2] = z;};
	inline SlTri(int x, int y, int z) {indices[0] = x; indices[1] = y; indices[2] = z;};
	inline SlTri() {};
	inline SlTri &operator=(const SlTri &that);
};

inline SlTri &SlTri::operator=(const SlTri &that) {
	this->indices[0] = that.indices[0];
	this->indices[1] = that.indices[1];
	this->indices[2] = that.indices[2];
	return (*this);
};

struct SimulationParameters {
	double dt, total_time, k, d, density;
};

struct Object {
	std::vector<Mass> masses;
	std::vector<Spring> springs;
	std::vector<SlTri> triangles;
};

bool readObject(const char *fname, Object &object) {
	char ch;
	Mass m;
	Spring s;
	SlTri t;
	
	std::ifstream in(fname, std::ios::in);
//	std::ifstream in("/Users/zhengli/Desktop/CMSC691/project_4/proj4/input.ms", std::ios::in);
	
	while (in>>ch) {
		if (ch == 'm') {
	  in>>m.pos[0]>>m.pos[1]>>m.pos[2]>>m.vol;
	  object.masses.push_back(m);
			continue;
		}
		if (ch == 's') {
	  in>>s.i>>s.j>>s.l;
	  object.springs.push_back(s);
			continue;
		}
		if (ch == 't') {
	  in>>t[0]>>t[1]>>t[2];
	  object.triangles.push_back(t);
			continue;
		}
	}
	in.close();
	std::cout<<"inputfile " << fname <<" read"<<std::endl;
	return true;
}

bool readInputFile(const char *fname,
				   SimulationParameters &params,
				   std::vector<Object> &objects) {
	
	std::ifstream in(fname, std::ios::in);
	
	Json::Value root;
	Json::Reader jReader;
	
	if(!jReader.parse(in, root)){
		std::cout << "couldn't read input file: " << fname << '\n'
		<< jReader.getFormattedErrorMessages() << std::endl;
		exit(1);
	}
	
	params.dt = root.get("dt", 1.0/300.0).asDouble();
	params.total_time = root.get("total_time", 1.0).asDouble();
	params.k = root.get("stiffness", 1.0/300.0).asDouble();
	params.d = root.get("damping", 1.0/300.0).asDouble();
	params.density = root.get("density", 1.0/300.0).asDouble();
	
	Json::Value objectsIn = root["objects"];
	objects.resize(objectsIn.size());
	for (unsigned int i=0; i<objectsIn.size(); i++) {
		readObject((objectsIn[i]["filename"]).asString().c_str(), objects[i]);
	}
	return true;
}

void writeObj(char *fname, const std::vector<Mass> &meshPts, const std::vector<SlTri> &triangles) {
	std::cout<<"writing "<<fname<<std::endl;
	std::ofstream out;
	std::vector<Mass>::const_iterator p;
	std::vector<SlTri>::const_iterator t;
	
	out.open(fname);
	
	for (p=meshPts.begin(); p!=meshPts.end(); p++)
		out<<"v "<<p->pos[0]<<" "<<p->pos[1]<<" "<<p->pos[2]<<std::endl;
	
	for (t=triangles.begin(); t!=triangles.end(); t++)
		out<<"f "<<(*t)[0]+1<<" "<<(*t)[1]+1<<" "<<(*t)[2]+1<<std::endl;
	
	out.close();
}

int main(int argc, char *argv[]) {
	char fname[80];
	SimulationParameters params;
	std::vector<Object> objects;
	
	//read input file
	readInputFile(argv[1], params, objects);
	
	double time = 0;
	int frame = 0;
	
	while (time < params.total_time) {
		for (unsigned int i=0; i<objects.size(); i++) {
			Mass massinitial;
			std::vector<Mass> TempAllMasses(objects[i].masses.size(), massinitial);
			
			//loop all springs and sum up all force in node i and j.
			for (int springNum=0; springNum<objects[i].springs.size(); springNum++) {
				//Calculate new position spring's magnitude:
				// l = sqrt((x1-x0)^2 + (y1-y0)^2 + (z1-z0)^2)
				double springMag =  sqrt((objects[i].masses[objects[i].springs[springNum].j].pos[0] -
										 objects[i].masses[objects[i].springs[springNum].i].pos[0]) *
										(objects[i].masses[objects[i].springs[springNum].j].pos[0] -
										 objects[i].masses[objects[i].springs[springNum].i].pos[0]) +
										(objects[i].masses[objects[i].springs[springNum].j].pos[1] -
										 objects[i].masses[objects[i].springs[springNum].i].pos[1]) *
										(objects[i].masses[objects[i].springs[springNum].j].pos[1] -
										 objects[i].masses[objects[i].springs[springNum].i].pos[1]) +
										(objects[i].masses[objects[i].springs[springNum].j].pos[2] -
										 objects[i].masses[objects[i].springs[springNum].i].pos[2]) *
										(objects[i].masses[objects[i].springs[springNum].j].pos[2] -
										 objects[i].masses[objects[i].springs[springNum].i].pos[2]));
				
				
				//F = k * deltaX
				//deltaXRatio = mag(newPosition) / lengthOfSpring(original) - 1
				double deltaDistance = springMag - objects[i].springs[springNum].l;
				double deltaDisRatio = 0.0;
				if (std::abs(deltaDistance) < 0.00002) {
					 deltaDisRatio = 0;      //fix the numerical problem
				} else {
					deltaDisRatio =  deltaDistance / objects[i].springs[springNum].l;
				}
				
				
				//force_ij = k * (mag(p_j - p_i) / l - 1) * (p_j - p_i) / mag(p_j - p_i) + damping * (v_j - v_i)
				//force of node i in axis X:
				TempAllMasses[objects[i].springs[springNum].i].frc[0] += params.k * deltaDisRatio *
					(objects[i].masses[objects[i].springs[springNum].j].pos[0] -
					objects[i].masses[objects[i].springs[springNum].i].pos[0]) /
					springMag + params.d *
					(objects[i].masses[objects[i].springs[springNum].j].vel[0] -
					objects[i].masses[objects[i].springs[springNum].i].vel[0]);
				
				//force of node i in axis Y:
				TempAllMasses[objects[i].springs[springNum].i].frc[1] += params.k * deltaDisRatio *
					(objects[i].masses[objects[i].springs[springNum].j].pos[1] -
					objects[i].masses[objects[i].springs[springNum].i].pos[1]) /
					springMag + params.d *
					(objects[i].masses[objects[i].springs[springNum].j].vel[1] -
					objects[i].masses[objects[i].springs[springNum].i].vel[1]);
				
				// total force of axis Z = force of spring + damping + gravityZ
				// gravity of X and Y are 0.
				//force of node i in axis Z:
				TempAllMasses[objects[i].springs[springNum].i].frc[2] += params.k * deltaDisRatio *
					(objects[i].masses[objects[i].springs[springNum].j].pos[2] -
					objects[i].masses[objects[i].springs[springNum].i].pos[2]) /
					springMag + params.d *
					(objects[i].masses[objects[i].springs[springNum].j].vel[2] -
					objects[i].masses[objects[i].springs[springNum].i].vel[2]);
				
				
				
				//force of node j in axis X:
				TempAllMasses[objects[i].springs[springNum].j].frc[0] += params.k * deltaDisRatio *
				(objects[i].masses[objects[i].springs[springNum].i].pos[0] -
					objects[i].masses[objects[i].springs[springNum].j].pos[0]) /
				springMag + params.d *
				(objects[i].masses[objects[i].springs[springNum].i].vel[0] -
					objects[i].masses[objects[i].springs[springNum].j].vel[0]);
				
				//force of node j in axis Y:
				TempAllMasses[objects[i].springs[springNum].j].frc[1] += params.k * deltaDisRatio *
				(objects[i].masses[objects[i].springs[springNum].i].pos[1] -
					objects[i].masses[objects[i].springs[springNum].j].pos[1]) /
				springMag + params.d *
				(objects[i].masses[objects[i].springs[springNum].i].vel[1] -
					objects[i].masses[objects[i].springs[springNum].j].vel[1]);
				
				//force of node j in axis Z:
				TempAllMasses[objects[i].springs[springNum].j].frc[2] += params.k * deltaDisRatio *
				(objects[i].masses[objects[i].springs[springNum].i].pos[2] -
					objects[i].masses[objects[i].springs[springNum].j].pos[2]) /
				springMag + params.d *
				(objects[i].masses[objects[i].springs[springNum].i].vel[2] -
					objects[i].masses[objects[i].springs[springNum].j].vel[2]);
				
			}
			
			if (frame % 10 == 0) {
				sprintf(fname, argv[2], i, frame/10);
//				sprintf(fname, "/Users/zhengli/Desktop/CMSC691/project_4/output-%02d-%05d.obj", i, frame / 10);
				writeObj(fname, objects[i].masses, objects[i].triangles);
			}

			
			//loop all mass points
			//update every ponits' force, acceleration, velocity, position.
			for (int massNum = 0; massNum < objects[i].masses.size(); massNum++) {
				//Update force
				//gravity in axis Z
				objects[i].masses[massNum].frc[0] = TempAllMasses[massNum].frc[0];
				objects[i].masses[massNum].frc[1] = TempAllMasses[massNum].frc[1];
				objects[i].masses[massNum].frc[2] = TempAllMasses[massNum].frc[2] + (-9.8) * objects[i].masses[massNum].vol * params.density;

				//update velocity
				// a = F / m
				// m = v * density
				// v(t+delta_t) = v(t) + a(t) * delta_t
				objects[i].masses[massNum].vel[0] += objects[i].masses[massNum].frc[0] / (objects[i].masses[massNum].vol * params.density) * params.dt;
				objects[i].masses[massNum].vel[1] += objects[i].masses[massNum].frc[1] / (objects[i].masses[massNum].vol * params.density) * params.dt;
				objects[i].masses[massNum].vel[2] += objects[i].masses[massNum].frc[2] / (objects[i].masses[massNum].vol * params.density) * params.dt;

				//update position
				//x(t+delta_t) = x(t) + v(t+delta_t) * delta_t
				objects[i].masses[massNum].pos[0] += objects[i].masses[massNum].vel[0] * params.dt;
				objects[i].masses[massNum].pos[1] += objects[i].masses[massNum].vel[1] * params.dt;
				objects[i].masses[massNum].pos[2] = std::max( objects[i].masses[massNum].pos[2] + objects[i].masses[massNum].vel[2] * params.dt, 0.0);
			}
			
		}
		
		time += params.dt;
		frame++;
	}
}







