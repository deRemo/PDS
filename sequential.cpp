#include<iostream>
#include<functional>
#include<vector>
#include<limits>
#include"pso_utils.hpp"
#include"utimer.hpp"

float PSO(int n_particles, std::function<float(float,float)> f, float a, float b, float c, int max_it, int lower_bound, int upper_bound){
	//Create random number generator
	std::default_random_engine generator(std::random_device{}());

	//Particle array
	std::vector<Particle> particles;
	particles.reserve(n_particles);
	
	//global best values
	float global_opt = std::numeric_limits<float>::max();
	float g_x;
	float g_y;

	//Init step: create particles and find the initial global optimum
	for(int i=0; i<n_particles; i++){
		Particle p(f, lower_bound, upper_bound, generator);
		particles.push_back(p);	 
		
		if(p.local_opt < global_opt){
			global_opt = p.local_opt;
			g_x = p.l_x;
			g_y = p.l_y;
		}
	}
	
	//Iterative step: For max_it iterations: For each particle: update velocity and position, then re-evaluate local and global optima
	//Init iteration best values
	float it_opt = global_opt;
	float it_x = g_x;
	float it_y = g_y;
	
	for(int i=0; i<max_it; i++){
		for(Particle& p: particles){
			//update velocity and position
			p.update_velocity(a, b, c, g_x, g_y);
			p.update_position();

			//evaluate local_opt and it_opt
			if(p.evaluate_local_opt(f)){
				if(p.local_opt < global_opt){
					it_opt = p.local_opt;
					it_x = p.l_x;
					it_y = p.l_y;
				}
			}
		}

		//assign best it values to the global values
		global_opt = it_opt;
		g_x = it_x;
		g_y = it_y;
	}
	
	return global_opt;
}

int main(int argc, char* argv[]){
	if(argc < 2){
		std::cout << "usage: ./prog n_particles" << std::endl;		
		return -1;
	}
	
	//user-defined param
	int n_particles = atoi(argv[1]);
	
	auto f = [](float x, float y){return (x*x) + (y*y) + 1;};
	{
		utimer u("Sequential");
		std::cout << "global opt: " << PSO(n_particles, f, A, B, C, MAX_IT, LB, UB) << std::endl;
	}

	return 0;
}
