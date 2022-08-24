#include<random>

//pso params
#define A 0.5
#define B 0.8
#define C 0.9
#define MAX_IT 150
#define LB -10000
#define UB 10000

struct Particle{
	//position
	float x;
	float y;

	//velocity
	float v_x;
	float v_y;

	//local best values
	float local_opt;
	float l_x;
	float l_y;
	
	//bounds
	float lower_bound;
	float upper_bound;

	//Pointer to the random number generator
	std::default_random_engine* generator;

	//Init variables
	Particle(std::function<float(float,float)> f, float _lower_bound, float _upper_bound, std::default_random_engine& _generator):generator(&_generator){
		lower_bound = _lower_bound;
		upper_bound = _upper_bound;

		x = R(lower_bound, upper_bound);
		y = R(lower_bound, upper_bound);
		
		v_x = R(-abs(upper_bound - lower_bound), abs(upper_bound - lower_bound));
		v_y = R(-abs(upper_bound - lower_bound), abs(upper_bound - lower_bound));		
	
		local_opt = f(x,y);
		l_x = x;
		l_y = y;
	}

	void update_velocity(float a, float b, float c, float g_x, float g_y){
		auto update = [&a, &b, &c, this](float i, float v_i, float l_i, float g_i){return (a * v_i) + (b * R() * (l_i - i)) + (c * R() * (g_i - i));};
		
		v_x = update(x, v_x, l_x, g_x);
		v_y = update(y, v_y, l_y, g_y);		
	}

	void update_position(){
		auto update = [](float i, float v_i){return i + v_i;};
		auto fixed_collision = [this](float i){
			if(i < lower_bound){
				return lower_bound;
			}
			else if(i > upper_bound){
				return upper_bound;
			}
			else return i;
		};

		x = update(x, v_x);
		y = update(y, v_y);

		x = fixed_collision(x);
		y = fixed_collision(y);
	}

	bool evaluate_local_opt(std::function<float(float,float)> f){
		float eval = f(x, y);

		if(eval < local_opt){
			local_opt = eval;
            l_x = x;
			l_y = y;

			return true;
		}
		else return false;
	}

	//return a random number drawn from a uniform distribution in [lower_bound, upper_bound)
	//DEFAULT: return a uniform random number in [0, 1)
	//NOTE: since distribution objects are lightweight, constructing a new one everytime should not impact the performance
	float R(float lower_bound=0.0, float upper_bound=1.0){
		return std::uniform_real_distribution<float>{lower_bound, upper_bound}(*generator);
	}
};

//Performance tools

//sum of the vector elements
int vector_sum(std::vector<int> v){
	int res=0;
	
	for(int i=0; i<v.size(); i++){
		res += v[i];
	}

	return res;
}

//average of the vector elements
float vector_avg(std::vector<int> v){
    float res= (float) vector_sum(v);
    int n=v.size();

    return res/n;
}

//print a vector in a readable way
void print_vector(std::vector<int> v){
    for(int i=0; i<v.size(); i++){
        std::cout << v[i] << " ";
	}

    std::cout << std::endl;
}
