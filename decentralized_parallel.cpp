#include<iostream>
#include<thread>
#include<functional>
#include<mutex>
#include<condition_variable>
#include<vector>
#include<queue>
#include<limits>
#include<atomic>

#include"pso_utils.hpp"
#include"utimer.hpp"

//Used assign a thread to a core in a round robin fashion
struct CorePicker{
    unsigned int current_id; //currently picked core index
	unsigned int n_cores; //number of cores of the machines
	
	CorePicker(int _n_cores=std::thread::hardware_concurrency()){
		current_id = 0;
        n_cores = _n_cores;
	}

	//pick a core index in a round-robin fashion
	unsigned int pick(){
		return current_id++ % n_cores;
	}
    
    //stick core tid to the picked core
	void stick(std::thread *tid){
		unsigned int pickedCore = pick();
		cpu_set_t cpuset;
		CPU_ZERO(&cpuset);
		CPU_SET(pickedCore, &cpuset);
		int rc = pthread_setaffinity_np(tid->native_handle(), sizeof(cpu_set_t), &cpuset);

		if (rc != 0) {
			std::cerr << "Error calling pthread_setaffinity_np: " << rc << std::endl;
		}
	}
};

struct ParticleGroup{
    std::vector<Particle> particles;
	std::thread tid;
	
    int n_particles;
	std::function<float(float,float)> f;

	float a;
	float b;
	float c;

	int max_it;

    float lower_bound;
    float upper_bound;

    //particle group best values
    float group_opt;
	float group_x;
	float group_y;
    
    //thread random number generator
    std::default_random_engine generator;

    ParticleGroup(int _n_particles, std::function<float(float,float)> _f, float _a, float _b, float _c, int _max_it, float _lower_bound, float _upper_bound): generator(std::random_device{}()){
        n_particles = _n_particles;
        f = _f;

		a = _a;
		b = _b;
		c = _c;
        
        lower_bound = _lower_bound;
        upper_bound = _upper_bound;
        
		max_it = _max_it;
        group_opt = std::numeric_limits<float>::max();
    }

    template<class Fun1, class Fun2>
	void run(Fun1 sync, Fun2 stick_to_core){
		auto body = [sync, this](){
            //reserve space in particle array
            particles.reserve(n_particles);
            
            //Init step: create a set of particles and find the initial global optimum
            for(int i=0; i<n_particles; i++){
                Particle p(f, lower_bound, upper_bound, generator);
                particles.push_back(p);

                if(p.local_opt < group_opt){
                    group_opt = p.local_opt;
                    group_x = p.l_x;
                    group_y = p.l_y;
                }
            }
			
			//stop at barrier, it will sync the initial group values between the groups	
			sync(this);

            //Iterative step: For max_it iterations: For each particle: update velocity and position, then re-evaluate local and global optima
            //iteration's best values
            float it_opt = group_opt;
            float it_x = group_x;
            float it_y = group_y;

			for(int i=0; i<max_it; i++){
				for(Particle& p : particles){
                    //update velocity and position
                    p.update_velocity(a, b, c, group_x, group_y);
                    p.update_position();

                    //evaluate local_opt and global_opt
                    if(p.evaluate_local_opt(f)){
                        if(p.local_opt < group_opt){
                            it_opt = p.local_opt;
                            it_x = p.l_x;
                            it_y = p.l_y;
		                }
                    }
                }
                
                //assign best it values to the group values
                group_opt = it_opt;
                group_x = it_x;
                group_y = it_y;
                
				//stop at barrier, wait for next iteration (it will sync the group values with the global values) 
				sync(this);
			}
		};
		
		tid=std::thread(body);
        stick_to_core(&tid);
	}

	void wait(){
    	tid.join();
    }
};

//Global optimum and global best position of the swarm
//It also works as a barrier to sync every threads at the same iteration
struct GlobalKnowledge{ 
    int n_threads;
    int cnt;
    std::mutex mtx;
    std::condition_variable cv;
    std::queue<ParticleGroup*> q;

    //global best values
	float global_opt;
	float g_x;
	float g_y;

    //Initiliaze the best position and optimum of the swarm
	GlobalKnowledge(int _n_threads){
        global_opt = std::numeric_limits<float>::max();

        n_threads = _n_threads; //set the barrier max count;
        cnt = 0;
    }

    //Evaluate the global optimum to see if we have reached a better position
    //and block the particle group at the barrier. The next iteration stars when
    //all the groups are blocked.
    void evaluate_global_opt(ParticleGroup* p_group){
        std::unique_lock<std::mutex> lock(mtx);
        
		if(p_group->group_opt < global_opt){
			global_opt = p_group->group_opt;
			g_x = p_group->group_x;
			g_y = p_group->group_y;
		}

        //BARRIER
        if(n_threads > 1){
            cnt++;
            if(cnt < n_threads){
                cnt = 0; //reset barrier

                //Update best group values, before waking the threads
                while(!q.empty()){
                    ParticleGroup* p_group = q.front();
                    p_group->group_opt = global_opt;
                    p_group->group_x = g_x;
                    p_group->group_y = g_y;
                    q.pop();
                }

                cv.notify_all();
            }
            else{
                q.push(p_group);
                cv.wait(lock, [this]{return cnt < n_threads;});
            }
        }
    }
};

int main(int argc, char* argv[]){
    if(argc < 3){
		std::cout << "usage: ./prog particles_per_thread n_thread" << std::endl;		
		return -1;
	}

    //user-defined params
    int particles_per_thread = atoi(argv[1]);
    int n_threads = atoi(argv[2]);
    
    auto f = [](float x, float y){return (x*x) + (y*y) + 1;};
    
    //Init the global knowledge
    GlobalKnowledge global_knowledge(n_threads);
    auto sync = [&global_knowledge](ParticleGroup* p_group){ 
        global_knowledge.evaluate_global_opt(p_group);
    };

    //Init the core picker
    CorePicker picker;
    auto stick_to_core =[&picker](std::thread* tid){
        picker.stick(tid);
    };

    //Create particle groups
    std::vector<ParticleGroup> pg;
    for(int i=0; i<n_threads; i++){
        pg.push_back(ParticleGroup(particles_per_thread, f, A, B, C, MAX_IT, LB, UB));
    }

    //run
    {
        utimer u("Decentralized parallel");
        for(int i=0; i<n_threads; i++){
            pg[i].run(sync, stick_to_core);
        }

        for(int i=0; i<n_threads; i++){
            pg[i].wait();
        }
    }

    std::cout << "global opt: " << global_knowledge.global_opt << std::endl;

    return 0;
}
