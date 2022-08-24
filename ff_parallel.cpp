#include<iostream>
#include<vector>
#include<functional>
#include<cmath>
#include<ff/ff.hpp>

#include"pso_utils.hpp"
#include"utimer.hpp"

//Message class: used by the worker to communicate its group knowledge, used by the worker to communicate the best group knowledge (global knowledge)
struct SyncMessage{
    float opt;
    float x;
    float y;

    SyncMessage(){
        opt = std::numeric_limits<float>::max();
    }
};


struct PSOMaster: ff::ff_node {
    ff::ff_loadbalancer *const lb; 

    int n_msg; //messages per iteration
    int msg_count; //current number of received messages
    std::queue<SyncMessage*> waiting_queue;

    int max_it; //max number of iterations
    int it; //current iteration

    //global best values
	float global_opt;
	float g_x;
	float g_y;
	
    PSOMaster(int _n_msg, int _max_it, ff::ff_loadbalancer *const _lb): lb(_lb){
        global_opt = std::numeric_limits<float>::max();

        n_msg = _n_msg;
        msg_count = 0;

        max_it = _max_it;
        it = 0;
    }

    void* svc(void* task){
        if(task == nullptr){
			return GO_ON;
        }
				
		msg_count++;
        SyncMessage* msg = (SyncMessage*) task;
        waiting_queue.push(msg);

        //re-evaluate global opt
        if(msg->opt < global_opt){
			global_opt = msg->opt;
			g_x = msg->x;
			g_y = msg->y;
		}
        
		//Check if all the workers finished the current iteration
        if(msg_count == n_msg){
            it++;
            
			//check for termination(note: +1 because the initial step is included in the count
            if(it == max_it + 1){
                //delete messages
                while(!waiting_queue.empty()){
                    delete waiting_queue.front();
                    waiting_queue.pop();
                }
				
                return EOS;
            }
            else{ //sync best values between workers and start new iteration
				msg_count = 0;
				
                int i = 0;
                while(!waiting_queue.empty()){
                    SyncMessage* msg = waiting_queue.front();

                    msg->opt = global_opt;
                    msg->x = g_x;
                    msg->y = g_y;

                    waiting_queue.pop();
                    lb->ff_send_out_to(msg, i);
                    i++;
                }
			}
        }
		
        return GO_ON;
    }
};

struct PSOWorker: ff::ff_node{
    std::vector<Particle> particles;
    int n_particles;
	std::function<float(float,float)> f;

	float a;
	float b;
	float c;

    float lower_bound;
    float upper_bound;

    //particle group best values
    float group_opt;
	float group_x;
	float group_y;
    
    //thread random number generator
    std::default_random_engine generator;
	
	//performance
	std::vector<int> Ts;

    PSOWorker(int _n_particles, std::function<float(float,float)> _f, float _a, float _b, float _c, float _lower_bound, float _upper_bound):generator(std::random_device{}()){
        n_particles = _n_particles;
        particles.reserve(_n_particles);

        f = _f;

		a = _a;
		b = _b;
		c = _c;

        lower_bound = _lower_bound;
        upper_bound = _upper_bound;
        
        group_opt = std::numeric_limits<float>::max();
    }

    //Init step: create a set of particles and find the initial global optimum
    int svc_init(){
		for(int i=0; i<n_particles; i++){
			Particle p(f, lower_bound, upper_bound, generator);
			particles.push_back(p);

            if(p.local_opt < group_opt){
                group_opt = p.local_opt;
                group_x = p.l_x;
                group_y = p.l_y;
		    }
		}
	
		//create the sync message	
		SyncMessage* msg = new SyncMessage();
		
		msg->opt = group_opt;
        msg->x = group_x;
        msg->y = group_y;

		ff_send_out(msg);
        return 0;
    }
	
    void* svc(void* task){
       	SyncMessage* msg = (SyncMessage*) task; 
		{
            silent_utimer u;
            
            //Iterative step
            //iteration best values
            float it_opt = msg->opt;
            float it_x = msg->x;
            float it_y = msg->y;

            for(Particle& p : particles){
                //update velocity and position
                p.update_velocity(a, b, c, msg->x, msg->y);
                p.update_position();

                //evaluate local_opt and global_opt
                if(p.evaluate_local_opt(f)){
                    if(p.local_opt < msg->opt){
                        it_opt = p.local_opt;
                        it_x = p.l_x;
                        it_y = p.l_y;
                    }
                }
            }
        
            //assign best iteration values to the message
            msg->opt = it_opt;
            msg->x = it_x;
            msg->y = it_y;

            Ts.push_back(u.get_current_micros());
		}
		return (void*) msg;
    }
};

int main(int argc, char* argv[]){
    if(argc < 3){
		std::cout << "usage: ./prog n_particles nw" << std::endl;		
		return -1;
	}
    
    //user-defined params
    int n_particles = atoi(argv[1]);
    int nw = atoi(argv[2]);
    int particles_per_group = ceil(float(n_particles)/float(nw));
    
    auto f = [](float x, float y){return (x*x) + (y*y) + 1;};
	
	int Tc = 0;
	{
    	silent_utimer u;

		//Create farm and nodes
    	ff::ff_farm par;
    	PSOMaster master(nw, MAX_IT, par.getlb());
    	std::vector<ff::ff_node*> workers;
    	for(int i=0; i<nw; i++){
        	workers.push_back(new PSOWorker(particles_per_group, f, A, B, C, LB, UB));
    	}
	
    	//Setup farm
    	par.add_workers(workers);
   	 	par.cleanup_workers(true);
    	par.add_emitter(&master);
    	par.remove_collector();

    	par.wrap_around();

    	//run farm
        par.run_and_wait_end();
		Tc = u.get_current_micros();

    	//performance
		std::cout << "completion time: " << Tc / 1000 << std::endl;

    	int Texec_w = 0;
    	for(int i=0; i<workers.size(); i++){
        	PSOWorker* w = (PSOWorker*) workers[i];
			Texec_w += vector_sum(w->Ts);
    	}
    	Texec_w = Texec_w / nw;
	
		std::cout << "Avg. worker exec. time: " << Texec_w / 1000 << std::endl;
		std::cout << "Master exec. time " << (Tc - Texec_w) / 1000<< std::endl;
    	std::cout << "global opt: " << master.global_opt << std::endl;
	}
    return 0;
}
