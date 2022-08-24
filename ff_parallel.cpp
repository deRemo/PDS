#include<iostream>
#include<vector>
#include<functional>
#include<ff/ff.hpp>

#include"pso_utils.hpp"
#include"utimer.hpp"

//Message class: used by the worker to communicate its group knowledge, used by the worker to communicate the best group knowledge (global knowledge)
struct SyncMessage{
    float opt;
    float x;
    float y;

    bool init_phase;

    SyncMessage(){
        opt = std::numeric_limits<float>::max();
        init_phase = true;
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
        //Create and send sync messages
        if(task == nullptr){
            for(int i=0; i<n_msg; i++){
                lb->ff_send_out_to(new SyncMessage(), i);
            }

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

        return 0;
    }

    void* svc(void* task){
        SyncMessage* msg = (SyncMessage*) task;
        
        //Send the initial group knowledge
        if(msg->init_phase == true){
            msg->opt = group_opt;
            msg->x = group_x;
            msg->y = group_y;
            msg->init_phase = false;

            return msg;
        }
		
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

        return (void*) msg;
    }
};

int main(int argc, char* argv[]){
    if(argc < 3){
		std::cout << "usage: ./prog particles_per_task nw" << std::endl;		
		return -1;
	}
    
    //user-defined params
    int particles_per_task = atoi(argv[1]);
    int nw = atoi(argv[2]);
    
    auto f = [](float x, float y){return (x*x) + (y*y) + 1;};

    //Create farm and nodes
    ff::ff_farm par;
    PSOMaster master(nw, MAX_IT, par.getlb());
    std::vector<ff::ff_node*> PSOWorkers;
    for(int i=0; i<nw; i++){
        PSOWorkers.push_back(new PSOWorker(particles_per_task, f, A, B, C, LB, UB));
    }

    //Setup farm
    par.add_workers(PSOWorkers);
    par.cleanup_workers(true);
    par.add_emitter(&master);
    par.remove_collector();

    par.wrap_around();
    par.set_scheduling_ondemand();

    //run farm
    {
        utimer u("FF parallel");
        par.run_and_wait_end();
    }

    std::cout << "global opt: " << master.global_opt << std::endl;

    return 0;
}
