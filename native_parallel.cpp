#include<iostream>
#include<vector>
#include<queue>
#include<thread>
#include<atomic>
#include<random>
#include<functional>
#include <cmath> 
#include"pso_utils.hpp"
#include"sample_queue.hpp"
#include"utimer.hpp"

#define EOS nullptr

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

//Message class: used by the worker to communicate its group knowledge, used by the worker to communicate the best group knowledge (global knowledge)
struct SyncMessage{
    float opt;
    float x;
    float y;

    bool terminate;

    SyncMessage(){
        opt = std::numeric_limits<float>::max();
        terminate = false;
    }
};

struct PSOMaster{
    int n_msg; //messages per iteration
    int msg_count; //current number of received messages
    std::queue<SyncMessage*> waiting_queue;

    int max_it; //max number of iterations
    int it; //current iteration

    //global best values
	float global_opt;
	float g_x;
	float g_y;

    //emitter tools
    std::thread tid;
    std::vector<sample_queue<SyncMessage*>*> in_queues;
    std::vector<sample_queue<SyncMessage*>*> out_queues;
    unsigned cnt;
	
    PSOMaster(int _n_msg, int _max_it){
        global_opt = std::numeric_limits<float>::max();

        n_msg = _n_msg;
        msg_count = 0;

        max_it = _max_it;
        it = 0;

        cnt = 0;
    }

    void run(){
        auto body = [&](){
            while(it < max_it + 1){ //+1 because the iterative step is also included
				//collect messages
				while(msg_count < n_msg){
                    auto msg = in_queues.at(cnt++ % in_queues.size())->pop();
					msg_count++;
                   	 
					//re-evaluate global opt
                    if(msg->opt < global_opt){
                        global_opt = msg->opt;
                        g_x = msg->x;
                        g_y = msg->y;
                    }
					waiting_queue.push(msg);
                }
				
                //sync best values between workers and start new iteration
                msg_count = 0;
                sync_and_send();
                
				it++;
            }
			
			//cleanup messages and terminate workers
            while(msg_count < n_msg){
                auto msg = in_queues.at(cnt % in_queues.size())->pop();

				msg_count++;
				msg->terminate = true;
                send_to(msg, cnt++ % in_queues.size());
            }
        };

        tid = std::thread(body);
    }

    //Update the messages with the global knowledge and send to workers
    void sync_and_send(){
        int i = 0;
        while(!waiting_queue.empty()){
            SyncMessage* msg = waiting_queue.front();

            //Update best group values
            msg->opt = global_opt;
            msg->x = g_x;
            msg->y = g_y;
            waiting_queue.pop();
            
            send_to(msg, i);
            i++;
        }
    }

    //Send msg to worker i
    void send_to(SyncMessage* msg, int i){
        out_queues.at(i)->push(msg);
    }
	
    //Wait for the thread to finish
    void wait(){
        tid.join();
    }

    //Setup input queues
    void push_in_q(sample_queue<SyncMessage*>* q){
        in_queues.push_back(q);
    }

    //Setup output queues
    void push_out_q(sample_queue<SyncMessage*>* q){
        out_queues.push_back(q);
    }
};

struct PSOWorker{
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

    //worker tools
    std::thread tid;
    sample_queue<SyncMessage*>* in_q;
    sample_queue<SyncMessage*>* out_q;
	
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

    void run(){
        auto body = [&](){
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
			
			//Create and send sync message
			SyncMessage* init_msg = new SyncMessage();
			
			init_msg->opt = group_opt;
			init_msg->x = group_x;
			init_msg->y = group_y;
			out_q->push(init_msg);

            //wait for the response and start iterative step
            auto msg = in_q->pop();
            while(msg->terminate != true){
				{
					silent_utimer u;

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
                    
                    //assign best it values to the group values
                    msg->opt = it_opt;
                    msg->x = it_x;
                    msg->y = it_y;
                    
                    Ts.push_back(u.get_current_micros());
				}

				//send to emitter
                out_q->push(msg);
				
				//wait for next iteration
                msg = in_q->pop();
            }
			
			delete msg;
        };

        tid = std::thread(body);
    }

    //Wait for the thread to finish
    void wait(){
        tid.join();
    }

    //Setup in queue
    void set_in_q(sample_queue<SyncMessage*>* q){
        in_q=q;
    }

    //Setup out queue
    void set_out_q(sample_queue<SyncMessage*>* q){
        out_q=q;
    }
};

struct PSOParallel{
    PSOMaster* master;
    std::vector<PSOWorker*> workers;
    CorePicker* core_picker;

    PSOParallel(int n_particles_per_task, int nw, std::function<float(float,float)> f, float a, float b, float c, int max_it, float lower_bound, float upper_bound){
        //create nodes and set up communications
        master = new PSOMaster(nw, max_it);
        for(int i=0; i<nw; i++){
            PSOWorker* w = new PSOWorker(n_particles_per_task, f, a, b, c, lower_bound, upper_bound);

            //master->w
            sample_queue<SyncMessage*>* q0 = new sample_queue<SyncMessage*>;
            master->push_out_q(q0);
            w->set_in_q(q0);
            
            //w->master
            sample_queue<SyncMessage*>* q1 = new sample_queue<SyncMessage*>;
            master->push_in_q(q1);
            w->set_out_q(q1);
            
            workers.push_back(w);
        }

        core_picker = NULL;
    }

    //Cleanups
    ~PSOParallel(){
        delete master;

        for(int i=0; i<workers.size(); i++){
            delete workers[i]->in_q;
            delete workers[i]->out_q;
            delete workers[i];
        }
    }

    //Run model
    void run(){
        master->run();
        core_picker->stick(&(master->tid));

		for(int i=0; i<workers.size(); i++){
            workers[i]->run();
            core_picker->stick(&(workers[i]->tid));
        }
    }

    //Wait model
    void wait(){
        master->wait();
        for(int i=0; i<workers.size(); i++){
            workers[i]->wait();
        }
    }

    void run_and_wait_end(){
        run();
        wait();
    }

    void set_picker(CorePicker* picker){
        core_picker = picker;
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
	
	//Init core picker
	CorePicker core_picker;
	int Tc = 0;
	{
    	silent_utimer u;
		PSOParallel par(particles_per_group, nw, f, A, B, C, MAX_IT, LB, UB);
    	par.set_picker(&core_picker);

        par.run_and_wait_end();
		Tc = u.get_current_micros();
	
		std::cout << "completion time: " << Tc / 1000 << std::endl;

    	int Texec_w = 0;
		for(int i=0; i<par.workers.size(); i++){
			Texec_w += vector_sum(par.workers[i]->Ts);
		}
		Texec_w = Texec_w / nw;

		std::cout << "Avg. worker exec. time: " << Texec_w / 1000 << std::endl;
		std::cout << "Master exec. time " << (Tc - Texec_w) / 1000<< std::endl;
    	std::cout << "global opt: " << par.master->global_opt << std::endl;
    }

    return 0;
}
