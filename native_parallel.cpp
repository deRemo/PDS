#include<iostream>
#include<vector>
#include<queue>
#include<thread>
#include<atomic>
#include<random>
#include<functional>
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

    bool init_phase;

    SyncMessage(){
        opt = std::numeric_limits<float>::max();
        init_phase = true;
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
			//Create and send sync messages
            for(int i=0; i<n_msg; i++){
                SyncMessage* msg = new SyncMessage();
                send_to(msg, i);
            }
			
            while(it < max_it + 1){
                //collect messages
				while(msg_count < n_msg){
					int i = get_id();
                    auto msg = get_from(i); //busy wait for messages
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
                int i = get_id();

                auto msg = get_from(i);
                msg_count++;
                send_to(EOS, i);
                delete msg;
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

    //Returns the id of a worker that finished the iteration in a round robin fashion
    int get_id(){
        while(in_queues.at(cnt % in_queues.size())->is_empty()){
            cnt++;
        }

        return cnt % in_queues.size();
    }

    //Get message from worker i
    SyncMessage* get_from(int i){
        return in_queues.at(i)->pop();
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

            //wait for the sync message
            auto msg = in_q->pop();
            while(msg != EOS){
				//First phase: synchronize the initial knowledge
                if(msg->init_phase == true){
                    msg->opt = group_opt;
                    msg->x = group_x;
                    msg->y = group_y;

                    msg->init_phase = false;
                }
                else{
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
                }

                //send to emitter
                out_q->push(msg);
                
				//wait for next iteration
                msg = in_q->pop();
            }
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
		std::cout << "usage: ./prog particles_per_task nw" << std::endl;		
		return -1;
	}
    
    //user-defined params
    int particles_per_task = atoi(argv[1]);
    int nw = atoi(argv[2]);

    auto f = [](float x, float y){return (x*x) + (y*y) + 1;};

    //Init core picker
    CorePicker core_picker;
    PSOParallel par(particles_per_task, nw, f, A, B, C, MAX_IT, LB, UB);
    par.set_picker(&core_picker);
    {
        utimer u("Native parallel");
        par.run_and_wait_end();
    }

    std::cout << "global opt: " << par.master->global_opt << std::endl;
    return 0;
}
