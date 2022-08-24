#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <ctime>

class utimer {
	std::chrono::system_clock::time_point start;
  	std::chrono::system_clock::time_point stop;
  	std::string message; 
  	using usecs = std::chrono::microseconds;
  	using msecs = std::chrono::milliseconds;
  
	public:

  	utimer(const std::string m) : message(m) {
    	start = std::chrono::system_clock::now();
  	}
	
	utimer(){
    	start = std::chrono::system_clock::now();
  	}

  	~utimer() {
    	stop = std::chrono::system_clock::now();
    	std::chrono::duration<double> elapsed = stop - start;
    	
		if(message.empty()){
			auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
			std::cout << msec << std::endl;
		}
		else{
			auto musec = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
			std::cout << message << " computed in " << musec << " usec" << std::endl;
		}
    }
};

class silent_utimer{
	std::chrono::system_clock::time_point start;
    std::chrono::system_clock::time_point stop;

    public:
    silent_utimer(){
        start = std::chrono::system_clock::now();
    }

    int get_current_micros(){
        stop = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed = stop - start;
        
		return std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
    }

	int get_current_millis(){
		stop = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed = stop - start;

        return std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
	}
};
