set -x
g++ -std=c++11 -O3 sequential.cpp -o sequential
g++ -std=c++11 -O3 -pthread native_parallel.cpp -o native_parallel
g++ -std=c++11 -O3 -pthread decentralized_parallel.cpp -o decentralized_parallel

export FF_ROOT=./
g++ -std=c++11 -I$FF_ROOT -O3 -pthread ff_parallel.cpp -o ff_parallel
