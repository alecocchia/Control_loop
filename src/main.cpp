#include "control_loop.h"
#include <vector>

using namespace std;

//the goal is to implement a simple PID controller, 
//running on input value to reach the desired one

// Sense: read a value from keyboard
// Plan:  generate the correct input
// Act:   set the input

int main(int argc, char** argv) {
    float initial_reference=50;
    float reference;

    if (argc>1) {
        initial_reference = atof(argv[1]);
    }

    CONTROLLER controller(1, 0.1, 0.001, initial_reference);

    vector<boost::thread> threads;
    string termination;

    while(controller.getexit()==false){

    }
    return 0;
}