#include <iostream>
#include <fstream>
#include <string>
#include "boost/thread.hpp"

using namespace std;

class CONTROLLER {
    public:
        CONTROLLER(const float &, const float &, const float&, const float&);
        
        void loop();                //Main loop function
        void waiting();             //loop waiting for reference and acquiring it
        bool getexit();

        void system_start();       //start the system
        void set_xdes(double x);   //member to set the desired value

    private:
        float _kp;                  //proportional gain
        float _ki;                  //integral gain
        float _kd;                  //derivative gain
        float _initial_val;
        float _cmd;                 //control action
        float _ref;                 //reference
        float _y;                   //measurement
        float _x;                   //state

        mutex _mutex;               //mutex per acquisire il reference
        int _a=0;
        bool _exit = false;
};
