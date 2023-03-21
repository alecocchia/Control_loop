#include "control_loop.h"

//We can use the class constructor to set parameters
CONTROLLER::CONTROLLER(const float &kp, const float &ki, const float &kd, const float &initial_value) {
    _kp=kp;
    _ki=ki;
    _kd=kd;
    set_xdes(initial_value);
    system_start();
    _y=_x;
    boost::thread loop_thread(&CONTROLLER::loop, this);
    boost::thread waiting_thread(&CONTROLLER::waiting, this);

}

//Decide if exit or not
bool CONTROLLER::getexit(){
    return  _exit;
}

//Sense: get input to change the state of our System
void CONTROLLER::set_xdes(double x) {
    _ref=x;
}


//Random initial value
void CONTROLLER::system_start() {
    srand((unsigned)(time(NULL)));
    _x = rand() % 300 +1;
}

void CONTROLLER::waiting(){
    string ref;
    while(1){
        getline(cin,ref);
        if (!ref.empty()){
            if (ref=="q"){
                _exit = true;
            } else {
                _mutex.lock();
                _a=1;
                this->set_xdes(stod(ref));
                cout<<"Acquiring reference\n";
                usleep(0.5*1e6);
                _a=0;
                _mutex.unlock();
                }
        }
    }
}

void CONTROLLER::loop() {

    float P = 0.0;
    float I = 0.0;
    float D = 0.0;

    float e = 0.0;
    float e_old =0.0;
    float xnew = 0.0;
    float Ts=0.01;
    int k=0;        //counting time

    ofstream of("PID_simulation.txt");

    if (!of){
        cerr<<"Impossible saving data on file PID_simulation.txt"<<endl;
    }

    cout<<"Stato iniziale: "<<_x<<"\n\n";
    cout<<"Press q + ENTER to exit or a number + ENTER to insert new reference\n"<<endl;
    cout<<"PID parameters:\tkp = "<<_kp<<"\tki = "<<_ki<<"\tkd = "<<_kd<<"\n"<<endl;
    cout<<"Program will start in 2 seconds . . ."<<endl;
    usleep(2*1e6);

    while (getexit()!=true){
        k=0;
        while(abs(_ref-_y)>0.005){
        //for(int i=0; i<100; i++){
            if(k!=0) {_x=xnew;}         //updating state if not first iteration

            if(_a==0){

                _y=_x;
                of<<_y<<endl;

                e_old=e;
            
                e =_ref -_y;

                //Actions

                P = e;

                I += e*Ts;            //Eulero's approximation for integral action

                D = (e - e_old)/Ts;

                // DEBUG //
        
                cout<<"Reference: "<<_ref<<"\t\t";
                cout<<"State: "<<_x<<"\t\t";
                cout<<"Error: "<< e<<"\n";

                cout<<"t = "<<k*Ts<<" s"<<endl<<endl;        

                _cmd =_kp*P +_ki*I + _kd*D;

                //simulating system
                xnew = _x +_cmd;

                k++;
            
                usleep(Ts*1e6);            //microsleep
            }
            of<<endl;
        }
    }
    of.close();
}

