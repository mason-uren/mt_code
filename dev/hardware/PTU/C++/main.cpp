#include <iostream>
#include "PanTiltController.h"
#include <zmq.hpp>
#include <chrono>

using namespace std;
using namespace std::chrono;

// in microseconds
#define send_Rate_Period 50000

#define KP 250

#define VELOCITY_MODE 0
#define POSITION_MODE 1

int main() {
    int just_sent = 0;
    auto start = high_resolution_clock::now();

    bool Success;
    PanTiltController ptu ("192.168.0.104", "192.168.0.111", "3001", "4000", Success);

    if(Success == false){
        cout<<"Error: Cant Connect to PTU"<<endl;
        exit(1);
    }

    // If we can connect than create ZMQ ports
    // We publish the ptu angles
    void *contextx = zmq_ctx_new();
    void *publisher = zmq_socket(contextx, ZMQ_PUB);
    int HWM = 1;
    int bind = zmq_bind(publisher, "tcp://*:9001");
    zmq_setsockopt(publisher,ZMQ_SNDHWM,&HWM, sizeof(HWM));
    zmq_setsockopt(publisher,ZMQ_RCVHWM,&HWM, sizeof(HWM));

    // Setting Angles is done exclusively by one client
    zmq::context_t context (1);
    zmq::socket_t subscriber (context, ZMQ_SUB);
    subscriber.connect("ipc:///tmp/ptu_sub");
    subscriber.setsockopt(ZMQ_SUBSCRIBE,"",0);

    float angles[2];
    double angles_d[2];
    double angles_d_prev[2];

    double pan_velocity = 0;
    double tilt_velocity = 0;

    // forever get and set ports
    while(1) {
        // Always Broadcast Angles
        bool success = ptu.GetAngles(angles[0], angles[1]);

        // Convert angles to double so that it can be cross compatible with python
        angles_d[0] = (double) angles[0];
        angles_d[1] = (double) angles[1];

//        zmq_send(angle_publisher, &angles_d, sizeof(double) * 2, ZMQ_NOBLOCK);
//        zmq::message_t message(20);
//        snprintf ((char *) message.data(), 20 ,
//                  "%.6f %.6f", (double) angles[0], (double) angles[1]);
//        publisher.send(message);

        zmq_msg_t msg;
        zmq::message_t message(20);

        auto duration = duration_cast<microseconds>(high_resolution_clock::now() - start);
        if (duration.count() >= send_Rate_Period) {
            start = high_resolution_clock::now();

            if((angles_d_prev[0] != angles_d[0]) || (angles_d_prev[1] != angles_d[1])) {
                zmq_send(publisher, &angles_d, sizeof(double) * 2, ZMQ_NOBLOCK);
                cout<<"Pan "<<angles[0]<<endl;
                just_sent = 0;
            }else{
                zmq_send(publisher, &angles_d, sizeof(double) * 2, ZMQ_NOBLOCK);
            }
            cout<<"Pan "<<angles[0]<<endl;
//
//            if(just_sent != 0){
//                just_sent = 0;
//                zmq_send(publisher, &angles_d, sizeof(double) * 2, ZMQ_NOBLOCK);
//                //cout<<"Pan "<<angles[0]<<endl;
//            }



            angles_d_prev[0] = (double) angles[0];
            angles_d_prev[1] = (double) angles[1];
        }
        //cout<<"Tilt "<<angles[1]<<endl;


        // Get Angles or velocity from commander
        zmq::message_t update;

        if(subscriber.recv(&update, ZMQ_DONTWAIT)){
            float mode, pan, tilt;
            std::istringstream iss(static_cast<char*>(update.data()));
            iss >> mode >> pan >> tilt ;

            cout<<mode<<endl;

            if(mode == VELOCITY_MODE){
                std::string error_buffer;

                cout<<pan<<tilt<<endl;
                ptu.SetVelocities(pan, tilt, error_buffer);
                continue;
            }

            if(mode == POSITION_MODE){
                double pan_error = pan - angles_d[0];
                pan_velocity = KP*pan_error;

                if(pan_velocity < -12000){
                    pan_velocity = -12000;
                }

                if(pan_velocity > 12000){
                    pan_velocity = 12000;
                }

                double tilt_error = tilt - angles_d[1];
                tilt_velocity = KP*tilt_error;


                if(tilt_velocity < -12000){
                    tilt_velocity = -12000;
                }

                if(tilt_velocity > 12000){
                    tilt_velocity = 12000;
                }


                // ZMQ wants a string for returning Errors
                std::string error_buffer;
                ptu.SetVelocities(pan_velocity, tilt_velocity, error_buffer);
            }
        }

    }

    return 0;
}