//
// Created by adnan on 5/10/18.
//

#include "dvrk_arm/Arm.h"




int main(int argc, char** argv){
    DVRK_Arm mtm("MTMR");
    sleep(1);
    std::vector<double> joint_pos;
    std::vector<double> joint_vel;
    std::vector<double> joint_effort;
    double fx, fy, fz;
//    for (int cnt = 0 ; cnt < 5000 ; cnt++) {
//        mtm.measured_jp(joint_pos);
//        mtm.measured_jf(joint_effort);
//        std::cout << '\r' << "Effort [" ;
//        for (int i = 0; i < joint_pos.size(); i++) {
//            std::cout << " " << joint_effort[i];
//        }
//        std::cout << "]";
//        usleep(10000);
//    }
    for (int cnt = 0 ; cnt < 5000 ; cnt++) {
        mtm.measured_cf_force(fx, fy, fz);
        std::cout << '\r' << " Fx: " << fx << std::endl << " Fy: " << fy << std::endl << " Fz: " << fz;
        std::cout << std::endl << "---------------------------" << std::endl;
        usleep(10000);
    }

    std::cout << std::endl;
    return 0;
}