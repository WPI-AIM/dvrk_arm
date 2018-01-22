#include "dvrk_arm/Console.h"

DVRK_Console::DVRK_Console(){
}

void DVRK_Console::init_console(ros::NodeHandle *n){
}

DVRK_Console::~DVRK_Console(){
    std::cerr << "DESTROYING DVRK_CONSOLE" << std::endl;
}
