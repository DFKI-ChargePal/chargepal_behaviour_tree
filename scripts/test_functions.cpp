#include <iostream>
#include <chrono>
#include <ros/ros.h>
#include <ros/package.h>
#include "util.h"
#include <string>

int main(int argc, char **argv){

ros::init(argc, argv, "test_functions");
ros::NodeHandle nh;

set_robot_value("ChargePal1","current_job","BRING_CHARGER");

return 1;
}
