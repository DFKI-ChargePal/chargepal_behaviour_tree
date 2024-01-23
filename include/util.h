#ifndef MY_FUNCTIONS_H
#define MY_FUNCTIONS_H

#include "ros/ros.h"
#include <string> 

std::string read_robot_value(const std::string& name,const std::string& key);
std::string read_cart_value(const std::string& name,const std::string& key);
bool set_robot_value(const std::string& name,const std::string& key, const std::string& value);
bool set_cart_value(const std::string& name,const std::string& key, const std::string& value);
bool delete_mission_queue();
std::pair<std::string,std::string> ask_free_BWS(const bool& ask_ldb);
std::pair<std::string,std::string> ask_free_BCS(const bool& ask_ldb);
std::string read_assertLift_value(const std::string& cart_name);
std::string fetch_job();
bool request_ldb();
bool update_ldb();
int get_operation_time(const std::string& robot);
bool reset_station_blocker(const std::string& station);
bool update_job_monitor();
void update_robot_charge(const std::string& robot_name);
#endif