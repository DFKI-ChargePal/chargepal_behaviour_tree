#ifndef MY_FUNCTIONS_H
#define MY_FUNCTIONS_H

#include "ros/ros.h"
#include <string> // Include any necessary header files

std::string read_robot_value(const std::string& name,const std::string& key);
std::string read_cart_value(const std::string& name,const std::string& key);
bool set_robot_value(const std::string& name,const std::string& key, const std::string& value);
bool set_cart_value(const std::string& name,const std::string& key, const std::string& value);
bool delete_mir_mission_queue();
std::string ask_free_BWS();
std::string ask_free_BCS();
std::string read_assertLift_value(const std::string& cart_name);
std::string fetch_job(const std::string& robot);
bool request_ldb();
bool update_ldb();
int get_operation_time(const std::string& robot);
#endif