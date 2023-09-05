#ifndef MY_FUNCTIONS_H
#define MY_FUNCTIONS_H

#include "ros/ros.h"
#include <string> // Include any necessary header files

std::string read_robot_value(const std::string& name,const std::string& key);
std::string read_cart_value(const std::string& name,const std::string& key);
bool set_robot_value(const std::string& name,const std::string& key, const std::string& value);
bool set_cart_value(const std::string& name,const std::string& key, const std::string& value);

#endif