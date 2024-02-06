#ifndef MY_FUNCTIONS_H
#define MY_FUNCTIONS_H

#include <iostream>
#include <chrono>
#include <ros/package.h>
#include <unordered_map>
#include <string>
#include <vector>
#include <SQLiteCpp/SQLiteCpp.h>
#include "ros/ros.h"
#include <string> 
#include<nlohmann/json.hpp>
#include <cctype>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "chargepal_actions/ArriveAtStationAction.h"
#include "chargepal_actions/ArriveAtHomeAction.h"
#include "chargepal_actions/PickUpChargerAction.h"
#include "chargepal_actions/PlaceChargerAction.h"
#include "chargepal_actions/DisconnectPlugFromCarAction.h"
#include "chargepal_actions/ConnectPlugToCarAction.h"
#include "chargepal_actions/PlugInAction.h"
#include "chargepal_actions/PlugOutAction.h"
#include "chargepal_actions/CallForHelpAction.h"

#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_cpp/exceptions.h"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/decorators/delay_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_observer.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/controls/reactive_fallback.h"

using namespace BT;
using namespace std::chrono_literals;
using namespace std::chrono;
using json = nlohmann::json;

enum class JobEnum {
    ONGOING,
    SUCCESS,
    FAILURE,
    RECOVERY
};

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
bool update_job_monitor(const std::string& job_type, std::string& job_status);
void update_robot_charge(const std::string& robot_name);
std::string enumToString(JobEnum value);

#endif