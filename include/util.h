#ifndef MY_FUNCTIONS_H
#define MY_FUNCTIONS_H

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/controls/reactive_fallback.h"
#include "behaviortree_cpp/decorators/delay_node.h"
#include "behaviortree_cpp/exceptions.h"
#include "behaviortree_cpp/loggers/bt_observer.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/tree_node.h"
#include "chargepal_actions/ArriveAtHomeAction.h"
#include "chargepal_actions/ArriveAtStationAction.h"
#include "chargepal_actions/CallForHelpAction.h"
#include "chargepal_actions/ConnectPlugToCarAction.h"
#include "chargepal_actions/DisconnectPlugFromCarAction.h"
#include "chargepal_actions/PickUpChargerAction.h"
#include "chargepal_actions/PlaceChargerAction.h"
#include "chargepal_actions/PlugInAction.h"
#include "chargepal_actions/PlugOutAction.h"
#include "ros/ros.h"
#include <SQLiteCpp/SQLiteCpp.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cctype>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <map>
#include <nlohmann/json.hpp>
#include <ros/package.h>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <chargepal_services/askFreeBCS.h>
#include <chargepal_services/askFreeBWS.h>
#include <chargepal_services/askOperationTime.h>
#include <chargepal_services/assertLiftValue.h>
#include <chargepal_services/deleteMirMission.h>
#include <chargepal_services/fetchJob.h>
#include <chargepal_services/isReadyToPlugin.h>
#include <chargepal_services/pushToLDB.h>
#include <chargepal_services/readRobotCharge.h>
#include <chargepal_services/resetStationBlocker.h>
#include <chargepal_services/updateJobMonitor.h>
#include <chargepal_services/updateRdbCopy.h>
#include <chargepal_services/verifyRdbSync.h>
using namespace BT;
using namespace std::chrono_literals;
using namespace std::chrono;
using json = nlohmann::json;

enum class JobEnum { ONGOING = 0, SUCCESS = 1, RECOVERY = 2, FAILURE = 3 };

std::string read_robot_value(const std::string &name, const std::string &key);
std::string read_cart_value(const std::string &name, const std::string &key);
bool set_robot_value(const std::string &name, const std::string &key,
                     const std::string &value);
bool set_cart_value(const std::string &name, const std::string &key,
                    const std::string &value);
std::pair<std::string, std::string> ask_free_BWS(const bool &ask_ldb);
std::pair<std::string, std::string> ask_free_BCS(const bool &ask_ldb);
std::string read_assertLift_value(const std::string &cart_name);
std::string fetch_job();
int get_operation_time(const std::string &robot);
bool reset_station_blocker(const std::string &station);
bool update_job_monitor(const std::string &job_type, std::string &job_status);
void update_robot_charge(const std::string &robot_name);
std::string enumToString(const int value);
bool check_ready_to_plugin(const std::string &station_name);
bool update_rdb_copy();
void enter_log_file(const std::string content);
template <typename T> std::string convertToString(const T &value) {
  if constexpr (std::is_convertible_v<T, std::string>) {
    return value;
  } else {
    return std::to_string(value);
  }
}

#endif