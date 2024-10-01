#ifndef UTIL_H
#define UTIL_H

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
#include "chargepal_actions/PickUpCartAction.h"
#include "chargepal_actions/PlaceCartAction.h"
#include "chargepal_actions/PlugInAdsAcAction.h"
#include "chargepal_actions/PlugInBcsAcAction.h"
#include "chargepal_actions/PlugOutAdsAcAction.h"
#include "chargepal_actions/PlugOutBcsAcAction.h"
#include "chargepal_actions/BatteryCommunicationAction.h"
#include "chargepal_actions/PullRdbToRdbcAction.h"
#include "chargepal_actions/PushRdbcToLdbAction.h"

#include <chargepal_services/askFreeBCS.h>
#include <chargepal_services/askFreeBWS.h>
#include <chargepal_services/askOperationTime.h>
#include <chargepal_services/assertLiftValue.h>
#include <chargepal_services/deleteMirMission.h>
#include <chargepal_services/fetchJob.h>
#include <chargepal_services/readRobotCharge.h>
#include <chargepal_services/resetIoForCart.h>
#include <chargepal_services/resetStationBlocker.h>
#include <chargepal_services/updateJobMonitor.h>

#include "ros/ros.h"
#include <SQLiteCpp/SQLiteCpp.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <map>
#include <nlohmann/json.hpp>
#include <ros/package.h>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <variant>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <optional>
#include <boost/shared_ptr.hpp>
#include <std_msgs/String.h>

using namespace BT;
using namespace std::chrono_literals;
using namespace std::chrono;
using json = nlohmann::json;
using KeyValue = std::pair<std::string, std::variant<int, std::string>>;
using SetColumnValue = std::variant<std::optional<int>, std::optional<bool>, std::optional<std::string>, std::nullptr_t>;

enum class JobEnum
{
  Ongoing = 0,
  Success = 1,
  Recovery = 2,
  Failure = 3
};

struct ColumnValue
{
  std::string column_name;
  std::variant<int, bool, std::string> value;
};
struct RowInfo
{
  std::string row_name;
  std::vector<ColumnValue> column_values;
};
struct TableInfo
{
  std::string table_name;
  RowInfo row;
};

const std::string ROBOT_TABLE = "robot_info";
const std::string CART_TABLE = "cart_info";
const std::string BATTERY_ACTIONS_TABLE = "battery_action_info";

extern bool recovery_enabled;
extern float server_timeout;
extern std::string rdbc_path;
extern std::string rdb_path;
extern std::string log_file_path;
extern std::string robot;

std::pair<std::string, std::string> ask_free_BWS(const bool &ask_ldb);
std::pair<std::string, std::string> ask_free_BCS(const bool &ask_ldb);
std::string read_robot_value(const std::string db_path, const std::string &name, const std::string &key);
std::string read_cart_value(const std::string db_path, const std::string &name, const std::string &key);
std::string read_assertLift_value(const std::string &cart_name);
std::string fetch_job();
std::string current_timestamp();
std::string enumToString(const int value);

void update_robot_charge(const std::string &rdbc_path, const std::string &robot_name);
void enter_log_file(const std::string file_path, const std::string content);
void update_gui_config(const std::string key, std::string value);
void calling_help(const std::string &robot_name, const std::string &text);

bool verify_rdb_ldb_connection(const int &server_timeout);
bool reset_station_blocker(const std::string &station);
bool update_job_monitor(const int &server_timeout, const std::string &robot_name,
                        const std::string &job_type, const std::string &job_status);
bool check_ready_to_plugin(const std::string &station_name);

bool recover_cart(const std::string action_name);

bool pull_rdb_to_rdbc(std::map<std::string, std::vector<std::string>> &table_rows);
std::map<std::string, std::vector<std::string>> identify_job_components(std::map<std::string, std::any> cp_params, const json &job_cart);
bool set_rdbc_values(const std::string &rdbc_path, const std::string &robot_name, const std::vector<TableInfo> &tables);
std::map<std::string, std::any> set_ros_params(const ros::NodeHandle &nh);

int get_operation_time(const std::string &robot);

template <typename T>
std::string convertToString(const T &value)
{
  return value;
};
#endif