#include "util.h"
#include "robot/arrive_at_station.h"
#include "robot/arrive_home.h"
#include "robot/pickup_cart.h"
#include "robot/place_cart.h"
#include "robot/plugin.h"
#include "robot/plugout.h"
#include "robot/recovery_arrive_at_station.h"
#include "battery/einschalten.h"
#include "battery/ausschalten.h"
#include "battery/end_ladeprozess.h"
#include "battery/start_ladeprozess.h"
#include "battery/idle.h"
#include "battery/modus_start.h"
#include "battery/bat_only.h"

class isBRING_CHARGER : public BT::ConditionNode
{
public:
  isBRING_CHARGER(const std::string &name, const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
      : BT::ConditionNode(name, config), arg_param(arg_cp_params) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override
  {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string job = masterBlackboard->get<std::string>("job_type");
    std::string robot = masterBlackboard->get<std::string>("robot");
    std::vector<TableInfo> tables_values;

    if (job == "BRING_CHARGER")
    {
      tables_values = {{ROBOT_TABLE, {robot, {{"current_job", std::string("BRING_CHARGER")}}}}};
      set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
      update_gui_config("ongoing_job", "BRING_CHARGER");
      return NodeStatus::SUCCESS;
    }
    else
    {
      return NodeStatus::FAILURE;
    }
  }

private:
  std::map<std::string, std::any> arg_param;
};

class isRECHARGE_CHARGER : public BT::ConditionNode
{
public:
  isRECHARGE_CHARGER(const std::string &name,
                     const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
      : BT::ConditionNode(name, config), arg_param(arg_cp_params) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override
  {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string robot = masterBlackboard->get<std::string>("robot");
    std::string job = masterBlackboard->get<std::string>("job_type");
    std::vector<TableInfo> tables_values;
    if (job == "RECHARGE_CHARGER")
    {
      update_gui_config("ongoing_job", "RECHARGE_CHARGER");
      tables_values = {{ROBOT_TABLE, {robot, {{"current_job", std::string("RECHARGE_CHARGER")}}}}};
      set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
      return NodeStatus::SUCCESS;
    }
    else
    {
      return NodeStatus::FAILURE;
    }
  }

private:
  std::map<std::string, std::any> arg_param;
};

class isRECHARGE_SELF : public BT::ConditionNode
{
public:
  isRECHARGE_SELF(const std::string &name, const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
      : BT::ConditionNode(name, config), arg_param(arg_cp_params) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override
  {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string robot = masterBlackboard->get<std::string>("robot");
    std::string job = masterBlackboard->get<std::string>("job_type");
    std::vector<TableInfo> tables_values;
    if (job == "RECHARGE_SELF")
    {
      update_gui_config("ongoing_job", "RECHARGE_SELF");
      tables_values = {{ROBOT_TABLE, {robot, {{"current_job", std::string("RECHARGE_SELF")}}}}};
      set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
      return NodeStatus::SUCCESS;
    }
    else
    {
      return NodeStatus::FAILURE;
    }
  }

private:
  std::map<std::string, std::any> arg_param;
};

class isSTOW_CHARGER : public BT::ConditionNode
{
public:
  isSTOW_CHARGER(const std::string &name, const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
      : BT::ConditionNode(name, config), arg_param(arg_cp_params) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override
  {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string robot = masterBlackboard->get<std::string>("robot");
    std::string job = masterBlackboard->get<std::string>("job_type");
    std::vector<TableInfo> tables_values;
    if (job == "STOW_CHARGER")
    {
      tables_values = {{ROBOT_TABLE, {robot, {{"current_job", std::string("STOW_CHARGER")}}}}};
      update_gui_config("ongoing_job", "STOW_CHARGER");
      set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
      return NodeStatus::SUCCESS;
    }
    else
    {
      return NodeStatus::FAILURE;
    }
  }

private:
  std::map<std::string, std::any> arg_param;
};

class isRobotAtADSorBCSorBWS : public BT::ConditionNode
{
public:
  isRobotAtADSorBCSorBWS(const std::string &name,
                         const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
      : BT::ConditionNode(name, config), arg_param(arg_cp_params) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override
  {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string robot = masterBlackboard->get<std::string>("robot");
    std::string location = read_robot_value(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, "robot_location");
    if ((location.find("ADS") != std::string::npos ||
         location.find("BCS") != std::string::npos ||
         location.find("BWS") != std::string::npos) &&
        location.find("pick") == std::string::npos)
    {
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return NodeStatus::FAILURE;
    }
  }

private:
  std::map<std::string, std::any> arg_param;
};

class notPickGoalLocation : public BT::ConditionNode
{
public:
  notPickGoalLocation(const std::string &name, const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
      : BT::ConditionNode(name, config), arg_param(arg_cp_params) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override
  {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string current_aas_goal = masterBlackboard->get<std::string>("current_aas_goal");

    if (current_aas_goal.find("pick") != std::string::npos)
    {
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return NodeStatus::FAILURE;
    }
  }

private:
  std::map<std::string, std::any> arg_param;
};

class isRobotAtRBS : public BT::ConditionNode
{
public:
  isRobotAtRBS(const std::string &name, const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
      : BT::ConditionNode(name, config), arg_param(arg_cp_params) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override
  {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string robot = masterBlackboard->get<std::string>("robot");

    std::string location = read_robot_value(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, "robot_location");
    if (location.find("RBS") != std::string::npos ||
        (location.find("pick") != std::string::npos &&
         location.find("BWS") != std::string::npos) ||
        (location.find("pick") != std::string::npos &&
         location.find("ADS") != std::string::npos))
    {

      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return NodeStatus::FAILURE;
    }
  }

private:
  std::map<std::string, std::any> arg_param;
};

class isRobotOrGoalAtBWS : public BT::ConditionNode
{
public:
  isRobotOrGoalAtBWS(const std::string &name, const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
      : BT::ConditionNode(name, config), arg_param(arg_cp_params) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override
  {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string robot = masterBlackboard->get<std::string>("robot");
    // std::string current_aas_goal = masterBlackboard->get<std::string>("current_aas_goal");
    std::string location = read_robot_value(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, "robot_location");

    size_t station_transition = location.find("_to_");
    //"_to_" is found
    if (station_transition != std::string::npos && location.find("BWS", station_transition + std::string("_to_").length()) != std::string::npos)
    {
      return BT::NodeStatus::SUCCESS;
    }

    else if (location.find("BWS") != std::string::npos)
    {
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return NodeStatus::FAILURE;
    }

    /* if (location.find("BWS") != std::string::npos || current_aas_goal.find("BWS") != std::string::npos)
    {
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return NodeStatus::FAILURE;
    } */
  }

private:
  std::map<std::string, std::any>
      arg_param;
};

class isRobotOrGoalAtBCS : public BT::ConditionNode
{
public:
  isRobotOrGoalAtBCS(const std::string &name, const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
      : BT::ConditionNode(name, config), arg_param(arg_cp_params) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override
  {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string robot = masterBlackboard->get<std::string>("robot");
    // std::string current_aas_goal = masterBlackboard->get<std::string>("current_aas_goal");
    std::string location = read_robot_value(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, "robot_location");
    size_t station_transition = location.find("_to_");

    //"_to_" is found
    if (station_transition != std::string::npos && location.find("BCS", station_transition + std::string("_to_").length()) != std::string::npos)
    {
      return BT::NodeStatus::SUCCESS;
    }
    else if (location.find("BCS") != std::string::npos)
    {
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return NodeStatus::FAILURE;
    }
  }

private:
  std::map<std::string, std::any> arg_param;
};

class isRobotOrGoalAtADS : public BT::ConditionNode
{
public:
  isRobotOrGoalAtADS(const std::string &name, const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
      : BT::ConditionNode(name, config), arg_param(arg_cp_params) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override
  {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string robot = masterBlackboard->get<std::string>("robot");
    // std::string current_aas_goal = masterBlackboard->get<std::string>("current_aas_goal");
    std::string location = read_robot_value(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, "robot_location");

    size_t station_transition = location.find("_to_");
    //"_to_" is found
    if (station_transition != std::string::npos && location.find("ADS", station_transition + std::string("_to_").length()) != std::string::npos)
    {
      return BT::NodeStatus::SUCCESS;
    }
    else if (location.find("ADS") != std::string::npos)
    {
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return NodeStatus::FAILURE;
    }
  }

private:
  std::map<std::string, std::any> arg_param;
};

class isArmFree : public BT::ConditionNode
{
public:
  isArmFree(const std::string &name, const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
      : BT::ConditionNode(name, config), arg_param(arg_cp_params) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override
  {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string robot = masterBlackboard->get<std::string>("robot");
    std::cout << "Enter y/n if the arm is free  ";
    char userInput;
    std::cin >> userInput;
    if (userInput == 'y')
    {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

private:
  std::map<std::string, std::any> arg_param;
};

class isSameBattery : public BT::ConditionNode
{
public:
  isSameBattery(const std::string &name, const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
      : BT::ConditionNode(name, config), arg_param(arg_cp_params) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override
  {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string job_cart = masterBlackboard->get<std::string>("cart");
    std::string robot = masterBlackboard->get<std::string>("robot");
    std::string cart_on_robot = read_robot_value(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, "cart_on_robot");

    if (job_cart == cart_on_robot)
    {
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return NodeStatus::FAILURE;
    }
  }

private:
  std::map<std::string, std::any> arg_param;
};

class isDifferentBattery : public BT::ConditionNode
{
public:
  isDifferentBattery(const std::string &name,
                     const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
      : BT::ConditionNode(name, config), arg_param(arg_cp_params) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override
  {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string job_cart = masterBlackboard->get<std::string>("cart");
    std::string robot = masterBlackboard->get<std::string>("robot");
    std::string cart_on_robot = read_robot_value(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, "cart_on_robot");

    if (job_cart != cart_on_robot)
    {
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return NodeStatus::FAILURE;
    }
  }

private:
  std::map<std::string, std::any> arg_param;
};

class isbattery_ADS_BCS : public BT::ConditionNode
{
public:
  isbattery_ADS_BCS(const std::string &name,
                    const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
      : BT::ConditionNode(name, config), arg_param(arg_cp_params) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override
  {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string source_station =
        masterBlackboard->get<std::string>("source_station");
    if (source_station.find("ADS") != std::string::npos ||
        source_station.find("BCS") != std::string::npos)
    {
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return NodeStatus::FAILURE;
    }
  }

private:
  std::map<std::string, std::any> arg_param;
};

class isbattery_BWS : public BT::ConditionNode
{
public:
  isbattery_BWS(const std::string &name, const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
      : BT::ConditionNode(name, config), arg_param(arg_cp_params) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override
  {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string source_station =
        masterBlackboard->get<std::string>("source_station");
    if (source_station.find("BWS") != std::string::npos)
    {
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return NodeStatus::FAILURE;
    }
  }

private:
  std::map<std::string, std::any> arg_param;
};

class isAssertLiftDown : public BT::ConditionNode
{
public:
  isAssertLiftDown(const std::string &name, const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
      : BT::ConditionNode(name, config), arg_param(arg_cp_params) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override
  {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string robot = masterBlackboard->get<std::string>("robot");

    std::string assertLift = read_assertLift_value(robot);

    if (assertLift == "down")
    {
      enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Assert lift is : " + assertLift);
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Assert lift is : " + assertLift);
      return NodeStatus::FAILURE;
    }
  }

private:
  std::map<std::string, std::any> arg_param;
};

class isCartPlaced : public BT::ConditionNode
{
public:
  isCartPlaced(const std::string &name, const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
      : BT::ConditionNode(name, config), arg_param(arg_cp_params) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override
  {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string cart = masterBlackboard->get<std::string>("cart");
    std::string robot = masterBlackboard->get<std::string>("robot");

    std::string cart_on_robot = read_robot_value(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, "cart_on_robot");
    std::string robot_on_cart = read_cart_value(std::any_cast<std::string>(arg_param["rdbc_path"]), cart, "robot_on_cart");

    if (cart_on_robot == robot_on_cart && cart_on_robot == "none")
    {

      return BT::NodeStatus::SUCCESS;
    }
    else
    {

      return NodeStatus::FAILURE;
    }
  }

private:
  std::map<std::string, std::any> arg_param;
};

class call_for_help : public BT::SyncActionNode
{
public:
  call_for_help(const std::string &name, const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
      : BT::SyncActionNode(name, config), arg_param(arg_cp_params) {}

  static PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    robot = masterBlackboard->get<std::string>("robot");
    job_type = masterBlackboard->get<std::string>("job_type");
    source_station = masterBlackboard->get<std::string>("source_station");
    target_station = masterBlackboard->get<std::string>("target_station");
    previous_robot_action = masterBlackboard->get<std::string>("previous_robot_action");
    previous_battery_action = masterBlackboard->get<std::string>("previous_battery_action");
    failed_robot_action = masterBlackboard->get<std::string>("failed_robot_action");
    failed_battery_action = masterBlackboard->get<std::string>("failed_battery_action");

    if (failed_robot_action.find("arrive_at_station") != std::string::npos) {
      update_gui_config("error_count_arrive_at_station", "");
    } else if (failed_robot_action.find("go_home") != std::string::npos) {
      update_gui_config("error_count_go_home", "");
    } else if (failed_robot_action.find("place_charger") != std::string::npos) {
      update_gui_config("error_count_place_cart", "");
    } else if (failed_robot_action.find("pickup_charger") != std::string::npos) {
      update_gui_config("error_count_pickup_cart", "");
    } else if (failed_robot_action.find("plugin") != std::string::npos) {
      update_gui_config("error_count_plugin_ads", "");
    } else if (failed_robot_action.find("plugout") != std::string::npos) {
      update_gui_config("error_count_plugout_ads", "");
    }

    call_context = job_type + "with source_station:" + source_station + " and target_station:" + target_station + " reached failure with failed_robot_action:" + failed_robot_action + " and failed_battery_action:" + failed_battery_action + ". The previous successfull actions were previous_robot_action:" + previous_robot_action + " and previous_battery_action:" + previous_battery_action;
    enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Calling for help");
    //calling_help(robot, call_context);
    return BT::NodeStatus::SUCCESS;
  }

private:
  BT::Blackboard::Ptr masterBlackboard;
  std::string robot, job_type, target_station, source_station, previous_robot_action, previous_battery_action, failed_robot_action, failed_battery_action, call_context;
  std::map<std::string, std::any> arg_param;
};

class error_count : public BT::SyncActionNode
{
public:
  error_count(const std::string &name, const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
      : BT::SyncActionNode(name, config), arg_param(arg_cp_params) {}

  static PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::vector<TableInfo> tables_values;
    std::string robot = masterBlackboard->get<std::string>("robot");
    std::string error_count = read_robot_value(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, "error_count");
    int ec = std::stoi(error_count);
    ec = ec + 1;
    tables_values = {{ROBOT_TABLE, {robot, {{"error_count", std::to_string(ec)}}}}};
    set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);

    return BT::NodeStatus::SUCCESS;
  }

private:
  std::map<std::string, std::any> arg_param;
};

class time_sleep : public BT::SyncActionNode
{
public:
  time_sleep(const std::string &name, const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
      : BT::SyncActionNode(name, config), arg_param(arg_cp_params) {}

  static PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    ros::Duration(10).sleep();
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::map<std::string, std::any> arg_param;
};
class sleep_until_charged : public BT::StatefulActionNode
{
public:
  sleep_until_charged(const std::string &name,
                      const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
      : BT::StatefulActionNode(name, config), arg_param(arg_cp_params) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus onStart() override
  {

    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string robot = masterBlackboard->get<std::string>("robot");
    std::string cart = read_robot_value(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, "cart_on_robot");
    int msec = get_operation_time(cart);

    if (msec <= 0)
    {
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      // once the deadline is reached, we will return SUCCESS.
      deadline_ = system_clock::now() + milliseconds(msec);
      return BT::NodeStatus::RUNNING;
    }
  }

  /// method invoked by an action in the RUNNING state.
  virtual NodeStatus onRunning() override
  {
    if (system_clock::now() >= deadline_)
    {
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::RUNNING;
    }
  }

  void onHalted() override
  {
    std::cout << "SleepNode interrupted" << std::endl;
  }

private:
  system_clock::time_point deadline_;
  std::map<std::string, std::any> arg_param;
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "chargepal_bt");
  ros::NodeHandle nh;
  std::vector<TableInfo> tables_values;
  bool job_server_update;
  std::string job_status;
  std::string job_input;
  json job_requested;
  std::map<std::string, std::any> cp_params = set_ros_params(nh);

  std::ofstream logFile(std::any_cast<std::string>(cp_params["log_file_path"]));

  BehaviorTreeFactory factory;
  BT::Blackboard::Ptr masterBlackboard = BT::Blackboard::create();

  bool recovery_enabled = std::any_cast<bool>(cp_params["recovery_enabled"]);

  factory.registerNodeType<isBRING_CHARGER>("isBRING_CHARGER", cp_params);
  factory.registerNodeType<isRECHARGE_CHARGER>("isRECHARGE_CHARGER", cp_params);
  factory.registerNodeType<isRECHARGE_SELF>("isRECHARGE_SELF", cp_params);
  factory.registerNodeType<isSTOW_CHARGER>("isSTOW_CHARGER", cp_params);

  factory.registerNodeType<isRobotAtADSorBCSorBWS>("isRobotAtADSorBCSorBWS", cp_params);
  factory.registerNodeType<isRobotAtRBS>("isRobotAtRBS", cp_params);
  factory.registerNodeType<isRobotOrGoalAtBWS>("isRobotOrGoalAtBWS", cp_params);
  factory.registerNodeType<isRobotOrGoalAtBCS>("isRobotOrGoalAtBCS", cp_params);
  factory.registerNodeType<isRobotOrGoalAtADS>("isRobotOrGoalAtADS", cp_params);

  factory.registerNodeType<isArmFree>("isArmFree", cp_params);
  factory.registerNodeType<isSameBattery>("isSameBattery", cp_params);
  factory.registerNodeType<isDifferentBattery>("isDifferentBattery", cp_params);
  factory.registerNodeType<isbattery_ADS_BCS>("isbattery_ADS_BCS", cp_params);
  factory.registerNodeType<isbattery_BWS>("isbattery_BWS", cp_params);
  factory.registerNodeType<isAssertLiftDown>("isAssertLiftDown", cp_params);
  factory.registerNodeType<isCartPlaced>("isCartPlaced", cp_params);
  factory.registerNodeType<notPickGoalLocation>("notPickGoalLocation", cp_params);

  factory.registerNodeType<arrive_at_station>("arrive_at_station", cp_params);
  factory.registerNodeType<go_home>("go_home", cp_params);
  factory.registerNodeType<call_for_help>("call_for_help", cp_params);
  factory.registerNodeType<drop_cart>("drop_cart", cp_params);
  factory.registerNodeType<pickup_cart>("pickup_cart", cp_params);
  factory.registerNodeType<plugin_ADS>("plugin_ADS", cp_params);
  factory.registerNodeType<plugin_BCS>("plugin_BCS", cp_params);
  factory.registerNodeType<plugout_ADS>("plugout_ADS", cp_params);
  factory.registerNodeType<plugout_BCS>("plugout_BCS", cp_params);
  factory.registerNodeType<recovery_arrive_BWS>("recovery_arrive_BWS", cp_params);
  factory.registerNodeType<recovery_arrive_BCS>("recovery_arrive_BCS", cp_params);
  factory.registerNodeType<sleep_until_charged>("sleep_until_charged", cp_params);
  factory.registerNodeType<error_count>("error_count", cp_params);
  factory.registerNodeType<time_sleep>("time_sleep", cp_params);
  factory.registerNodeType<battery_wakeup>("battery_wakeup", cp_params);
  factory.registerNodeType<battery_ladeprozess_ende>("battery_ladeprozess_ende", cp_params);
  factory.registerNodeType<battery_ladeprozess_start>("battery_ladeprozess_start", cp_params);
  factory.registerNodeType<battery_mode_req_idle>("battery_mode_req_idle", cp_params);
  factory.registerNodeType<battery_mode_req_charge>("battery_mode_req_charge", cp_params);
  factory.registerNodeType<battery_mode_req_standby>("battery_mode_req_standby", cp_params);
  factory.registerNodeType<battery_mode_bat_only>("battery_mode_bat_only", cp_params);

  std::string package_folder =
      ros::package::getPath("chargepal_behaviour_tree");

  factory.registerBehaviorTreeFromFile(package_folder + "/xml/Main.xml");
  factory.registerBehaviorTreeFromFile(package_folder + "/xml/Jobs.xml");
  factory.registerBehaviorTreeFromFile(package_folder + "/xml/Actionsets.xml");
  factory.registerBehaviorTreeFromFile(package_folder + "/xml/Actions.xml");
  factory.registerBehaviorTreeFromFile(package_folder + "/xml/Recovery.xml");

  auto mainTree = factory.createTree("main", masterBlackboard);
  BT::Groot2Publisher publisher(mainTree);

  while (ros::ok() && job_requested.empty())
  {

    // FETCH JOB
    try
    {
      job_input = fetch_job();

      if (!job_input.empty())
      {
        job_requested = json::parse(job_input);
        
        // IF FETCHED JOB IS NOT EMPTY
        if (!job_requested.empty())
        {
          // CLEAR LOG FILE
          logFile.open(std::any_cast<std::string>(cp_params["log_file_path"]), std::ofstream::out | std::ofstream::trunc);
          logFile.close();
          enter_log_file(std::any_cast<std::string>(cp_params["log_file_path"]), "Received job : " + job_input);

          int job_id = job_requested["job_id"];
          std::string job_type = job_requested["job_type"];
          std::string robot = job_requested["robot_name"];
          std::string cart = job_requested["cart"];
          std::string source_station = job_requested["source_station"];
          std::string target_station = job_requested["target_station"];
          std::string charging_type = job_requested["charging_type"];

          try
          {
            if (robot != std::any_cast<std::string>(cp_params["robot"]))
            {
              enter_log_file(std::any_cast<std::string>(cp_params["log_file_path"]), "Robot Name in config and Robot Name from server do not match!");
              break;
            }
          }
          catch (const std::bad_any_cast &e)
          {
            std::cerr << "Bad any cast: " << e.what() << std::endl;
          }

          // VEFRIFY SERVER CONNECTION WITH ROBOT DATABASE
          bool rdb_sync = verify_rdb_ldb_connection(std::any_cast<int>(cp_params["server_timeout"]));
          if (rdb_sync)
          {
            // UPDATE RDBC TABLE WITH JOB'S ROBOT AND CART
            std::map<std::string, std::vector<std::string>> target_components = identify_job_components(cp_params, job_requested);
            pull_rdb_to_rdbc(target_components);

            // SET BT BLACKBOARD
            masterBlackboard->set("recovery_enabled", recovery_enabled);
            masterBlackboard->set("job_type", job_type);
            masterBlackboard->set("robot", robot);
            masterBlackboard->set("cart", cart);
            masterBlackboard->set("source_station", source_station);
            masterBlackboard->set("target_station", target_station);
            masterBlackboard->set("charging_type", charging_type);
            masterBlackboard->set("job_status", 0);
            masterBlackboard->set("current_aas_goal", "");
            masterBlackboard->set("failed_robot_action", "");
            masterBlackboard->set("previous_robot_action", "");
            masterBlackboard->set("previous_battery_action", "");
            masterBlackboard->set("failed_battery_action", "");

            ROS_INFO("Performing: %s", job_type.c_str());
            job_status = enumToString(masterBlackboard->get<int>("job_status"));

            tables_values = {{ROBOT_TABLE, {std::any_cast<std::string>(cp_params["robot"]), {{"current_job_id", job_id}, {"availability", false}, {"job_status", job_status}}}}};
            set_rdbc_values(std::any_cast<std::string>(cp_params["rdbc_path"]), std::any_cast<std::string>(cp_params["robot"]), tables_values);
            // update_job_monitor(std::any_cast<int>(cp_params["server_timeout"]), std::any_cast<std::string>(cp_params["robot"]), job_type, job_status);

            mainTree.tickWhileRunning();
            BT::NodeStatus status = mainTree.rootNode()->status();

            if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::IDLE ||
                status == BT::NodeStatus::FAILURE)
            {
              job_status = enumToString(masterBlackboard->get<int>("job_status"));
              tables_values = {{ROBOT_TABLE, {std::any_cast<std::string>(cp_params["robot"]), {{"availability", true}, {"job_status", job_status}, {"current_job_id", std::string("none")}}}}};
              set_rdbc_values(std::any_cast<std::string>(cp_params["rdbc_path"]), std::any_cast<std::string>(cp_params["robot"]), tables_values);
              enter_log_file(std::any_cast<std::string>(cp_params["log_file_path"]), "Updating job status as: " + job_status);
              update_gui_config("ongoing_action", "Job completed");
              job_server_update = update_job_monitor(std::any_cast<int>(cp_params["server_timeout"]), std::any_cast<std::string>(cp_params["robot"]), job_type, job_status);
              enter_log_file(std::any_cast<std::string>(cp_params["log_file_path"]), "The job update to server is  :" + job_server_update);
              job_requested.clear();
              std::cout << "................................................................" << std::endl;
              if (!job_server_update)
              {
                break;
              }
              if (job_status == "failure")
              {
                break;
              }
            }
          }

          // IF SERVER IS NOT CONNECTED AFTER JOB FETCH
          else
          {
            enter_log_file(std::any_cast<std::string>(cp_params["log_file_path"]), "No server connection to the robot after job fetch.");
            calling_help(robot, "No server connection after job fetch.");
            break;
            // job_server_update = update_job_monitor(std::any_cast<int>(cp_params["server_timeout"]), std::any_cast<std::string>(cp_params["robot"]), job_type, "Failure");
            // enter_log_file(std::any_cast<std::string>(cp_params["log_file_path"]), "The job update received is  :" + job_server_update);
            // job_requested.clear();
            // if (!job_server_update)
            //{
            //   break;
            // }
          }
        }
      }
      else
      {
        std::cout << "No job received" << std::endl;
        tables_values = {{ROBOT_TABLE, {std::any_cast<std::string>(cp_params["robot"]), {{"robot_charge", ""}}}}};
        set_rdbc_values(std::any_cast<std::string>(cp_params["rdbc_path"]), std::any_cast<std::string>(cp_params["robot"]), tables_values);
        ros::Duration(1.0).sleep();
      }
    }
    catch (const nlohmann::json::exception &e)
    {
      std::cerr << "Error parsing JSON: " << e.what() << std::endl;
    }
  }
  /* if (!job_server_update)
  {
    enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Failed updating job status as: " + job_status);
    actionlib::SimpleActionClient<chargepal_actions::CallForHelpAction>
        cfh("call_for_help", true);
    chargepal_actions::CallForHelpGoal goal;
    cfh.waitForServer();
    cfh.sendGoal(goal);
    cfh.waitForResult(ros::Duration(900.0));
  } */

  ros::spinOnce();

  // Exit the ROS node
  ros::shutdown();
  return 0;
}