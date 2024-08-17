#ifndef ARRIVE_AT_HOME_H
#define ARRIVE_AT_HOME_H

/**
 * @brief The `go_home` class is a synchronous action node in the behavior tree that represents the behavior of the robot going home.
 *
 * It sends a goal to the `arrive_at_home` action server and waits for the result.
 * If the robot successfully reaches home, it updates the relevant information in the RDBC (Robot Database Copy) and returns `SUCCESS`.
 * If the robot fails to reach home or encounters an error, it updates the relevant information in the RDBC, sets the `failed_robot_action` parameter in the blackboard, and returns `FAILURE`.
 */

class go_home : public BT::SyncActionNode
{
public:
  go_home(const std::string &name, const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
      : BT::SyncActionNode(name, config), arg_param(arg_cp_params) {}

  static PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {

    actionlib::SimpleActionClient<chargepal_actions::ArriveAtHomeAction> aah(
        "arrive_at_home", true);
    aah.waitForServer();

    masterBlackboard = config().blackboard;
    robot = masterBlackboard->get<std::string>("robot");
    robot_location = read_robot_value(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, "robot_location");

    for (char c : robot)
    {
      if (std::isdigit(c))
      {
        robot_number = std::string(1, c);
        goal.target_station = "RBS_" + robot_number;
      }
    }
    station_transition = robot_location + "_to_" + goal.target_station;
    aah.sendGoal(goal);
    tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("go_home_") + goal.target_station}, {"robot_location", station_transition}}}}};
    set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
    enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Performing arrive_at_home to " + goal.target_station);
    update_gui_config("ongoing_action", "Performing arrive_at_home to " + goal.target_station);
    bool aah_action = aah.waitForResult(ros::Duration(900.0));
    if (aah_action)
    {
      result = *aah.getResult();
      bool home_reached = result.station_reached;
      if (home_reached)
      {
        std::string current_station = result.current_station;
        tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", std::string("go_home_") + goal.target_station}, {"robot_location", current_station}}}}};
        set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
        masterBlackboard->set("previous_robot_action", std::string("go_home_") + goal.target_station);
        return BT::NodeStatus::SUCCESS;
      }
      action_result = result.action_status;

      enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "arrive_at_home to" + goal.target_station + " status is " +
                                                                                 action_result);
      update_gui_config("ongoing_action", "arrive_at_home to" + goal.target_station + " status is " + action_result);
      tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", std::string("go_home_") + goal.target_station + "_" + action_result}}}}};
        }
    else
    {
      aah.cancelGoal();
      enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "go_home_" + goal.target_station + " cannot be finished within a timeout of 900 seconds");
      update_gui_config("ongoing_action", "go_home_" + goal.target_station + " cannot be finished within a timeout of 900 seconds");
      tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", std::string("go_home_") + goal.target_station + std::string("_ActionTimeout")}}}}};
    }
    set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
    masterBlackboard->set("failed_robot_action", std::string("go_home_") + goal.target_station);
    return BT::NodeStatus::FAILURE;
  }

private:
  BT::Blackboard::Ptr masterBlackboard;
  chargepal_actions::ArriveAtHomeGoal goal;
  chargepal_actions::ArriveAtHomeResult result;
  std::vector<TableInfo> tables_values;
  std::string robot, robot_location, station_transition, action_result, robot_number;
  std::map<std::string, std::any> arg_param;
};
#endif