#ifndef ARRIVE_AT_STATION_H
#define ARRIVE_AT_STATION_H

/**
 * @brief The `arrive_at_station` class is a synchronous action node in the behavior tree.
 *
 * This node is responsible for sending a goal to the `arrive_at_station` action server and
 * waiting for the robot to reach the target station. It retrieves necessary information from
 * the blackboard and performs appropriate actions based on the current state of the robot
 * and the job type.
 */

class arrive_at_station : public BT::SyncActionNode
{
public:
  arrive_at_station(const std::string &name,
                    const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
      : BT::SyncActionNode(name, config), arg_param(arg_cp_params) {}

  static PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {

    actionlib::SimpleActionClient<chargepal_actions::ArriveAtStationAction> aas(
        "arrive_at_station", true);
    aas.waitForServer();
    chargepal_actions::ArriveAtStationGoal goal;

    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    job = masterBlackboard->get<std::string>("job_type");
    cart = masterBlackboard->get<std::string>("cart");
    robot = masterBlackboard->get<std::string>("robot");
    charging_type = masterBlackboard->get<std::string>("charging_type");
    std::transform(charging_type.begin(), charging_type.end(), charging_type.begin(),[](unsigned char c){ return std::toupper(c); });
    source_station = masterBlackboard->get<std::string>("source_station");
    target_station = masterBlackboard->get<std::string>("target_station");
    robot_location = read_robot_value(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, "robot_location");
    cart_on_robot = read_robot_value(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, "cart_on_robot");

    if (robot_location == target_station || robot_location == source_station)
    {
      goal.target_station = target_station;
    }

    // When the robot is not at the source station or target station
    else
    {
      goal.target_station = source_station;
    }

    // When goal station is BWS
    if (goal.target_station.find("BWS") != std::string::npos)
    {
      if (job == "BRING_CHARGER")
      {
        goal.target_station = goal.target_station + "_pick";
      }
    }
    else if (goal.target_station.find("BCS") != std::string::npos)
    {
      if (job == "BRING_CHARGER" || job == "STOW_CHARGER")
      {
        goal.target_station = goal.target_station + "_pick";
      }
    }
    else if (goal.target_station.find("ADS") != std::string::npos)
    {
      if (job == "RECHARGE_CHARGER" || job == "STOW_CHARGER")
      {
        goal.target_station = goal.target_station + "_" + charging_type + "_pick";
      }
      else{
        goal.target_station = goal.target_station + "_" + charging_type;
        }
    }
    aas_goal_string = std::string("arrive_at_station_") + goal.target_station;
    station_transition = robot_location + "_to_" + goal.target_station;
    masterBlackboard->set("current_aas_goal", goal.target_station);
    aas.sendGoal(goal);
    if (cart_on_robot != "none" && cart_on_robot != "")
    {
      tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", aas_goal_string}, {"robot_location", station_transition}}}}, {CART_TABLE, {cart_on_robot, {{"cart_location", station_transition}}}}};
    }
    else
    {
      tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", aas_goal_string}, {"robot_location", station_transition}}}}};
    }

    set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
    enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Performing " + aas_goal_string);
    bool aas_action = aas.waitForResult(ros::Duration(900.0));
    if (aas_action)
    {
      chargepal_actions::ArriveAtStationResult result = *aas.getResult();

      bool station_reached = result.station_reached;
      action_result = result.action_status;

      if (station_reached)
      {
        std::string current_station = result.current_station;

        if (cart_on_robot != "none" && cart_on_robot != "")
        {
          tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", aas_goal_string}, {"robot_location", current_station}}}}, {CART_TABLE, {cart_on_robot, {{"cart_location", current_station}}}}};
          set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
        }
        else
        {
          tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", aas_goal_string}, {"robot_location", current_station}}}}};
          set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
        }
        masterBlackboard->set("previous_robot_action", aas_goal_string);
        return BT::NodeStatus::SUCCESS;
      }

      enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), aas_goal_string + " status is " +
                                                                                 action_result);

      tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", aas_goal_string + "_" + action_result}}}}};
    }
    else
    {
      aas.cancelGoal();
      enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), aas_goal_string + " cannot be finished within a timeout of 900 seconds");
      tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", aas_goal_string + std::string("_ActionTimeout")}}}}};
    }
    set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
    masterBlackboard->set("failed_robot_action", aas_goal_string);
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string job, cart, robot, source_station, target_station, robot_location, cart_on_robot, action_result, aas_goal_string, station_transition, charging_type;
  std::vector<TableInfo> tables_values;
  std::map<std::string, std::any> arg_param;
};
#endif