#ifndef PLACE_CART_H
#define PLACE_CART_H

/**
 * @brief The `drop_cart` class is a synchronous action node in the behavior tree.
 * It represents the action of placing a cart.
 */
class drop_cart : public BT::SyncActionNode
{
public:
  drop_cart(const std::string &name, const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
      : BT::SyncActionNode(name, config), arg_param(arg_cp_params) {}

  static PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {

    actionlib::SimpleActionClient<chargepal_actions::PlaceCartAction> plc(
        "place_cart", true);
    plc.waitForServer();

    masterBlackboard = config().blackboard;
    robot = masterBlackboard->get<std::string>("robot");
    robot_location = read_robot_value(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, "robot_location");
    cart_on_robot = read_robot_value(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, "cart_on_robot");
    goal.cart_name = cart_on_robot;

    plc.sendGoal(goal);

    tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("place_cart_") + goal.cart_name}}}}};
    set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
    enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Performing place_cart of " + goal.cart_name);
    update_gui_config("ongoing_action", "Performing place_cart of " + goal.cart_name);
    bool plc_action = plc.waitForResult(ros::Duration(900.0));
    if (plc_action)
    {
      result = *plc.getResult();
      bool cart_placed = result.cart_placed;
      if (cart_placed)
      {
        tables_values = {{ROBOT_TABLE, {robot, {{"cart_on_robot", std::string("none")}, {"ongoing_action", std::string("none")}, {"previous_action", std::string("place_cart_") + goal.cart_name}}}}, {CART_TABLE, {cart_on_robot, {{"robot_on_cart", std::string("none")}, {"cart_location", robot_location}}}}};
        set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
        masterBlackboard->set("previous_robot_action", "place_cart_" + goal.cart_name);
        return BT::NodeStatus::SUCCESS;
      }
      action_result = result.action_status;

      enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "place_cart of" + goal.cart_name + " status is " +
                                                                                 action_result);
      update_gui_config("ongoing_action", "place_cart of" + goal.cart_name + " status is " + action_result);
      tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", std::string("place_cart_") + goal.cart_name + "_" + action_result}}}}};
    }
    else
    {
      plc.cancelGoal();
      enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "place_cart of" + goal.cart_name + " cannot be finished within a timeout of 900 seconds");
      update_gui_config("ongoing_action", "place_cart of" + goal.cart_name + " cannot be finished within a timeout of 900 seconds");
      tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", std::string("place_cart_") + goal.cart_name + std::string("_ActionTimeout")}}}}};
    }
    // RESET IO PINS OF THE ROBOT FOR RETRYING THE PLACE_CART ACTION AGAIN
    recover_status = recover_cart("place_cart");
    if (!recover_status)
    {
      enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "place_cart of" + goal.cart_name +
                                                                                 " IO recover status is False");
      update_gui_config("ongoing_action", "place_cart of" + goal.cart_name + " IO recover status is False");
    }

    set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
    masterBlackboard->set("failed_robot_action",
                          "place_cart_" + goal.cart_name);
    return BT::NodeStatus::FAILURE;
  }

private:
  BT::Blackboard::Ptr masterBlackboard;
  std::vector<TableInfo> tables_values;
  bool recover_status;
  chargepal_actions::PlaceCartGoal goal;
  chargepal_actions::PlaceCartResult result;
  std::map<std::string, std::any> arg_param;
  std::string cart_on_robot, robot, robot_location, action_result;
};
#endif