#ifndef PICKUP_CART_H
#define PICKUP_CART_H

/**
 * @brief The `pickup_cart` class is a synchronous action node in the behavior tree.
 * It represents the action of picking up a cart.
 */
class pickup_cart : public BT::SyncActionNode
{
public:
  pickup_cart(const std::string &name, const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
      : BT::SyncActionNode(name, config), arg_param(arg_cp_params) {}

  static PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {

    actionlib::SimpleActionClient<chargepal_actions::PickUpCartAction> puc(
        "pick_up_cart", true);
    puc.waitForServer();

    masterBlackboard = config().blackboard;
    cart = masterBlackboard->get<std::string>("cart");
    robot = masterBlackboard->get<std::string>("robot");
    cart_location = read_cart_value(std::any_cast<std::string>(arg_param["rdbc_path"]), cart, "cart_location");

    goal.cart_name = cart;

    puc.sendGoal(goal);
    tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("pickup_cart_") + goal.cart_name}}}}};
    set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
    enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Performing pickup_cart of " + goal.cart_name);
    update_gui_config("ongoing_action", "Performing pickup_cart of " + goal.cart_name);
    bool puc_action = puc.waitForResult(ros::Duration(900.0));
    if (puc_action)
    {
      chargepal_actions::PickUpCartResult result = *puc.getResult();
      bool cart_picked = result.cart_picked;
      if (cart_picked)
      {
        tables_values = {{ROBOT_TABLE, {robot, {{"cart_on_robot", cart}, {"ongoing_action", std::string("none")}, {"previous_action", std::string("pickup_cart_") + goal.cart_name}, {"robot_location", cart_location}}}}, {CART_TABLE, {cart, {{"robot_on_cart", robot}}}}};
        set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
        masterBlackboard->set("previous_robot_action", "pickup_cart_" + goal.cart_name);
        return BT::NodeStatus::SUCCESS;
      }
      action_result = result.action_status;

      enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "pickup_cart of" + goal.cart_name + " status is " +
                                                                                 action_result);
      update_gui_config("ongoing_action", "pickup_cart of" + goal.cart_name + " status is " + action_result);
      tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", std::string("pickup_cart_") + goal.cart_name + "_" + action_result}}}}};
    }
    else
    {
      puc.cancelGoal();
      enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "pickup_cart of" + goal.cart_name + " cannot be finished within a timeout of 900 seconds");
      update_gui_config("ongoing_action", "pickup_cart of" + goal.cart_name + " cannot be finished within a timeout of 900 seconds");
      tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", std::string("pickup_cart_") + goal.cart_name + std::string("_ActionTimeout")}}}}};
    }

    recover_status = recover_cart("pickup_cart");
    if (!recover_status)
    {
      enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "pickup_cart of" + goal.cart_name +
                                                                                 " IO recover status is False");
      update_gui_config("ongoing_action", "pickup_cart of" + goal.cart_name + " IO recover status is False");
    }
    set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
    masterBlackboard->set("failed_robot_action",
                          "pickup_cart_" + goal.cart_name);
    return BT::NodeStatus::FAILURE;
  }

private:
  BT::Blackboard::Ptr masterBlackboard;
  std::vector<TableInfo> tables_values;
  bool recover_status;
  chargepal_actions::PickUpCartGoal goal;
  std::string cart, robot, cart_location, action_result;
  std::map<std::string, std::any> arg_param;
};
#endif