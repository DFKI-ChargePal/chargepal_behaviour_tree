#ifndef PLUGOUT_H
#define PLUGOUT_H

class plugout_ADS : public BT::SyncActionNode
{
public:
  plugout_ADS(const std::string &name, const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
      : BT::SyncActionNode(name, config), arg_param(arg_cp_params) {}

  static PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    sim_flag = std::any_cast<bool>(arg_param["sim_flag"]);
    masterBlackboard = config().blackboard;
    cart = masterBlackboard->get<std::string>("cart");
    robot = masterBlackboard->get<std::string>("robot");
    location = read_robot_value(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, "robot_location");
    enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Performing plugout_ADS at " + location);
    if (sim_flag)
    {
      tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("plugout_cart_ads")}}}}};
      set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
      ros::Duration(10).sleep();
      tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", std::string("plugout_cart_")+location}}}}, {CART_TABLE, {cart, {{"plugged", false}}}}};
      set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);

      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      actionlib::SimpleActionClient<chargepal_actions::PlugOutAdsAcAction> po_ads("plug_out_ads_ac", true);
      po_ads.waitForServer();
      po_ads.sendGoal(goal);

      tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("plugout_cart_")+location}}}}};
      set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
      po_ads_action = po_ads.waitForResult(ros::Duration(900.0));
      if (po_ads_action)
      {
        result = *po_ads.getResult();
        bool plug_out = result.success;
        if (plug_out)
        {
          tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", std::string("plugout_cart_")+location}}}}, {CART_TABLE, {cart, {{"plugged", false}}}}};
          set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
          masterBlackboard->set("previous_robot_action", "Plugout at" + location);
          return BT::NodeStatus::SUCCESS;
        }
      }

      enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Plugout_ADS failed at " + location);
      tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", std::string("plugout_cart_failure")+location}}}}};
      set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
      masterBlackboard->set("failed_robot_action", "Plugout at" + location);
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  bool sim_flag, po_ads_action;
  BT::Blackboard::Ptr masterBlackboard;
  std::vector<TableInfo> tables_values;
  chargepal_actions::PlugOutAdsAcGoal goal;
  chargepal_actions::PlugOutAdsAcResult result;
  std::string cart, robot, location;
  std::map<std::string, std::any> arg_param;
};

class plugout_BCS : public BT::SyncActionNode
{
public:
  plugout_BCS(const std::string &name, const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
      : BT::SyncActionNode(name, config), arg_param(arg_cp_params) {}

  static PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    sim_flag = std::any_cast<bool>(arg_param["sim_flag"]);
    masterBlackboard = config().blackboard;
    cart = masterBlackboard->get<std::string>("cart");
    robot = masterBlackboard->get<std::string>("robot");
    location = read_robot_value(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, "robot_location");
    enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Performing plugout_BCS at " + location);
    if (sim_flag)
    {
      tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("plugout_cart_")+location}}}}};
      set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
      ros::Duration(10).sleep();
      tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", std::string("plugout_cart_bcs")}}}}, {CART_TABLE, {cart, {{"plugged", false}}}}};
      set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);

      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      actionlib::SimpleActionClient<chargepal_actions::PlugOutBcsAcAction> po_bcs("plug_out_bcs", true);
      po_bcs.waitForServer();
      po_bcs.sendGoal(goal);

      tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("plugout_cart_")+location}}}}};
      set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
      po_bcs_action = po_bcs.waitForResult(ros::Duration(900.0));
      if (po_bcs_action)
      {
        result = *po_bcs.getResult();
        bool plug_out = result.success;
        if (plug_out)
        {
          tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", std::string("plugout_cart_")+location}}}}, {CART_TABLE, {cart, {{"plugged", false}}}}};
          set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
          masterBlackboard->set("previous_robot_action", "Plugout at" + location);
          return BT::NodeStatus::SUCCESS;
        }
      }

      enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Plugout_BCS failed at " + location);
      tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", std::string("plugout_cart_failure")+location}}}}};
      set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
      masterBlackboard->set("failed_robot_action", "Plugout at" + location);
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  bool sim_flag, po_bcs_action;
  BT::Blackboard::Ptr masterBlackboard;
  std::vector<TableInfo> tables_values;
  chargepal_actions::PlugOutBcsAcGoal goal;
  chargepal_actions::PlugOutBcsAcResult result;
  std::string cart, robot, location;
  std::map<std::string, std::any> arg_param;
};
#endif