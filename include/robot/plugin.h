#ifndef PLUGIN_H
#define PLUGIN_H

class plugin_ADS : public BT::SyncActionNode
{
public:
    plugin_ADS(const std::string &name, const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
        : BT::SyncActionNode(name, config), arg_param(arg_cp_params) {}

    static PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        sim_flag = std::any_cast<bool>(arg_param["sim_flag"]);
        masterBlackboard = config().blackboard;
        cart = masterBlackboard->get<std::string>("cart");
        robot = masterBlackboard->get<std::string>("robot");
        target_station = masterBlackboard->get<std::string>("target_station");
        charging_type = masterBlackboard->get<std::string>("charging_type");
        std::transform(charging_type.begin(), charging_type.end(), charging_type.begin(), ::tolower);

        enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Performing plugin_ADS" + charging_type + " at " + target_station);
        if (sim_flag)
        {
            tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("plugin__cart_") + charging_type + target_station}}}}};
            set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);

            ros::Duration(10).sleep();
            tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", std::string("plugin_cart_") + charging_type + target_station}}}}, {CART_TABLE, {cart, {{"plugged", true}}}}};
            set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);

            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            actionlib::SimpleActionClient<chargepal_actions::PlugInAdsAcAction> pi_ads("plug_in_ads_" + charging_type, true);
            pi_ads.waitForServer();

            pi_ads.sendGoal(goal);
            tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("plugin_cart_") + charging_type + target_station}}}}};
            set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);

            pi_ads_action = pi_ads.waitForResult(ros::Duration(900.0));
            if (pi_ads_action)
            {
                result = *pi_ads.getResult();
                bool plug_in = result.success;
                if (plug_in)
                {
                    tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", std::string("plugin_cart_") + charging_type + target_station}}}}, {CART_TABLE, {cart, {{"plugged", true}}}}};
                    set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
                    masterBlackboard->set("previous_robot_action", "Plugin at" + charging_type + target_station);
                    return BT::NodeStatus::SUCCESS;
                }
            }

            enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Plugin_ADS failed at" + charging_type + target_station);
            tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", std::string("plugin_cart_failure") + charging_type + target_station}}}}};
            set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);

            masterBlackboard->set("failed_robot_action", "Plugin at" + charging_type + target_station);
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    bool sim_flag, pi_ads_action;
    BT::Blackboard::Ptr masterBlackboard;
    std::vector<TableInfo> tables_values;
    chargepal_actions::PlugInAdsAcGoal goal;
    chargepal_actions::PlugInAdsAcResult result;
    std::string cart, robot, target_station, charging_type;
    std::map<std::string, std::any> arg_param;
};

class plugin_BCS : public BT::SyncActionNode
{
public:
    plugin_BCS(const std::string &name, const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
        : BT::SyncActionNode(name, config), arg_param(arg_cp_params) {}

    static PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        sim_flag = std::any_cast<bool>(arg_param["sim_flag"]);
        masterBlackboard = config().blackboard;
        cart = masterBlackboard->get<std::string>("cart");
        robot = masterBlackboard->get<std::string>("robot");
        target_station = masterBlackboard->get<std::string>("target_station");
        enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Performing plugin_BCS at " + target_station);
        if (sim_flag)
        {
            tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("plugin_cart_") + target_station}}}}};
            set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);

            ros::Duration(10).sleep();
            tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", std::string("plugin_cart_") + target_station}}}}, {CART_TABLE, {cart, {{"plugged", true}}}}};
            set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);

            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            actionlib::SimpleActionClient<chargepal_actions::PlugInBcsAcAction> pi_bcs("plug_in_bcs", true);
            pi_bcs.waitForServer();

            pi_bcs.sendGoal(goal);
            tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("plugin_cart_") + target_station}}}}};
            set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);

            pi_bcs_action = pi_bcs.waitForResult(ros::Duration(900.0));
            if (pi_bcs_action)
            {
                result = *pi_bcs.getResult();
                bool plug_in = result.success;
                if (plug_in)
                {
                    tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", std::string("plugin_cart_") + target_station}}}}, {CART_TABLE, {cart, {{"plugged", true}}}}};
                    set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
                    masterBlackboard->set("previous_robot_action", "Plugin at" + target_station);
                    return BT::NodeStatus::SUCCESS;
                }
            }

            enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Plugin_ADS failed at" + target_station);
            tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", std::string("plugin_cart_failure") + target_station}}}}};
            set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);

            masterBlackboard->set("failed_robot_action", "Plugin at" + target_station);
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    bool sim_flag, pi_bcs_action;
    BT::Blackboard::Ptr masterBlackboard;
    std::vector<TableInfo> tables_values;
    chargepal_actions::PlugInBcsAcGoal goal;
    chargepal_actions::PlugInBcsAcResult result;
    std::string cart, robot, target_station;
    std::map<std::string, std::any> arg_param;
};
#endif