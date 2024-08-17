#ifndef BAT_ONLY_H
#define BAT_ONLY_H

class battery_mode_bat_only  : public BT::StatefulActionNode
{
public:
    battery_mode_bat_only (const std::string &name, const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
        : BT::StatefulActionNode(name, config), arg_param(arg_cp_params), bcs("battery_communication", true) {}

    static PortsList providedPorts()
    {
        return {};
    }
    // You must override the virtual function tick()
    virtual NodeStatus onStart() override
    {
        masterBlackboard = config().blackboard;
        cart_name = masterBlackboard->get<std::string>("cart");
        robot = masterBlackboard->get<std::string>("robot");
        std::chrono::minutes timeout_ms(std::any_cast<int>(arg_param["server_timeout"]) * 1000);
        deadline_ = system_clock::now() + timeout_ms;

        bcs.waitForServer();
        
        goal.cart_name = cart_name;
        goal.request_name = "mode_bat_only";
        tables_values = {{BATTERY_ACTIONS_TABLE, {cart_name, {{"action_state", std::string("peforming_bat_only _") + goal.cart_name}}}}};
        set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);

        enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Performing bat_only of " + cart_name);
        bcs.sendGoal(goal);
        return BT::NodeStatus::RUNNING;
    }

    /// method invoked by an action in the RUNNING state.
    virtual NodeStatus onRunning() override
    {
        if (bcs.waitForResult(ros::Duration(0.2)))
        {
            chargepal_actions::BatteryCommunicationResult result = *bcs.getResult();
            if (result.success)
            {
                tables_values = {{BATTERY_ACTIONS_TABLE, {cart_name, {{"action_state", std::string("successful_bat_only _") + goal.cart_name}}}}};
                set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
                enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Successful bat_only of " + cart_name);
                masterBlackboard->set("previous_battery_action", std::string("bat_only_") + goal.cart_name);

                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                tables_values = {{BATTERY_ACTIONS_TABLE, {cart_name, {{"action_state", std::string("failure_bat_only _") + goal.cart_name}}}}};
                set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
                enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Failure bat_only of " + cart_name);
                masterBlackboard->set("failed_battery_action", std::string("bat_only_") + goal.cart_name);
                return BT::NodeStatus::FAILURE;
            }
        }

        else
        {
            if (system_clock::now() >= deadline_)
            {
                bcs.cancelGoal();
                tables_values = {{BATTERY_ACTIONS_TABLE, {cart_name, {{"action_state", std::string("failure_bat_only _") + goal.cart_name}}}}};
                set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
                enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Failure bat_only of " + cart_name + ". Action timeout reached");
                masterBlackboard->set("failed_battery_action", std::string("bat_only_") + goal.cart_name);
                return BT::NodeStatus::FAILURE;
            }
            else
            {
                return BT::NodeStatus::RUNNING;
            }
        }
    }

    void onHalted() override
    {

        std::cout << "Waiting for parllel action to finish" << std::endl;
    }

private:
    system_clock::time_point deadline_;
    chargepal_actions::BatteryCommunicationGoal goal;
    BT::Blackboard::Ptr masterBlackboard;
    std::string cart_name, robot;
    std::vector<TableInfo> tables_values;
    actionlib::SimpleActionClient<chargepal_actions::BatteryCommunicationAction> bcs;
    std::map<std::string, std::any> arg_param;
};
#endif