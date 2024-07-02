#ifndef MODUS_START_H
#define MODUS_START_H

class battery_mode_req_charge : public BT::StatefulActionNode
{
public:
    battery_mode_req_charge(const std::string &name, const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
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
        charging_type = masterBlackboard->get<std::string>("charging_type");
        job = masterBlackboard->get<std::string>("job_type");
        std::chrono::minutes timeout_ms(std::any_cast<int>(arg_param["server_timeout"]) * 1000);
        deadline_ = system_clock::now() + timeout_ms;

        bcs.waitForServer();

        goal.cart_name = cart_name;

        if (job == "RECHARGE_CHARGER")
        {
            goal.request_name = "mode_req_Bat_AC_Charge";
        }
        else
        {
            goal.request_name = "mode_req_EV_" + charging_type + "_Charge";
        }
        tables_values = {{BATTERY_ACTIONS_TABLE, {cart_name, {{"action_state", std::string("performing_" + charging_type + "_modus_start_") + goal.cart_name}}}}};
        set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
        enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Performing modus start for " + cart_name);
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
                tables_values = {{BATTERY_ACTIONS_TABLE, {cart_name, {{"action_state", std::string("successful_" + charging_type + "_modus_start_") + goal.cart_name}}}}};
                set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
                enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Finished modus ende for " + cart_name);
                masterBlackboard->set("previous_battery_action", charging_type + "_modus_start_" + goal.cart_name);
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                tables_values = {{BATTERY_ACTIONS_TABLE, {cart_name, {{"action_state", std::string("failure_" + charging_type + "_modus_start_") + goal.cart_name}}}}};
                set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
                enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Failure of modus start for " + cart_name);
                masterBlackboard->set("failed_battery_action", charging_type + "_modus_start_" + goal.cart_name);

                return BT::NodeStatus::FAILURE;
            }
        }

        else
        {
            if (system_clock::now() >= deadline_)
            {
                bcs.cancelGoal();
                tables_values = {{BATTERY_ACTIONS_TABLE, {cart_name, {{"action_state", std::string("failure_" + charging_type + "_modus_start_") + goal.cart_name}}}}};
                set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
                enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Failure of modus start for " + cart_name + ". Action timeout reached");
                masterBlackboard->set("failed_battery_action", charging_type + "_modus_start_" + goal.cart_name);

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
    std::string cart_name, charging_type, job, robot;
    std::vector<TableInfo> tables_values;
    actionlib::SimpleActionClient<chargepal_actions::BatteryCommunicationAction> bcs;
    std::map<std::string, std::any> arg_param;
};
#endif