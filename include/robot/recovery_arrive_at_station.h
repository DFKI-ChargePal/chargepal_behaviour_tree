#ifndef RECOVERY_ARRIVE_AT_STATION_H
#define RECOVERY_ARRIVE_AT_STATION_H

class recovery_arrive_BWS : public BT::SyncActionNode
{
public:
    recovery_arrive_BWS(const std::string &name,
                        const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
        : BT::SyncActionNode(name, config), arg_param(arg_cp_params) {}

    static PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {

        masterBlackboard = config().blackboard;
        robot = masterBlackboard->get<std::string>("robot");
        cart_on_robot = read_robot_value(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, "cart_on_robot");

        actionlib::SimpleActionClient<chargepal_actions::ArriveAtStationAction> aas(
            "arrive_at_station", true);

        enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Asking for free BWS");

        auto free_bws_result = ask_free_BWS(ask_ldb);
        free_bws = free_bws_result.first;
        connection_status = free_bws_result.second;
        enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "   Received: " + free_bws);
        enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "   Server connection: " + connection_status);

        while (free_bws != "none" || free_bws != "")
        {

            while (retry_attempt < 3)
            {
                aas.waitForServer();
                chargepal_actions::ArriveAtStationGoal goal;
                robot_location = read_robot_value(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, "robot_location");
                goal.target_station = free_bws;
                station_transition = robot_location + "_" + goal.target_station;
                aas.sendGoal(goal);
                if (cart_on_robot != "none" && cart_on_robot != "")
                {
                    tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("recovery_arrive_") + free_bws}, {"robot_location", station_transition}}}}, {CART_TABLE, {cart_on_robot, {{"cart_location", station_transition}}}}};
                }
                else
                {
                    tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("recovery_arrive_") + free_bws}, {"robot_location", station_transition}}}}};
                }

                set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);

                enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Performing arrive_at_station to " +
                                                                                           goal.target_station);
                bool aas_action = aas.waitForResult(ros::Duration(900.0));

                if (aas_action)
                {
                    chargepal_actions::ArriveAtStationResult result = *aas.getResult();
                    bool station_reached = result.station_reached;
                    if (station_reached)
                    {
                        std::string current_station = result.current_station;
                        if (cart_on_robot != "none")
                        {
                            tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", std::string("recovery_arrive_") + free_bws}, {"robot_location", current_station}}}}, {CART_TABLE, {cart_on_robot, {{"cart_location", current_station}}}}};
                            set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
                        }
                        else
                        {
                            tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", std::string("recovery_arrive_") + free_bws}, {"robot_location", current_station}}}}};
                            set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
                        }
                        reset_station_blocker("BWS");
                        masterBlackboard->set("previous_robot_action", "recovery_arrive_" + goal.target_station);
                        return BT::NodeStatus::SUCCESS;
                    }
                    else
                    {
                        retry_attempt += 1;
                        if (retry_attempt == 3)
                        {
                            enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "arrive_at_station to" + goal.target_station +
                                                                                                       " status is " + action_result);
                            masterBlackboard->set("failed_robot_action",
                                                  std::string("recovery_arrive_") + goal.target_station);
                            action_result = result.action_status;
                            tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", std::string("recovery_arrive_") + goal.target_station + std::string("_") + action_result}}}}};
                            set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
                            masterBlackboard->set("failed_robot_action", "recovery_arrive_" + goal.target_station);
                        }
                    }
                }
                else
                {
                    enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "arrive_at_station to" + goal.target_station +
                                                                                               " cannot be finished within a timeout of 900 seconds");
                    aas.cancelGoal();
                    ros::Duration(5.0).sleep();
                    retry_attempt += 1;
                    if (retry_attempt == 3)
                    {
                        masterBlackboard->set("failed_robot_action", std::string("recovery_arrive_") + goal.target_station);
                        tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", std::string("recovery_arrive_") + goal.target_station + std::string("_ActionTimeout")}}}}};
                        set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
                        masterBlackboard->set("failed_robot_action", "recovery_arrive_" + goal.target_station);
                    }
                }
            }

            auto free_bws_result = ask_free_BWS(ask_ldb);
            free_bws = free_bws_result.first;
            connection_status = free_bws_result.second;
            enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "   Received: " + free_bws);
            enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "   Server connection: " + connection_status);
            if (free_bws == "none" || free_bws == "")
            {
                break;
            }
        }

        enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "No free BWS left");
        return BT::NodeStatus::FAILURE;
    }

private:
    BT::Blackboard::Ptr masterBlackboard;
    std::vector<TableInfo> tables_values;
    int retry_attempt = 0;
    int ldb_connection_error_count = -1;
    bool ask_ldb = true;
    std::string free_bws = "";
    std::string connection_status = "";
    std::map<std::string, std::any> arg_param;
    std::string robot, robot_location, cart_on_robot, action_result, station_transition;
};

class recovery_arrive_BCS : public BT::SyncActionNode
{
public:
    recovery_arrive_BCS(const std::string &name,
                        const BT::NodeConfiguration &config, std::map<std::string, std::any> arg_cp_params)
        : BT::SyncActionNode(name, config), arg_param(arg_cp_params) {}

    static PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        masterBlackboard = config().blackboard;
        robot = masterBlackboard->get<std::string>("robot");

        cart_on_robot = read_robot_value(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, "cart_on_robot");

        actionlib::SimpleActionClient<chargepal_actions::ArriveAtStationAction> aas(
            "arrive_at_station", true);

        enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Asking for free BCS");

        auto free_bcs_result = ask_free_BCS(ask_ldb);
        free_bcs = free_bcs_result.first;
        connection_status = free_bcs_result.second;
        enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "   Received: " + free_bcs);
        enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "   Server connection: " + connection_status);

        while (free_bcs != "none" || free_bcs != "")
        {
            while (retry_attempt < 3)
            {
                aas.waitForServer();
                chargepal_actions::ArriveAtStationGoal goal;
                robot_location = read_robot_value(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, "robot_location");
                goal.target_station = free_bcs;
                station_transition = robot_location + "_" + goal.target_station;
                aas.sendGoal(goal);
                if (cart_on_robot != "none" && cart_on_robot != "")
                {
                    tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("recovery_arrive_") + free_bcs}, {"robot_location", station_transition}}}}, {CART_TABLE, {cart_on_robot, {{"cart_location", station_transition}}}}};
                }
                else
                {
                    tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("recovery_arrive_") + free_bcs}, {"robot_location", station_transition}}}}};
                }

                enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "Performing arrive_at_station to " +
                                                                                           goal.target_station);
                bool aas_action = aas.waitForResult(ros::Duration(900.0));
                if (aas_action)
                {
                    chargepal_actions::ArriveAtStationResult result = *aas.getResult();
                    bool station_reached = result.station_reached;
                    if (station_reached)
                    {
                        std::string current_station = result.current_station;

                        if (cart_on_robot != "none")
                        {
                            tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", std::string("recovery_arrive_") + free_bcs}, {"robot_location", current_station}}}}, {CART_TABLE, {cart_on_robot, {{"cart_location", current_station}}}}};
                            set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
                        }
                        else
                        {
                            tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", std::string("recovery_arrive_") + free_bcs}, {"robot_location", current_station}}}}};
                            set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
                        }
                        reset_station_blocker("BCS");
                        masterBlackboard->set("previous_robot_action", std::string("recovery_arrive_") + goal.target_station);
                        return BT::NodeStatus::SUCCESS;
                    }
                    else
                    {
                        retry_attempt += 1;
                        if (retry_attempt == 3)
                        {
                            enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "arrive_at_station to" + goal.target_station +
                                                                                                       " status is " + action_result);
                            masterBlackboard->set("failed_robot_action",
                                                  std::string("recovery_arrive_") + goal.target_station);
                            action_result = result.action_status;
                            tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", std::string("recovery_arrive_") + goal.target_station + std::string("_") + action_result}}}}};
                            set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
                            masterBlackboard->set("failed_robot_action", "recovery_arrive_" + goal.target_station);
                        }
                    }
                }
                else
                {
                    enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "arrive_at_station to" + goal.target_station +
                                                                                               " cannot be finished within a timeout of 900 seconds");
                    aas.cancelGoal();
                    ros::Duration(5.0).sleep();
                    retry_attempt += 1;
                    if (retry_attempt == 3)
                    {
                        masterBlackboard->set("failed_robot_action", std::string("recovery_arrive_") + goal.target_station);
                        tables_values = {{ROBOT_TABLE, {robot, {{"ongoing_action", std::string("none")}, {"previous_action", std::string("recovery_arrive_") + goal.target_station + std::string("_ActionTimeout")}}}}};
                        set_rdbc_values(std::any_cast<std::string>(arg_param["rdbc_path"]), robot, tables_values);
                        masterBlackboard->set("failed_robot_action", "recovery_arrive_" + goal.target_station);
                    }
                }
            }

            auto free_bcs_result = ask_free_BCS(ask_ldb);
            free_bcs = free_bcs_result.first;
            connection_status = free_bcs_result.second;
            enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "   Received: " + free_bcs);
            enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "   Server connection: " + connection_status);
            if (free_bcs == "none" || free_bcs == "")
            {
                break;
            }
        }

        enter_log_file(std::any_cast<std::string>(arg_param["log_file_path"]), "No free BCS left");
        return BT::NodeStatus::FAILURE;
    }

private:
    BT::Blackboard::Ptr masterBlackboard;
    std::vector<TableInfo> tables_values;
    int retry_attempt = 0;
    int ldb_connection_error_count = -1;
    bool ask_ldb = true;
    std::string free_bcs = "";
    std::string connection_status = "";
    std::map<std::string, std::any> arg_param;
    std::string robot, robot_location, cart_on_robot, action_result, station_transition;
};
#endif