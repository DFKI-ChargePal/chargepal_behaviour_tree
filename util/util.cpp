#include "util.h"

/**
 * @brief This function calls the service to update the rdb_copy 
 * 
 * @return The boolean response to the service call
 */
bool update_rdb_copy() {
    ros::NodeHandle n;
    ros::ServiceClient client_ldb_server = n.serviceClient<chargepal_services::updateRdbCopy>("/robot_db/update_rdb_copy");
    chargepal_services::updateRdbCopy srv_ldb_server;

    if (client_ldb_server.call(srv_ldb_server)){
        
        return srv_ldb_server.response.success; 
    }

}

/**
 * @brief This function calls the service to push the robot database to local data base 
 * 
 * @param table: table name to push to
 * @param item: row name to be pushed 
 * @return The boolean response to the service call
 */
bool push_rdb_copy(const std::string& table,const std::string& item) {
    ros::NodeHandle n;
    ros::ServiceClient client_ldb_server = n.serviceClient<chargepal_services::pushToLDB>("/ldb_server/push_to_ldb");
    chargepal_services::pushToLDB srv_ldb_server;
    srv_ldb_server.request.table_name = table;
    srv_ldb_server.request.row_name = item;

    if (client_ldb_server.call(srv_ldb_server)){
        std::string connection = srv_ldb_server.response.connection_status;
        return srv_ldb_server.response.success; 
    }

}

/**
 * @brief This function calls the service to verify syncronization between local database and robot database 
 * 
 * @return The boolean response to the service call
 */
bool verify_sync() {
    ros::NodeHandle n;
    ros::ServiceClient client_ldb_server = n.serviceClient<chargepal_services::verifyRdbSync>("/robot_db/verify_rdb_rdb_copy_sync");
    chargepal_services::verifyRdbSync srv_ldb_server;

    if (client_ldb_server.call(srv_ldb_server)){
        return srv_ldb_server.response.success; 
    }

}

/**
 * @brief This function helps reading a robot information from the robot databse 
 * 
 * @param name: the name of the robot
 * @param key: the name of the column in robot_info table 
 * @return The string response of the value in the column
 */
std::string read_robot_value(const std::string& name,const std::string& key) {
    // Update the rdb_copy from rdb beofre reading any information
    bool updated_rdb_copy = update_rdb_copy();

    if (updated_rdb_copy) {
        std::string robot_name = name;
        std::string selected_column = key;
        SQLite::Database db(ros::package::getPath("chargepal_bundle")+"/db/rdb_copy.db");
        SQLite::Statement query(db,"SELECT CASE ? "
                            "WHEN 'robot_location' THEN robot_location "
                            "WHEN 'cart_on_robot' THEN cart_on_robot "
                            "WHEN 'previous_action' THEN previous_action "
                            "WHEN 'ongoing_action' THEN ongoing_action "
                            "WHEN 'current_job' THEN current_job "
                            "WHEN 'error_count' THEN error_count "
                            "ELSE NULL END AS result "
                            "FROM robot_info WHERE robot_name = ?");

        query.bind(1, selected_column);
        query.bind(2, robot_name);
        std::string position;
        while (query.executeStep()) {
            return query.getColumn(0).getText();
        }
        query.reset();
    }
}

/**
 * @brief This function helps reading a cart information from the robot databse 
 * 
 * @param name: the name of the cart
 * @param key: the name of the column in cart_info table 
 * @return The string response of the value in the column
 */
std::string read_cart_value(const std::string& name,const std::string& key) {
    bool updated_rdb_copy = update_rdb_copy();

    if (updated_rdb_copy) {
        std::string cart_name = name;
        std::string selected_column = key;
        SQLite::Database db(ros::package::getPath("chargepal_bundle")+"/db/rdb_copy.db");
        SQLite::Statement query(db,"SELECT CASE ? "
                            "WHEN 'cart_location' THEN cart_location "
                            "WHEN 'robot_on_cart' THEN robot_on_cart "
                            "WHEN 'plugged' THEN plugged "
                            "ELSE NULL END AS result "
                            "FROM cart_info WHERE cart_name = ?");
        //SQLite::Statement query(db, "SELECT robot_location FROM robot_info WHERE cart_name = ?");
        query.bind(1, selected_column);
        query.bind(2, cart_name);
        std::string position;
        while (query.executeStep()) {
            return query.getColumn(0).getText();
        }
        query.reset();
    }
}

/**
 * @brief This function helps update a robot value in the robot databse 
 * 
 * @param name: the name of the robot
 * @param key: the name of the column in robot_info table 
 * @param value: the value to be updated  
 * @return The boolean response of update success
 */
bool set_robot_value(const std::string& name,const std::string& key,const std::string& value) {
    bool updated_rdb_copy = update_rdb_copy();
    if (updated_rdb_copy) {

        SQLite::Database db(ros::package::getPath("chargepal_bundle")+"/db/rdb_copy.db",SQLite::OPEN_READWRITE|SQLite::OPEN_CREATE);
        
        std::string robot_name = name; 
        std::string column_to_update = key; 
        std::string new_value = value;
        std::string table_name = "robot_info";                

        std::string sql_query = "UPDATE robot_info SET " + column_to_update + " = ? WHERE robot_name = ?";
        SQLite::Statement query(db,sql_query);
        query.bind(1, new_value);
        query.bind(2, robot_name);
        query.exec(); 
        query.reset();

        update_robot_charge(robot_name);

        push_rdb_copy(table_name,robot_name);
        std::this_thread::sleep_for(std::chrono::seconds(3)); //To account for updating rdb from ldb
        verify_sync();
        
        return true;
    }
    
}

/**
 * @brief This function helps fetching the robot charge and updating it in the table 
 * 
 * @param robot_name: the name of the robot
 */
void update_robot_charge(const std::string& robot_name){
    ros::NodeHandle n;
    ros::ServiceClient client_ldb_server = n.serviceClient<chargepal_services::readRobotCharge>("/mir_rest_api/robot_charge");
    chargepal_services::readRobotCharge srv_ldb_server;
    srv_ldb_server.request.robot_name = robot_name;

    if (client_ldb_server.call(srv_ldb_server)){
        float charge = srv_ldb_server.response.robot_charge;

        SQLite::Database db(ros::package::getPath("chargepal_bundle")+"/db/rdb_copy.db",SQLite::OPEN_READWRITE|SQLite::OPEN_CREATE);
        std::string sql_query = "UPDATE robot_info SET robot_charge = ? WHERE robot_name = ?";
        SQLite::Statement query(db,sql_query);
        query.bind(1, charge);
        query.bind(2, robot_name);
        query.exec(); 
        query.reset();
    }
}

/**
 * @brief This function helps update a cart value in the robot databse 
 * 
 * @param name: the name of the cart
 * @param key: the name of the column in cart_info table 
 * @param value: the value to be updated  
 * @return The boolean response of update success
 */
bool set_cart_value(const std::string& name,const std::string& key,const std::string& value) {
    
    bool updated_rdb_copy = update_rdb_copy();
    if (updated_rdb_copy) {
        SQLite::Database db(ros::package::getPath("chargepal_bundle")+"/db/rdb_copy.db",SQLite::OPEN_READWRITE|SQLite::OPEN_CREATE);
        
        std::string cart_name = name; 
        std::string column_to_update = key; 
        std::string new_value = value;
        std::string table_name = "cart_info";                 

        std::string sql_query = "UPDATE cart_info SET " + column_to_update + " = ? WHERE cart_name = ?";
        SQLite::Statement query(db,sql_query);
        query.bind(1, new_value);
        query.bind(2, cart_name);
        query.exec(); 
        query.reset();

        push_rdb_copy(table_name,cart_name);
        std::this_thread::sleep_for(std::chrono::seconds(3)); //To account for updating rdb from ldb
        verify_sync();

        return true;
  
    }
}

/**
 * @brief This function calls the service to reset a blocked station in the local database
 * 
 * @param station_name: the name of the station
 * @return The boolean response of update success
 */
bool reset_station_blocker(const std::string& station_name) {

    ros::NodeHandle n;
    ros::ServiceClient client_ldb_server = n.serviceClient<chargepal_services::resetStationBlocker>("/ldb_server/reset_station_blocker");
    chargepal_services::resetStationBlocker srv_ldb_server;
    srv_ldb_server.request.station = station_name;

    if (client_ldb_server.call(srv_ldb_server)){
        std::string connection = srv_ldb_server.response.connection_status;
        return srv_ldb_server.response.success; 
    }

}

/**
 * @brief This function calls the service to ask the database for an available battery waiting station
 * 
 * @param ask_ldb: a boolean if to ask the local databse or not
 * @return returns a pair of strings containing the available battery waiting station and connection status to the local database
 */
std::pair<std::string,std::string> ask_free_BWS(const bool& ask_ldb) {
    std::string connection_status = "";
    std::string free_bws = "";
    ros::NodeHandle n;
    ros::ServiceClient client_ldb_server = n.serviceClient<chargepal_services::askFreeBWS>("/ldb_server/ask_free_bws");
    chargepal_services::askFreeBWS srv_ldb_server;

    srv_ldb_server.request.ask_ldb = ask_ldb;
    if (client_ldb_server.call(srv_ldb_server)){
        free_bws = srv_ldb_server.response.station_name;
        connection_status = srv_ldb_server.response.connection_status;
    }
    else {
        ROS_ERROR("ask_free_bws service failed");
    }
    return std::make_pair(free_bws,connection_status);
}

/**
 * @brief This function calls the service to ask the database for an available battery charging station
 * 
 * @param ask_ldb: a boolean if to ask the local databse or not
 * @return returns a pair of strings containing the available battery charging station and connection status to the local database
 */
std::pair<std::string,std::string> ask_free_BCS(const bool& ask_ldb) {
    std::string connection_status = "";
    std::string free_bcs = "";
    ros::NodeHandle n;
    ros::ServiceClient client_ldb_server = n.serviceClient<chargepal_services::askFreeBCS>("/ldb_server/ask_free_bcs");
    chargepal_services::askFreeBCS srv_ldb_server;
    
    srv_ldb_server.request.ask_ldb = ask_ldb;
    if (client_ldb_server.call(srv_ldb_server)){
        free_bcs = srv_ldb_server.response.station_name;
        connection_status = srv_ldb_server.response.connection_status;
    }
    else {
        ROS_ERROR("ask_free_bcs service failed");
    }
    return std::make_pair(free_bcs,connection_status);
}

/**
 * @brief This function calls the service to ask the mir platform if the status of assert lift 
 * 
 * @param robot_name: the name of the robot
 * @return returns a boolean of assert lift value
 */
std::string read_assertLift_value(const std::string& robot_name) {
    std::string assert_lift_value = "";
    ros::NodeHandle n;
    ros::ServiceClient client_mir_rapi = n.serviceClient<chargepal_services::assertLiftValue>("/mir_rest_api/assert_lift_value");
    chargepal_services::assertLiftValue srv_mir_rapi;
    if (client_mir_rapi.call(srv_mir_rapi)){
        assert_lift_value = srv_mir_rapi.response.state;
    }
    else {
        ROS_ERROR("assert_lift_value service failed");
    }
    return assert_lift_value;
}

/**
 * @brief This function calls the service to fetch a job from the local server 
 * 
 * @return returns a string that contains the job
 */
std::string fetch_job() {
    std::string job;
    ros::NodeHandle n;
    ros::ServiceClient client_ldb_server = n.serviceClient<chargepal_services::fetchJob>("/ldb_server/fetch_job");
    chargepal_services::fetchJob srv_ldb_server;

    try{
        if (client_ldb_server.call(srv_ldb_server)){
            
            job = srv_ldb_server.response.job;
            
        }
        else {
            ROS_ERROR("Fetch job service failed");
        }
    }
    catch(...){
        ROS_INFO("Connecting to server failed");
    }


    return job;
}

/**
 * @brief This function calls the service to update the job monitor in the local server 
 * 
 * @return returns a boolean of update success value
 */
bool update_job_monitor(const std::string& job_type, std::string& job_status) {
    ros::NodeHandle n;
    ros::ServiceClient client_ldb_server = n.serviceClient<chargepal_services::updateJobMonitor>("/ldb_server/update_job_monitor");

    chargepal_services::updateJobMonitor srv_ldb_server;
    srv_ldb_server.request.job_type = job_type;
    srv_ldb_server.request.job_status = job_status;

    if (client_ldb_server.call(srv_ldb_server)){
        return srv_ldb_server.response.success;
    }
    else {
        ROS_ERROR("updating job monitor failed");
    }

}

/**
 * @brief This function calls the service to delete the queue actions in mir platform
 *
 * @return returns a boolean of update success
 */
bool delete_mission_queue(){
    ros::NodeHandle n;
    ros::ServiceClient client_mir_rapi = n.serviceClient<chargepal_services::deleteMirMission>("/mir_rest_api/delete_mission_queue");
    chargepal_services::deleteMirMission srv_mir_rapi;
    if (client_mir_rapi.call(srv_mir_rapi)){
        if (srv_mir_rapi.response.success == false) {
            ROS_ERROR("mir_delete_mission service failed");
            return false;
        }
        else{
            return true;
        }
    }       
    else{
        ROS_ERROR("Failed to service call mir_delete_mission");
        return false;
    }
        
}

/**
 * @brief This function calls the service to fetch the booking time of the cart in mseconds  
 * 
 * @param cart_name: the name of the cart
 * @return returns the time in mseconds
 */
int get_operation_time(const std::string& cart_name){
    int operation_time = 0;
    ros::NodeHandle n;
    ros::ServiceClient client_ldb_server = n.serviceClient<chargepal_services::askOperationTime>("/ldb_server/charging_operation_time");
    chargepal_services::askOperationTime srv_ldb_server;
    srv_ldb_server.request.cart_name = cart_name;
    if (client_ldb_server.call(srv_ldb_server)){
        operation_time = srv_ldb_server.response.time_msec;
    }
    else {
        ROS_ERROR("asking operation time service failed");
    }
    return operation_time;
}

std::string enumToString(JobEnum value) {
    switch(value) {
        case JobEnum::ONGOING:
            return "Ongoing";
        case JobEnum::SUCCESS:
            return "Success";
        case JobEnum::FAILURE:
            return "Failure";
        case JobEnum::RECOVERY:
            return "Recovery";
    }
}

    