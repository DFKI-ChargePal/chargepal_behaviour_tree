#include "ros/ros.h"
#include "util.h"
#include <ros/package.h>
#include <SQLiteCpp/SQLiteCpp.h>
#include <string>
#include <list>

std::string read_robot_value(const std::string& name,const std::string& key) {
    std::string robot_name = name;
    std::string selected_column = key;
    SQLite::Database db(ros::package::getPath("chargepal_ldb")+"/chargepal_ldb.db");
    SQLite::Statement query(db,"SELECT CASE ? "
                        "WHEN 'robot_location' THEN robot_location "
                        "WHEN 'cart_on_robot' THEN cart_on_robot "
                        "WHEN 'previous_action' THEN previous_action "
                        "WHEN 'ongoing_action' THEN ongoing_action "
                        "WHEN 'current_job' THEN current_job "
                        "ELSE NULL END AS result "
                        "FROM robot_info WHERE robot_name = ?");
    //SQLite::Statement query(db, "SELECT robot_location FROM robot_info WHERE robot_name = ?");
    query.bind(1, selected_column);
    query.bind(2, robot_name);
    std::string position;
    while (query.executeStep()) {
        return query.getColumn(0).getText();
    }
    query.reset();
    // Implementation of function1
}

std::string read_cart_value(const std::string& name,const std::string& key) {
    std::string cart_name = name;
    std::string selected_column = key;
    SQLite::Database db(ros::package::getPath("chargepal_ldb")+"/chargepal_ldb.db");
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
    // Implementation of function1
}

bool set_robot_value(const std::string& name,const std::string& key,const std::string& value) {
    
    SQLite::Database db(ros::package::getPath("chargepal_ldb")+"/chargepal_ldb.db",SQLite::OPEN_READWRITE|SQLite::OPEN_CREATE);
    
    std::string robot_name = name; 
    std::string column_to_update = key; 
    std::string new_value = value;                   

    std::string sql_query = "UPDATE robot_info SET " + column_to_update + " = ? WHERE robot_name = ?";
    SQLite::Statement query(db,sql_query);
    query.bind(1, new_value);
    query.bind(2, robot_name);
    query.exec(); 
    query.reset();
    return true;
    
    //ToDo: Update the database on the server, receive a handshake that the value is updated. wait until received
}

bool set_cart_value(const std::string& name,const std::string& key,const std::string& value) {
    
    SQLite::Database db(ros::package::getPath("chargepal_ldb")+"/chargepal_ldb.db",SQLite::OPEN_READWRITE|SQLite::OPEN_CREATE);
    
    std::string cart_name = name; 
    std::string column_to_update = key; 
    std::string new_value = value;                   

    std::string sql_query = "UPDATE cart_info SET " + column_to_update + " = ? WHERE cart_name = ?";
    SQLite::Statement query(db,sql_query);
    query.bind(1, new_value);
    query.bind(2, cart_name);
    query.exec(); 
    query.reset();
    return true;
    
    //ToDo: Update the database on the server, receive a handshake that the value is updated. wait until received
}

std::list<std::string> ask_free_BWS() {
    //ToDo: Ask the database for free BWS, it can be a list of strings
}

std::list<std::string> ask_free_BCS() {
    //ToDo: Ask the database for free BCS, it can be a list of strings
}
std::string read_assertLift_value(const std::string& name) {
    //ToDo: Ask the database for Assert Lift value
    }
bool read_armFree_value(const std::string& name) {
//ToDo: Ask the database if arm is free
}
    