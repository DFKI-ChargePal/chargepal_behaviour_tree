#include <iostream>
#include <chrono>
#include <ros/ros.h>
#include <ros/package.h>
#include <unordered_map>
#include <string>
#include <vector>
#include <SQLiteCpp/SQLiteCpp.h>
#include "util.h"
#include<nlohmann/json.hpp>
#include <cctype>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "chargepal_actions/ArriveAtStationAction.h"
#include "chargepal_actions/ArriveAtHomeAction.h"
#include "chargepal_actions/PickUpChargerAction.h"
#include "chargepal_actions/PlaceChargerAction.h"
#include "chargepal_actions/DisconnectPlugFromCarAction.h"
#include "chargepal_actions/ConnectPlugToCarAction.h"
#include "chargepal_actions/PlugInAction.h"
#include "chargepal_actions/PlugOutAction.h"
#include "chargepal_actions/CallForHelpAction.h"

#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_cpp/exceptions.h"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/decorators/delay_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_observer.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/controls/reactive_fallback.h"

using namespace BT;
using namespace std::chrono_literals;
using namespace std::chrono;
using json = nlohmann::json;


class isBRING_CHARGER : public  BT::ConditionNode {
    public:
        isBRING_CHARGER(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }

        virtual NodeStatus tick() override {
            BT::Blackboard::Ptr masterBlackboard = config().blackboard;
            std::string job = masterBlackboard->get<std::string>("job_type");
            std::string robot_name = masterBlackboard->get<std::string>("robot_name");
            
            if(job == "BRING_CHARGER"){
                std::cout << "BRING CHARGER requested" << std::endl;
                set_robot_value(robot_name,"current_job", "BRING_CHARGER");
                return NodeStatus::SUCCESS;
            }
            else{
                return NodeStatus::FAILURE;
            }
            
        }

};

class isRECHARGE_CHARGER : public  BT::ConditionNode {
    public:
        isRECHARGE_CHARGER(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
        
        virtual NodeStatus tick() override {
            BT::Blackboard::Ptr masterBlackboard = config().blackboard;
            std::string robot_name = masterBlackboard->get<std::string>("robot_name");
            std::string job = masterBlackboard->get<std::string>("job_type");
            if(job == "RECHARGE_CHARGER"){
                set_robot_value(robot_name,"current_job", "RECHARGE_CHARGER");
                return NodeStatus::SUCCESS;
            }
            else{
                return NodeStatus::FAILURE;
            }
        }
};

class isRECHARGE_SELF : public  BT::ConditionNode {
    public:
        isRECHARGE_SELF(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
        
        virtual NodeStatus tick() override {
            BT::Blackboard::Ptr masterBlackboard = config().blackboard;
            std::string robot_name = masterBlackboard->get<std::string>("robot_name");
            std::string job = masterBlackboard->get<std::string>("job_type");
            if(job == "RECHARGE_SELF"){
                set_robot_value(robot_name,"current_job", "RECHARGE_SELF");
                return NodeStatus::SUCCESS;
            }
            else{
                return NodeStatus::FAILURE;
            }  
        }
};

class isSTOW_CHARGER : public  BT::ConditionNode {
    public:
        isSTOW_CHARGER(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
        
        virtual NodeStatus tick() override {
            BT::Blackboard::Ptr masterBlackboard = config().blackboard;
            std::string robot_name = masterBlackboard->get<std::string>("robot_name");
            std::string job = masterBlackboard->get<std::string>("job_type");
            if(job == "STOW_CHARGER"){
                set_robot_value(robot_name,"current_job", "STOW_CHARGER");
                return NodeStatus::SUCCESS;
            }
            else{
                return NodeStatus::FAILURE;
            }    
        }
};


class isRobotAtADSorBCSorBWS : public  BT::ConditionNode {
    public:
        isRobotAtADSorBCSorBWS(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
        
        virtual NodeStatus tick() override {
            BT::Blackboard::Ptr masterBlackboard = config().blackboard;
            std::string robot_name = masterBlackboard->get<std::string>("robot_name");

            std::string location = read_robot_value(robot_name,"robot_location"); 
            if (location.find("ADS") != std::string::npos || location.find("BCS")!= std::string::npos || location.find("BWS")!= std::string::npos){
                std::cout << "Robot location is ADS or BCS" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            else {
                std::cout << "Robot location is not ADS or BCS or BWS" << std::endl;
                return NodeStatus::FAILURE;
            }

            
        }
};

class isRobotAtRBS : public  BT::ConditionNode {
    public:
        isRobotAtRBS(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
        
        virtual NodeStatus tick() override {
            BT::Blackboard::Ptr masterBlackboard = config().blackboard;
            std::string robot_name = masterBlackboard->get<std::string>("robot_name");

            std::string location = read_robot_value(robot_name,"robot_location"); 
            if (location.find("RBS") != std::string::npos) {
                std::cout << "Robot location is at RBS" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            else {
                std::cout << "Robot location is not at RBS" << std::endl;
                return NodeStatus::FAILURE;
            }   
        }
};

class isRobotAtBWS : public  BT::ConditionNode {
    public:
        isRobotAtBWS(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
        
        virtual NodeStatus tick() override {
            BT::Blackboard::Ptr masterBlackboard = config().blackboard;
            std::string robot_name = masterBlackboard->get<std::string>("robot_name");

            std::string location = read_robot_value(robot_name,"robot_location"); 
            if (location.find("BWS") != std::string::npos) {
                std::cout << "Robot location is at BWS" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            else {
                std::cout << "Robot location is not at BWS" << std::endl;
                return NodeStatus::FAILURE;
            }   
        }
};

class isRobotAtBCS : public  BT::ConditionNode {
    public:
        isRobotAtBCS(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
        
        virtual NodeStatus tick() override {
            BT::Blackboard::Ptr masterBlackboard = config().blackboard;
            std::string robot_name = masterBlackboard->get<std::string>("robot_name");

            std::string location = read_robot_value(robot_name,"robot_location");
            if (location.find("BCS") != std::string::npos) {
                std::cout << "Robot location is at BCS" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            else {
                std::cout << "Robot location is not at BCS" << std::endl;
                return NodeStatus::FAILURE;
            } 
        }
};

class isRobotAtADS : public  BT::ConditionNode {
    public:
        isRobotAtADS(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
        
        virtual NodeStatus tick() override {
            BT::Blackboard::Ptr masterBlackboard = config().blackboard;
            std::string robot_name = masterBlackboard->get<std::string>("robot_name");

            std::string location = read_robot_value(robot_name,"robot_location");
            if (location.find("ADS") != std::string::npos) {
                std::cout << "Robot location is at ADS" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            else {
                std::cout << "Robot location is not at ADS" << std::endl;
                return NodeStatus::FAILURE;
            }    
        }
};

class isArmFree : public  BT::ConditionNode {
    public:
        isArmFree(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
        
        virtual NodeStatus tick() override {
            BT::Blackboard::Ptr masterBlackboard = config().blackboard;
            std::string robot_name = masterBlackboard->get<std::string>("robot_name");
            std::cout << "Enter y/n if the arm is free  ";
            char userInput;
            std::cin >> userInput;
            if (userInput == 'y'){
                return BT::NodeStatus::SUCCESS;}
            else if (userInput == 'n'){
                return BT::NodeStatus::FAILURE;}
            
        }
};

class isSameBattery : public  BT::ConditionNode {
    public:
        isSameBattery(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
        
        virtual NodeStatus tick() override {
            BT::Blackboard::Ptr masterBlackboard = config().blackboard;
            std::string job_charger = masterBlackboard->get<std::string>("charger");
            std::string robot_name = masterBlackboard->get<std::string>("robot_name");
            std::string cart_on_robot = read_robot_value(robot_name,"cart_on_robot");
            
            if (job_charger == cart_on_robot){
                std::cout << "Same battery requested" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            else {
                std::cout << "Same battery not requested" << std::endl;
                return NodeStatus::FAILURE;
            }   
        }
};



class isDifferentBattery : public  BT::ConditionNode {
    public:
        isDifferentBattery(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
        
        virtual NodeStatus tick() override {
            BT::Blackboard::Ptr masterBlackboard = config().blackboard;
            std::string job_charger = masterBlackboard->get<std::string>("charger");
            std::string robot_name = masterBlackboard->get<std::string>("robot_name");
            std::string cart_on_robot = read_robot_value(robot_name,"cart_on_robot");
            
            if (job_charger != cart_on_robot){
                std::cout << "Different battery requested" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            else {
                std::cout << "Different battery not requested" << std::endl;
                return NodeStatus::FAILURE;
            }    
        }
};

class isbattery_ADS_BCS : public  BT::ConditionNode {
    public:
        isbattery_ADS_BCS(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
        
        virtual NodeStatus tick() override {
            BT::Blackboard::Ptr masterBlackboard = config().blackboard;
            std::string source_station = masterBlackboard->get<std::string>("source_station");
            if (source_station.find("ADS") != std::string::npos || source_station.find("BCS")!= std::string::npos){
                std::cout << "Requested battery is at ADS or BCS" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            else {
                std::cout << "Requested battery is not at ADS or BCS" << std::endl;
                return NodeStatus::FAILURE;
            }    
        }
};

class isbattery_BWS : public  BT::ConditionNode {
    public:
        isbattery_BWS(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
        
        virtual NodeStatus tick() override {
            BT::Blackboard::Ptr masterBlackboard = config().blackboard;
            std::string source_station = masterBlackboard->get<std::string>("source_station");
            if (source_station.find("BWS") != std::string::npos){
                std::cout << "Requested battery is at BWS" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            else {
                std::cout << "Requested battery is not at BWS" << std::endl;
                return NodeStatus::FAILURE;
            }
            
        }
};

class askAssertLift : public  BT::ConditionNode {
    public:
        askAssertLift(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
        
        virtual NodeStatus tick() override {
            BT::Blackboard::Ptr masterBlackboard = config().blackboard;
            std::string robot_name = masterBlackboard->get<std::string>("robot_name");

            std::string assertLift = read_assertLift_value(robot_name);

            std::cout << "Enter y/n if the assert lift is down  ";
            char userInput;
            std::cin >> userInput;
            if (userInput == 'y'){
                return BT::NodeStatus::SUCCESS;}
            else if (userInput == 'n'){
                return BT::NodeStatus::FAILURE;}
            
            /*
            if (assertLift == "down"){
                std::cout << "Assert lift is down" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            else {
                std::cout << "Assert lift is up" << std::endl;
                return NodeStatus::FAILURE;
            }*/
            
        }
};


class isCartPlaced : public  BT::ConditionNode {
    public:
        isCartPlaced(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
        
        virtual NodeStatus tick() override {
            BT::Blackboard::Ptr masterBlackboard = config().blackboard;
            std::string cart_name = masterBlackboard->get<std::string>("charger");
            std::string robot_name = masterBlackboard->get<std::string>("robot_name");

            std::string cart_on_robot = read_robot_value(robot_name,"cart_on_robot");
            std::string robot_on_cart = read_cart_value(cart_name,"robot_on_cart");

            if (cart_on_robot == robot_on_cart && cart_on_robot == "none"){
                std::cout << "Cart is placed" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            else {
                std::cout << "Cart is not placed" << std::endl;
                return NodeStatus::FAILURE;
            }
            
        }
};

class arrive_at_station: public BT::SyncActionNode
{
    public:
    arrive_at_station(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
    
    BT::NodeStatus tick() override
    {
        int retry_attempt = 0;
        bool retry_flag = true;
        actionlib::SimpleActionClient<chargepal_actions::ArriveAtStationAction> aas("arrive_at_station", true);
        aas.waitForServer();
        chargepal_actions::ArriveAtStationGoal goal;

        BT::Blackboard::Ptr masterBlackboard = config().blackboard;
        std::string job = masterBlackboard->get<std::string>("job_type");
        std::string charger = masterBlackboard->get<std::string>("charger");
        std::string robot_name = masterBlackboard->get<std::string>("robot_name");
        std::string source_station = masterBlackboard->get<std::string>("source_station");
        std::string target_station = masterBlackboard->get<std::string>("target_station");
        std::string robot_location = read_robot_value(robot_name,"robot_location");
        
        if (robot_location == target_station){
            goal.target_station = target_station;

        }
        // When the robot is at the source station
        else if (robot_location.find(source_station) !=std::string::npos) {
            goal.target_station = target_station;

            if (goal.target_station.find("ADS")!= std::string::npos && job == "BRING_CHARGER"){
                goal.target_station = target_station;
            }
            
        }
        //When the robot is not at the source station
        else{
            goal.target_station = source_station;
            if (goal.target_station.find("ADS")!= std::string::npos && job == "BRING_CHARGER"){
                goal.target_station = source_station +"_pick";
            }
        }
        
        //When goal station is BWS
        if (goal.target_station.find("BWS")!= std::string::npos) {
            if (job == "BRING_CHARGER"){
                goal.target_station = source_station + "_pick";//"BWSpick_" + charger;
            }
            else if (job == "STOW_CHARGER"){
                goal.target_station = target_station; //"BWSplace_" + charger;
            }
        } 
        else if (goal.target_station.find("BCS")!= std::string::npos) {
            if (job == "BRING_CHARGER" || job == "STOW_CHARGER"){
                goal.target_station = source_station + "_pick"; //"BCSpick_" + charger;
            }
            else if (job == "RECHARGE_CHARGER"){
                goal.target_station =  target_station; //"BCSplace_" + charger;
            }
        }
        else if (goal.target_station.find("ADS")!= std::string::npos) {
            if (job == "RECHARGE_CHARGER" || job == "STOW_CHARGER"){
                goal.target_station = source_station + "_pick" ;//"ADSpick_" + charger;
            }
        }
        
        aas.sendGoal(goal);
        set_robot_value(robot_name,"ongoing_action", "arrive_at_station_"+goal.target_station);
        bool aas_action = aas.waitForResult(ros::Duration(900.0));
        if (aas_action) {
            chargepal_actions::ArriveAtStationResult result = *aas.getResult();
            bool station_reached = result.station_reached;

            if (station_reached){
                std::string current_station = result.current_station;
                set_robot_value(robot_name,"ongoing_action", "none");
                set_robot_value(robot_name,"previous_action", "arrive_at_station_"+goal.target_station);
                set_robot_value(robot_name,"robot_location", current_station);

                std::string cart_on_robot = read_robot_value(robot_name,"cart_on_robot");
                if (cart_on_robot != "none"){
                    set_cart_value(cart_on_robot,"cart_location", current_station);
                }
                return BT::NodeStatus::SUCCESS;
            }
        }
        
        set_robot_value(robot_name,"ongoing_action", "none");
        set_robot_value(robot_name,"previous_action", "arrive_at_station_"+goal.target_station+"_failure");
        set_robot_value(robot_name,"robot_location", robot_location);
        return BT::NodeStatus::FAILURE;  
    }
};


class go_home: public BT::SyncActionNode
{
    public:
    go_home(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
    
    BT::NodeStatus tick() override
    {
        int retry_attempt = 0;
        bool retry_flag = true;
        actionlib::SimpleActionClient<chargepal_actions::ArriveAtHomeAction> aah("arrive_at_home", true);
        aah.waitForServer();
        chargepal_actions::ArriveAtHomeGoal goal;
        
        BT::Blackboard::Ptr masterBlackboard = config().blackboard;
        std::string robot_name = masterBlackboard->get<std::string>("robot_name");
        std::string robot_location = read_robot_value(robot_name,"robot_location");
        
        for (char c : robot_name){
            if (std::isdigit(c)){
                std::string robot_number = std::string(1,c);
                goal.target_station = "RBS_" + robot_number;
            }
        }
        
        aah.sendGoal(goal);
        set_robot_value(robot_name,"ongoing_action", "go_home");

        bool aah_action = aah.waitForResult(ros::Duration(900.0));
        if (aah_action) {
            chargepal_actions::ArriveAtHomeResult result = *aah.getResult();
            bool home_reached = result.station_reached;
            if (home_reached){
                std::string current_station = result.current_station;
                set_robot_value(robot_name,"ongoing_action", "none");
                set_robot_value(robot_name,"previous_action", "go_home");
                set_robot_value(robot_name,"robot_location", current_station);
                return BT::NodeStatus::SUCCESS;
            }
        }
    
        set_robot_value(robot_name,"ongoing_action", "none");
        set_robot_value(robot_name,"previous_action", "go_home_failure");
        set_robot_value(robot_name,"robot_location", robot_location);
        return BT::NodeStatus::FAILURE; 
    }
};


class call_for_help: public BT::SyncActionNode
{
    public:
    call_for_help(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
    
    BT::NodeStatus tick() override
    {
        int retry_attempt = 0;
        bool retry_flag = true;
        actionlib::SimpleActionClient<chargepal_actions::CallForHelpAction> cfh("call_for_help", true);
        chargepal_actions::CallForHelpGoal goal;
        cfh.waitForServer();
        
        cfh.sendGoal(goal);
    
        bool cfh_action = cfh.waitForResult(ros::Duration(900.0));
        if (cfh_action) {
            chargepal_actions::CallForHelpResult result = *cfh.getResult();
            return BT::NodeStatus::SUCCESS;
        }
        
        return BT::NodeStatus::FAILURE;
    }
};

class drop_cart: public BT::SyncActionNode
{
    public:
    drop_cart(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
    
    BT::NodeStatus tick() override
    {
        int retry_attempt = 0;
        bool retry_flag = true;
        actionlib::SimpleActionClient<chargepal_actions::PlaceChargerAction> plc("place_charger", true);
        plc.waitForServer();
        chargepal_actions::PlaceChargerGoal goal;
        
        BT::Blackboard::Ptr masterBlackboard = config().blackboard;
        std::string charger = masterBlackboard->get<std::string>("charger");
        std::string robot_name = masterBlackboard->get<std::string>("robot_name");
        std::string robot_location = read_robot_value(robot_name,"robot_location");
        
        goal.charger_name = charger;
        
        plc.sendGoal(goal);
        set_robot_value(robot_name,"ongoing_action", "place_charger");

        bool plc_action = plc.waitForResult(ros::Duration(900.0));
        if (plc_action) {
            chargepal_actions::PlaceChargerResult result = *plc.getResult();
            bool charger_placed = result.charger_placed;
            if (charger_placed){
                set_robot_value(robot_name,"cart_on_robot", "none");
                set_robot_value(robot_name,"ongoing_action", "none");
                set_robot_value(robot_name,"previous_action", "place_charger");
                set_cart_value(charger,"cart_location", robot_location);
                set_cart_value(charger,"robot_on_cart", "none");
                return BT::NodeStatus::SUCCESS;
            }
        }
        set_robot_value(robot_name,"ongoing_action", "none");
        set_robot_value(robot_name,"previous_action", "place_charger_failure");
        return BT::NodeStatus::FAILURE;     
    }
};
            

class pickup_cart: public BT::SyncActionNode
{
    public:
    pickup_cart(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
    
    BT::NodeStatus tick() override
    {
        int retry_attempt = 0;
        bool retry_flag = true;
        actionlib::SimpleActionClient<chargepal_actions::PickUpChargerAction> puc("pick_up_charger", true);
        puc.waitForServer();
        chargepal_actions::PickUpChargerGoal goal;
        
        BT::Blackboard::Ptr masterBlackboard = config().blackboard;
        std::string charger = masterBlackboard->get<std::string>("charger");
        std::string robot_name = masterBlackboard->get<std::string>("robot_name");
        std::string robot_location = read_robot_value(robot_name,"robot_location");

        goal.charger_name = charger;
 
        puc.sendGoal(goal);
        set_robot_value(robot_name,"ongoing_action", "pickup_charger");

        bool puc_action = puc.waitForResult(ros::Duration(900.0));
        if (puc_action) {
            chargepal_actions::PickUpChargerResult result = *puc.getResult();
            bool charger_picked = result.charger_picked;
            if (charger_picked){
                set_robot_value(robot_name,"cart_on_robot", charger);
                set_robot_value(robot_name,"ongoing_action", "none");
                set_robot_value(robot_name,"previous_action", "pickup_charger");
                set_cart_value(charger,"cart_location", robot_location);
                set_cart_value(charger,"robot_on_cart", robot_name);
                return BT::NodeStatus::SUCCESS;
            }
        }
        set_robot_value(robot_name,"ongoing_action", "none");
        set_robot_value(robot_name,"previous_action", "pickup_charger_failure");
        return BT::NodeStatus::FAILURE;     
    }
};
            

class plugin_ADS: public BT::SyncActionNode
{
    public:
    plugin_ADS(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
    
    BT::NodeStatus tick() override
    {
        int retry_attempt = 0;
        bool retry_flag = true;
        actionlib::SimpleActionClient<chargepal_actions::ConnectPlugToCarAction> pi_ads("connect_to_car", true);
        pi_ads.waitForServer();
        chargepal_actions::ConnectPlugToCarGoal goal;
        
        BT::Blackboard::Ptr masterBlackboard = config().blackboard;
        std::string charger = masterBlackboard->get<std::string>("charger");
        std::string robot_name = masterBlackboard->get<std::string>("robot_name");
        

        pi_ads.sendGoal(goal);
        set_robot_value(robot_name,"ongoing_action", "plugin_charger_ads");
        bool pi_ads_action = pi_ads.waitForResult(ros::Duration(900.0));
        if (pi_ads_action){
            chargepal_actions::ConnectPlugToCarResult result = *pi_ads.getResult();
            bool plug_in = result.connect_to_car;
            if (plug_in){
                set_robot_value(robot_name,"ongoing_action", "none");
                set_robot_value(robot_name,"previous_action", "plugin_charger_ads");
                set_cart_value(charger,"plugged", "true");
                return BT::NodeStatus::SUCCESS;
            }
        }
                
        set_robot_value(robot_name,"ongoing_action", "none");
        set_robot_value(robot_name,"previous_action", "plugin_charger_ads_failure"); 
        return BT::NodeStatus::FAILURE;     
    }
};
            

class plugin_BCS: public BT::SyncActionNode
{
    public:
    plugin_BCS(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
    
    BT::NodeStatus tick() override
    {
        int retry_attempt = 0;
        bool retry_flag = true;
        actionlib::SimpleActionClient<chargepal_actions::PlugInAction> pi_bcs("plugin_charger_bcs", true);
        pi_bcs.waitForServer();
        chargepal_actions::PlugInGoal goal;
        
        BT::Blackboard::Ptr masterBlackboard = config().blackboard;
        std::string charger = masterBlackboard->get<std::string>("charger");
        std::string robot_name = masterBlackboard->get<std::string>("robot_name");
        
        pi_bcs.sendGoal(goal);
        set_robot_value(robot_name,"ongoing_action", "plugin_charger_bcs");
        bool pi_bcs_action = pi_bcs.waitForResult(ros::Duration(900.0));
        if (pi_bcs_action){
            chargepal_actions::PlugInResult result = *pi_bcs.getResult();
            bool plug_in = result.plug_in;
            if (plug_in){
                set_robot_value(robot_name,"ongoing_action", "none");
                set_robot_value(robot_name,"previous_action", "plugin_charger_bcs");
                set_cart_value(charger,"plugged", "true");
                return BT::NodeStatus::SUCCESS;
            }
        }
        set_robot_value(robot_name,"ongoing_action", "none");
        set_robot_value(robot_name,"previous_action", "plugin_charger_bcs_failure");
        return BT::NodeStatus::FAILURE;     
    }
};
            

class plugout_ADS: public BT::SyncActionNode
{
    public:
    plugout_ADS(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
    
    BT::NodeStatus tick() override
    {
        int retry_attempt = 0;
        bool retry_flag = true;
        actionlib::SimpleActionClient<chargepal_actions::DisconnectPlugFromCarAction> po_ads("disconnect_from_car", true);
        po_ads.waitForServer();
        chargepal_actions::DisconnectPlugFromCarGoal goal;
        
        BT::Blackboard::Ptr masterBlackboard = config().blackboard;
        std::string charger = masterBlackboard->get<std::string>("charger");
        std::string robot_name = masterBlackboard->get<std::string>("robot_name");
        
      
        po_ads.sendGoal(goal);
        set_robot_value(robot_name,"ongoing_action", "plugout_charger_ads");
        bool po_ads_action = po_ads.waitForResult(ros::Duration(900.0));
        if (po_ads_action){
            chargepal_actions::DisconnectPlugFromCarResult result = *po_ads.getResult();
            bool plug_out = result.disconnect_from_car;
            if (plug_out){
                set_robot_value(robot_name,"ongoing_action", "none");
                set_robot_value(robot_name,"previous_action", "plugout_charger_ads");
                set_cart_value(charger,"plugged", "false");
                return BT::NodeStatus::SUCCESS;
            }
        }
        set_robot_value(robot_name,"ongoing_action", "none");
        set_robot_value(robot_name,"previous_action", "plugout_charger_ads_failure");
        return BT::NodeStatus::FAILURE;    
    }
};
            

class plugout_BCS: public BT::SyncActionNode
{
    public:
    plugout_BCS(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
    
    BT::NodeStatus tick() override
    {
        int retry_attempt = 0;
        bool retry_flag = true;
        actionlib::SimpleActionClient<chargepal_actions::PlugOutAction> po_bcs("plugout_charger_bcs", true);
        po_bcs.waitForServer();
        chargepal_actions::PlugOutGoal goal;
        
        BT::Blackboard::Ptr masterBlackboard = config().blackboard;
        std::string charger = masterBlackboard->get<std::string>("charger");
        std::string robot_name = masterBlackboard->get<std::string>("robot_name");
        
        po_bcs.sendGoal(goal);
        set_robot_value(robot_name,"ongoing_action", "plugout_charger_bcs");
        bool po_bcs_action = po_bcs.waitForResult(ros::Duration(900.0));
        if (po_bcs_action){
            chargepal_actions::PlugOutResult result = *po_bcs.getResult();
            bool plug_out = result.plug_out;
            if (plug_out){
                set_robot_value(robot_name,"ongoing_action", "none");
                set_robot_value(robot_name,"previous_action", "plugout_charger_bcs");
                set_cart_value(charger,"plugged", "false");
                return BT::NodeStatus::SUCCESS;
            }
        }     
        set_robot_value(robot_name,"ongoing_action", "none");
        set_robot_value(robot_name,"previous_action", "plugout_charger_bcs_failure");  
        return BT::NodeStatus::FAILURE;     
    }
};

class recovery_arrive_BWS: public BT::SyncActionNode
{
    public:
    recovery_arrive_BWS(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
    
    BT::NodeStatus tick() override
    {
        int retry_attempt = 0;
        int ldb_connection_error_count = -1;
        bool ask_ldb = true;
        std::string free_bws = "";
        std::string connection_status = "";

        BT::Blackboard::Ptr masterBlackboard = config().blackboard;
        std::string robot_name = masterBlackboard->get<std::string>("robot_name");
        actionlib::SimpleActionClient<chargepal_actions::ArriveAtStationAction> aas("arrive_at_station", true);
        
        std::cout << "Ask for free BWS" << std::endl;

        while (connection_status != " SUCCESSFUL"){
            auto free_bws_result = ask_free_BWS(ask_ldb);
            free_bws = free_bws_result.first;
            connection_status = free_bws_result.second;
            ldb_connection_error_count ++;
            if (ldb_connection_error_count > 5){
                ask_ldb = false;
            }

        }

        while (free_bws != "none" || free_bws != "") {
            ldb_connection_error_count = 0;

            while (retry_attempt < 3) {
                aas.waitForServer();
                chargepal_actions::ArriveAtStationGoal goal;
                goal.target_station = free_bws;
                aas.sendGoal(goal);
                set_robot_value(robot_name,"ongoing_action", "recovery_arrive_"+free_bws);
                bool aas_action = aas.waitForResult(ros::Duration(900.0));
                if (aas_action) {
                    chargepal_actions::ArriveAtStationResult result = *aas.getResult();
                    bool station_reached = result.station_reached;
                    if (station_reached){
                        std::string current_station = result.current_station;
                        set_robot_value(robot_name,"ongoing_action", "none");
                        set_robot_value(robot_name,"previous_action", "recovery_arrive_"+free_bws);
                        set_robot_value(robot_name,"robot_location", current_station);

                        std::string cart_on_robot = read_robot_value(robot_name,"cart_on_robot");
                        if (cart_on_robot != "none"){
                            set_cart_value(cart_on_robot,"cart_location", current_station);
                        }
                        reset_station_blocker("BWS");
                        return BT::NodeStatus::SUCCESS;
                    }
                    else {
                        retry_attempt += 1 ;
                        bool delete_mission = delete_mission_queue();
                        
                        if (delete_mission == false) {
                                ROS_ERROR("mir_delete_mission service failed");
                            } 
                    }
                }
                else {
                    ROS_ERROR("Failed waiting for arrive_at_station result");
                } 
                
                ask_ldb = true;
                connection_status = "";
                while (connection_status != " SUCCESSFUL"){
                    auto free_bws_result = ask_free_BWS(ask_ldb);
                    free_bws = free_bws_result.first;
                    connection_status = free_bws_result.second;
                    ldb_connection_error_count ++;
                    if (ldb_connection_error_count > 5){
                        ask_ldb = false;
                    }
                }
            } 
        }
        ROS_ERROR("No free BWS left");    
        return BT::NodeStatus::FAILURE;
            
    }
       
};

class recovery_arrive_BCS: public BT::SyncActionNode
{
    public:
    recovery_arrive_BCS(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
    
    BT::NodeStatus tick() override
    {
        int retry_attempt = 0;
        int ldb_connection_error_count = -1;
        bool ask_ldb = true;
        std::string free_bcs = "";
        std::string connection_status = "";

        BT::Blackboard::Ptr masterBlackboard = config().blackboard;
        std::string robot_name = masterBlackboard->get<std::string>("robot_name");
        actionlib::SimpleActionClient<chargepal_actions::ArriveAtStationAction> aas("arrive_at_station", true);
        
        std::cout << "Ask for free BCS" << std::endl;

        while (connection_status != " SUCCESSFUL"){
            auto free_bcs_result = ask_free_BCS(ask_ldb);
            free_bcs = free_bcs_result.first;
            connection_status = free_bcs_result.second;
            ldb_connection_error_count ++;
            if (ldb_connection_error_count > 5){
                ask_ldb = false;
            }
        }
        
        while (free_bcs != "none" || free_bcs != "") {
            while (retry_attempt < 3) {
                aas.waitForServer();
                chargepal_actions::ArriveAtStationGoal goal;
                goal.target_station = free_bcs;
                aas.sendGoal(goal);
                set_robot_value(robot_name,"ongoing_action", "recovery_arrive_"+free_bcs);
                bool aas_action = aas.waitForResult(ros::Duration(900.0));
                if (aas_action) {
                    chargepal_actions::ArriveAtStationResult result = *aas.getResult();
                    bool station_reached = result.station_reached;
                    if (station_reached){
                        std::string current_station = result.current_station;
                        set_robot_value(robot_name,"ongoing_action", "none");
                        set_robot_value(robot_name,"previous_action", "recovery_arrive_"+free_bcs);
                        set_robot_value(robot_name,"robot_location", current_station);

                        std::string cart_on_robot = read_robot_value(robot_name,"cart_on_robot");
                        if (cart_on_robot != "none"){
                            set_cart_value(cart_on_robot,"cart_location", current_station);
                        }
                        reset_station_blocker("BCS");
                        return BT::NodeStatus::SUCCESS;
                    }
                    else {
                        retry_attempt += 1 ;
                        bool delete_mission = delete_mission_queue();
                        
                        if (delete_mission == false) {
                                ROS_ERROR("mir_delete_mission service failed");
                            } 
                    }
                }
                else {
                    ROS_ERROR("Failed waiting for arrive_at_station result");
                }
            
                ask_ldb = true;
                connection_status = "";
                while (connection_status != " SUCCESSFUL"){
                    auto free_bcs_result = ask_free_BCS(ask_ldb);
                    free_bcs = free_bcs_result.first;
                    connection_status = free_bcs_result.second;
                    ldb_connection_error_count ++;
                    if (ldb_connection_error_count > 5){
                        ask_ldb = false;
                    }
                }
            } 
        }
        ROS_ERROR("No free BWS left");    
        return BT::NodeStatus::FAILURE;
            
    }
       
};

class delete_mir_mission_queue: public BT::SyncActionNode
{
    public:
    delete_mir_mission_queue(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
    
    BT::NodeStatus tick() override
    {
        BT::Blackboard::Ptr masterBlackboard = config().blackboard;
        std::string robot_name = masterBlackboard->get<std::string>("robot_name");

        bool delete_mission = delete_mission_queue();
        if (delete_mission) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
        
    }
    
};

class error_count: public BT::SyncActionNode
{
    public:
    error_count(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
    
    BT::NodeStatus tick() override
    {
        BT::Blackboard::Ptr masterBlackboard = config().blackboard;
        std::string robot_name = masterBlackboard->get<std::string>("robot_name");
        std::string error_count = read_robot_value(robot_name,"error_count");
        
        int error = std::stoi(error_count);
        set_robot_value(robot_name,"error_count", std::to_string(error ++));
        return BT::NodeStatus::SUCCESS;
        
    }
    
};

class time_sleep: public BT::SyncActionNode
{
    public:
    time_sleep(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
    
    BT::NodeStatus tick() override
    {
        ros::Duration(10).sleep();
        return BT::NodeStatus::SUCCESS;
    }
};
class sleep_until_charged: public BT::StatefulActionNode
{
    public:
    sleep_until_charged(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config){}

        static PortsList providedPorts(){
            return{};
        }
    
    virtual NodeStatus onStart() override
    {
        
        BT::Blackboard::Ptr masterBlackboard = config().blackboard;
        std::string robot_name = masterBlackboard->get<std::string>("robot_name");
        std::string cart = read_robot_value(robot_name,"cart_on_robot");
        int msec = get_operation_time(cart);

        if( msec <= 0 ) {
            return BT::NodeStatus::SUCCESS;
        }
        else {
            // once the deadline is reached, we will return SUCCESS.
            deadline_ = system_clock::now() + milliseconds(msec);
            return BT::NodeStatus::RUNNING;
        }
    }

    /// method invoked by an action in the RUNNING state.
    virtual NodeStatus onRunning() override
    {
        if ( system_clock::now() >= deadline_ ) {
            return BT::NodeStatus::SUCCESS;
        }
        else {
            return BT::NodeStatus::RUNNING;
        }
    }

    void onHalted() override
    {
      std::cout << "SleepNode interrupted" << std::endl;
    }
    
    private:
        system_clock::time_point deadline_;
};


int main(int argc, char **argv){

ros::init(argc, argv, "chargepal_bt");
ros::NodeHandle nh;

BehaviorTreeFactory factory;
BT::Blackboard::Ptr masterBlackboard = BT::Blackboard::create();

factory.registerNodeType<isBRING_CHARGER>("isBRING_CHARGER");
factory.registerNodeType<isRECHARGE_CHARGER>("isRECHARGE_CHARGER");
factory.registerNodeType<isRECHARGE_SELF>("isRECHARGE_SELF");
factory.registerNodeType<isSTOW_CHARGER>("isSTOW_CHARGER");

factory.registerNodeType<isRobotAtADSorBCSorBWS>("isRobotAtADSorBCSorBWS");
factory.registerNodeType<isRobotAtRBS>("isRobotAtRBS");
factory.registerNodeType<isRobotAtBWS>("isRobotAtBWS");
factory.registerNodeType<isRobotAtBCS>("isRobotAtBCS");
factory.registerNodeType<isRobotAtADS>("isRobotAtADS");

factory.registerNodeType<isArmFree>("isArmFree");
factory.registerNodeType<isSameBattery>("isSameBattery");
factory.registerNodeType<isDifferentBattery>("isDifferentBattery");
factory.registerNodeType<isbattery_ADS_BCS>("isbattery_ADS_BCS");
factory.registerNodeType<isbattery_BWS>("isbattery_BWS");
factory.registerNodeType<askAssertLift>("askAssertLift");
factory.registerNodeType<isCartPlaced>("isCartPlaced");

factory.registerNodeType<arrive_at_station>("arrive_at_station");
factory.registerNodeType<go_home>("go_home");
factory.registerNodeType<call_for_help>("call_for_help");
factory.registerNodeType<drop_cart>("drop_cart");
factory.registerNodeType<pickup_cart>("pickup_cart");
factory.registerNodeType<plugin_ADS>("plugin_ADS");
factory.registerNodeType<plugin_BCS>("plugin_BCS");
factory.registerNodeType<plugout_ADS>("plugout_ADS");
factory.registerNodeType<plugout_BCS>("plugout_BCS");
factory.registerNodeType<recovery_arrive_BWS>("recovery_arrive_BWS");
factory.registerNodeType<recovery_arrive_BCS>("recovery_arrive_BCS");
factory.registerNodeType<sleep_until_charged>("sleep_until_charged");
factory.registerNodeType<delete_mir_mission_queue>("delete_mir_mission_queue");
factory.registerNodeType<error_count>("error_count");
factory.registerNodeType<time_sleep>("time_sleep");


std::string package_folder = ros::package::getPath("chargepal_behaviour_tree");

factory.registerBehaviorTreeFromFile(package_folder + "/xml/Main.xml");
factory.registerBehaviorTreeFromFile(package_folder + "/xml/Jobs.xml");
factory.registerBehaviorTreeFromFile(package_folder + "/xml/Actionsets.xml");
factory.registerBehaviorTreeFromFile(package_folder + "/xml/Actions.xml");
factory.registerBehaviorTreeFromFile(package_folder + "/xml/Recovery.xml");


auto mainTree = factory.createTree("main",masterBlackboard);
BT::Groot2Publisher publisher(mainTree);

json job_requested;


while (job_requested.empty()){
    std::string job_input = fetch_job();
    try{
        job_requested = json::parse(job_input);
        }
    catch(const nlohmann::json::exception& e){
        std::cerr << "Error parsing JSON: " << e.what() << std::endl;
    }

    if (job_requested.empty()){
        std::cout << "Waiting for new job";
    }

    else {

        std::string job_type = job_requested["job_type"];
        std::string robot_name = job_requested["robot_name"];
        std::string charger = job_requested["charger"];
        std::string source_station = job_requested["source_station"];
        std::string target_station = job_requested["target_station"];

        masterBlackboard->set("job_type", job_type);
        masterBlackboard->set("robot_name",robot_name);
        masterBlackboard->set("charger",charger);
        masterBlackboard->set("source_station",source_station);
        masterBlackboard->set("target_station",target_station);

        mainTree.tickWhileRunning();
        
        BT::NodeStatus status = mainTree.rootNode()->status();
        
        if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::IDLE) {
            bool job_server_update = false;
            while(job_server_update == false){
                ROS_INFO("Updating job completion to server!");
                job_server_update = update_job_monitor();
            }
            job_requested.clear();
            ROS_INFO("Job completion succeeded.");
        } 
        
        else {
            ROS_ERROR("Behavior Tree tick failed.");
            break;
        }

    }
}

// Exit the ROS node
ros::shutdown();
return 0;
}