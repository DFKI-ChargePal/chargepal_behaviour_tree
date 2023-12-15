#include <iostream>
#include <chrono>
#include <ros/ros.h>
#include <ros/package.h>
#include <unordered_map>
#include <string>
#include <vector>
#include <SQLiteCpp/SQLiteCpp.h>
#include "util.h"


#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "chargepal_actions/ArriveAtStationAction.h"
#include "chargepal_actions/ArriveAtHomeAction.h"
#include "chargepal_actions/PickUpChargerAction.h"
#include "chargepal_actions/PlaceChargerAction.h"
#include "chargepal_actions/PlugInAction.h"
#include "chargepal_actions/PlugOutAction.h"
#include "chargepal_actions/CallForHelpAction.h"

#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_cpp/exceptions.h"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/bt_factory.h"

using namespace BT;
using namespace std::chrono_literals;
using namespace std::chrono;
//BT::Blackboard::Ptr masterBlackboard = config().blackboard;

class isBRING_CHARGER : public  BT::ConditionNode {
    public:
        isBRING_CHARGER(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config){}

        static PortsList providedPorts(){
            return { InputPort<std::string>("message") };
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
            return { InputPort<std::string>("message") };
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
            return { InputPort<std::string>("message") };
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
            return { InputPort<std::string>("message") };
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
            return { InputPort<std::string>("message") };
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
            return { InputPort<std::string>("message") };
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
            return { InputPort<std::string>("message") };
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
            return { InputPort<std::string>("message") };
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
            return { InputPort<std::string>("message") };
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


class isSameBattery : public  BT::ConditionNode {
    public:
        isSameBattery(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config){}

        static PortsList providedPorts(){
            return { InputPort<std::string>("message") };
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
            return { InputPort<std::string>("message") };
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
            return { InputPort<std::string>("message") };
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
            return { InputPort<std::string>("message") };
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
            return { InputPort<std::string>("message") };
        }
        
        virtual NodeStatus tick() override {
            BT::Blackboard::Ptr masterBlackboard = config().blackboard;
            std::string robot_name = masterBlackboard->get<std::string>("robot_name");

            std::string assertLift = read_assertLift_value(robot_name);
            

            if (assertLift == "down"){
                std::cout << "Assert lift is down" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            else {
                std::cout << "Assert lift is up" << std::endl;
                return NodeStatus::FAILURE;
            }
            
        }
};


class isCartPlaced : public  BT::ConditionNode {
    public:
        isCartPlaced(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config){}

        static PortsList providedPorts(){
            return { InputPort<std::string>("message") };
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
            return { InputPort<std::string>("message") };
        }
    // You must override the virtual function tick()
    BT::NodeStatus tick() override
    {
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
        // When the robot is at the source station
        if (source_station == robot_location) {
            goal.target_station = target_station;

            if (goal.target_station.find("ADS")!= std::string::npos && job == "BRING_CHARGER"){
                goal.target_station = "ADSplace_" + charger;
            }
            
        }
        //When the robot is not at the source station
        else{
            goal.target_station = source_station;
            if (goal.target_station.find("ADS")!= std::string::npos && job == "BRING_CHARGER"){
                goal.target_station = "ADSpick_" + charger;
            }
        }
        
        //When goal station is BWS
        if (goal.target_station.find("BWS")!= std::string::npos) {
            if (job == "BRING_CHARGER"){
                goal.target_station = "BWSpick_" + charger;
            }
            else if (job == "STOW_CHARGER"){
                goal.target_station = "BWSplace_" + charger;
            }
        } 
        else if (goal.target_station.find("BCS")!= std::string::npos) {
            if (job == "BRING_CHARGER" || job == "STOW_CHARGER"){
                goal.target_station = "BCSpick_" + charger;
            }
            else if (job == "RECHARGE_CHARGER"){
                goal.target_station = "BCSplace_" + charger;
            }
        }
        else if (goal.target_station.find("ADS")!= std::string::npos) {
            if (job == "RECHARGE_CHARGER" || job == "STOW_CHARGER"){
                goal.target_station = "ADSpick_" + charger;
            }
        }
        
        aas.sendGoal(goal);
        set_robot_value(robot_name,"ongoing_action", "arrive_at_station");
        bool aas_action = aas.waitForResult(ros::Duration(900.0));
        if (aas_action) {
            chargepal_actions::ArriveAtStationResult result = *aas.getResult();
            bool station_reached = result.station_reached;
            if (station_reached){
                std::string current_station = result.current_station;
                set_robot_value(robot_name,"ongoing_action", "none");
                set_robot_value(robot_name,"previous_action", "arrive_at_station");
                set_robot_value(robot_name,"robot_location", current_station);

                std::string cart_on_robot = read_robot_value(robot_name,"cart_on_robot");
                if (cart_on_robot != "none"){
                    set_cart_value(cart_on_robot,"cart_location", current_station);
                }
                return BT::NodeStatus::SUCCESS;
            }
            else {
                return BT::NodeStatus::FAILURE;
            }
        }
        else {
            return BT::NodeStatus::FAILURE;
        }    
    }
};


class go_home: public BT::SyncActionNode
{
    public:
    go_home(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return { InputPort<std::string>("message") };
        }
    // You must override the virtual function tick()
    BT::NodeStatus tick() override
    {
        actionlib::SimpleActionClient<chargepal_actions::ArriveAtHomeAction> aah("arrive_at_home", true);
        aah.waitForServer();
        chargepal_actions::ArriveAtHomeGoal goal;
        
        BT::Blackboard::Ptr masterBlackboard = config().blackboard;
        std::string robot_name = masterBlackboard->get<std::string>("robot_name");
        std::string robot_location = read_robot_value(robot_name,"robot_location");
        aah.sendGoal(goal);
        set_robot_value(robot_name,"ongoing_action", "place_charger");

        bool aah_action = aah.waitForResult(ros::Duration(900.0));
        if (aah_action) {
            chargepal_actions::ArriveAtHomeResult result = *aah.getResult();
            bool home_reached = result.home_reached;
            if (home_reached){
                set_robot_value(robot_name,"ongoing_action", "none");
                set_robot_value(robot_name,"previous_action", "go_home");
                return BT::NodeStatus::SUCCESS;
            }
            else {
                return BT::NodeStatus::FAILURE;
            }
        } 
        else {
            return BT::NodeStatus::FAILURE;
        }  
    }
};

class call_for_help: public BT::SyncActionNode
{
    public:
    call_for_help(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return { InputPort<std::string>("message") };
        }
    // You must override the virtual function tick()
    BT::NodeStatus tick() override
    {
        actionlib::SimpleActionClient<chargepal_actions::CallForHelpAction> cfh("call_for_help", true);
        chargepal_actions::CallForHelpGoal goal;
        cfh.waitForServer();
        cfh.sendGoal(goal);
    
        bool cfh_action = cfh.waitForResult(ros::Duration(900.0));
        if (cfh_action) {
            chargepal_actions::CallForHelpResult result = *cfh.getResult();
            return BT::NodeStatus::SUCCESS;
            }
 
        else {
            return BT::NodeStatus::FAILURE;
        }  
    }
};

class drop_cart: public BT::SyncActionNode
{
    public:
    drop_cart(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return { InputPort<std::string>("message") };
        }
    // You must override the virtual function tick()
    BT::NodeStatus tick() override
    {
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
            else {
                return BT::NodeStatus::FAILURE;
            }
        } 
        else {
            return BT::NodeStatus::FAILURE;
        }  
    }
};

class pickup_cart: public BT::SyncActionNode
{
    public:
    pickup_cart(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return { InputPort<std::string>("message") };
        }
    // You must override the virtual function tick()
    BT::NodeStatus tick() override
    {
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
            else {
                return BT::NodeStatus::FAILURE;
            }
        } 
        else {
            return BT::NodeStatus::FAILURE;
        }  
        
    }
};

class plugin_ADS: public BT::SyncActionNode
{
    public:
    plugin_ADS(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return { InputPort<std::string>("message") };
        }
    // You must override the virtual function tick()
    BT::NodeStatus tick() override
    {
        actionlib::SimpleActionClient<chargepal_actions::PlugInAction> pi_ads("plugin_charger_ads", true);
        pi_ads.waitForServer();
        chargepal_actions::PlugInGoal goal;
        
        BT::Blackboard::Ptr masterBlackboard = config().blackboard;
        std::string charger = masterBlackboard->get<std::string>("charger");
        std::string robot_name = masterBlackboard->get<std::string>("robot_name");
        
        pi_ads.sendGoal(goal);
        set_robot_value(robot_name,"ongoing_action", "plugin_charger_ads");
        bool pi_ads_action = pi_ads.waitForResult(ros::Duration(900.0));
        if (pi_ads_action){
            chargepal_actions::PlugInResult result = *pi_ads.getResult();
            bool plug_in = result.plug_in;
            if (plug_in){
                set_robot_value(robot_name,"ongoing_action", "none");
                set_robot_value(robot_name,"previous_action", "plugin_charger_ads");
                set_cart_value(charger,"plugged", "true");
                return BT::NodeStatus::SUCCESS;
            }
            else {
                return BT::NodeStatus::FAILURE;
            }
        } 
        else {
            return BT::NodeStatus::FAILURE;
        }  
    }
};

class plugin_BCS: public BT::SyncActionNode
{
    public:
    plugin_BCS(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return { InputPort<std::string>("message") };
        }
    // You must override the virtual function tick()
    BT::NodeStatus tick() override
    {
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
            else {
                return BT::NodeStatus::FAILURE;
            }
        } 
        else {
            return BT::NodeStatus::FAILURE;
        }  
    }
};

class plugout_ADS: public BT::SyncActionNode
{
    public:
    plugout_ADS(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return { InputPort<std::string>("message") };
        }
    // You must override the virtual function tick()
    BT::NodeStatus tick() override
    {
        actionlib::SimpleActionClient<chargepal_actions::PlugOutAction> po_ads("plugout_charger_ads", true);
        po_ads.waitForServer();
        chargepal_actions::PlugOutGoal goal;
        
        BT::Blackboard::Ptr masterBlackboard = config().blackboard;
        std::string charger = masterBlackboard->get<std::string>("charger");
        std::string robot_name = masterBlackboard->get<std::string>("robot_name");
        
        po_ads.sendGoal(goal);
        set_robot_value(robot_name,"ongoing_action", "plugout_charger_ads");
        bool po_ads_action = po_ads.waitForResult(ros::Duration(900.0));
        if (po_ads_action){
            chargepal_actions::PlugOutResult result = *po_ads.getResult();
            bool plug_out = result.plug_out;
            if (plug_out){
                set_robot_value(robot_name,"ongoing_action", "none");
                set_robot_value(robot_name,"previous_action", "plugout_charger_ads");
                set_cart_value(charger,"plugged", "false");
                return BT::NodeStatus::SUCCESS;
            }
            else {
                return BT::NodeStatus::FAILURE;
            }
        } 
        else {
            return BT::NodeStatus::FAILURE;
        }  
    }
};

class plugout_BCS: public BT::SyncActionNode
{
    public:
    plugout_BCS(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return { InputPort<std::string>("message") };
        }
    // You must override the virtual function tick()
    BT::NodeStatus tick() override
    {
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
            else {
                return BT::NodeStatus::FAILURE;
            }
        } 
        else {
            return BT::NodeStatus::FAILURE;
        }  
    }
};

class recovery_arrive_BWS: public BT::SyncActionNode
{
    public:
    recovery_arrive_BWS(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return { InputPort<std::string>("message") };
        }
    // You must override the virtual function tick()
    BT::NodeStatus tick() override
    {
        BT::Blackboard::Ptr masterBlackboard = config().blackboard;
        std::string robot_name = masterBlackboard->get<std::string>("robot_name");
        int retry_attempt = 0;
        actionlib::SimpleActionClient<chargepal_actions::ArriveAtStationAction> aas("arrive_at_station", true);
        
        std::cout << "Ask for free BWS" << std::endl;

        std::string free_bws = ask_free_BWS();

        while (free_bws != "none" || free_bws != "") {
            while (retry_attempt < 3) {
                aas.waitForServer();
                chargepal_actions::ArriveAtStationGoal goal;
                goal.target_station = free_bws;
                aas.sendGoal(goal);
                set_robot_value(robot_name,"ongoing_action", "recovery_arrive_BWS");
                bool aas_action = aas.waitForResult(ros::Duration(900.0));
                if (aas_action) {
                    chargepal_actions::ArriveAtStationResult result = *aas.getResult();
                    bool station_reached = result.station_reached;
                    if (station_reached){
                        std::string current_station = result.current_station;
                        set_robot_value(robot_name,"ongoing_action", "none");
                        set_robot_value(robot_name,"previous_action", "recovery_arrive_BWS");
                        set_robot_value(robot_name,"robot_location", current_station);

                        std::string cart_on_robot = read_robot_value(robot_name,"cart_on_robot");
                        if (cart_on_robot != "none"){
                            set_cart_value(cart_on_robot,"cart_location", current_station);
                        }
                        return BT::NodeStatus::SUCCESS;
                    }
                    else {
                        retry_attempt += 1 ;
                        bool delete_mission = delete_mir_mission_queue();
                        
                        if (delete_mission == false) {
                                ROS_ERROR("mir_delete_mission service failed");
                            } 
                    }
                }
                else {
                    ROS_ERROR("Failed waiting for arrive_at_station result");
                } 
                free_bws = ask_free_BWS();
            } 
        }
        return BT::NodeStatus::FAILURE;
        ROS_ERROR("No free BWS left");
            
    }
       
};

class recovery_arrive_BCS: public BT::SyncActionNode
{
    public:
    recovery_arrive_BCS(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return { InputPort<std::string>("message") };
        }
    // You must override the virtual function tick()
    BT::NodeStatus tick() override
    {
        BT::Blackboard::Ptr masterBlackboard = config().blackboard;
        std::string robot_name = masterBlackboard->get<std::string>("robot_name");
        int retry_attempt = 0;
        actionlib::SimpleActionClient<chargepal_actions::ArriveAtStationAction> aas("arrive_at_station", true);
        
        std::cout << "Ask for free BCS" << std::endl;
        
        std::string free_bcs = ask_free_BCS();
        
        while (free_bcs != "none" || free_bcs != "") {
            while (retry_attempt < 3) {
                aas.waitForServer();
                chargepal_actions::ArriveAtStationGoal goal;
                goal.target_station = free_bcs;
                aas.sendGoal(goal);
                set_robot_value(robot_name,"ongoing_action", "recovery_arrive_BCS");
                bool aas_action = aas.waitForResult(ros::Duration(900.0));
                if (aas_action) {
                    chargepal_actions::ArriveAtStationResult result = *aas.getResult();
                    bool station_reached = result.station_reached;
                    if (station_reached){
                        std::string current_station = result.current_station;
                        set_robot_value(robot_name,"ongoing_action", "none");
                        set_robot_value(robot_name,"previous_action", "recovery_arrive_BCS");
                        set_robot_value(robot_name,"robot_location", current_station);

                        std::string cart_on_robot = read_robot_value(robot_name,"cart_on_robot");
                        if (cart_on_robot != "none"){
                            set_cart_value(cart_on_robot,"cart_location", current_station);
                        }
                        return BT::NodeStatus::SUCCESS;
                    }
                    else {
                        retry_attempt += 1 ;
                        bool delete_mission = delete_mir_mission_queue();
                        
                        if (delete_mission == false) {
                                ROS_ERROR("mir_delete_mission service failed");
                            } 
                    }
                }
                else {
                    ROS_ERROR("Failed waiting for arrive_at_station result");
                }
            
                free_bcs = ask_free_BCS();
            } 
        }
        return BT::NodeStatus::FAILURE;
        ROS_ERROR("No free BCS left");
            
    }
       
};



class set_arm_free_BCS: public BT::SyncActionNode
{
    public:
    set_arm_free_BCS(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return { InputPort<std::string>("message") };
        }
    // You must override the virtual function tick()
    BT::NodeStatus tick() override
    {
        std::cout << "set arm free at BCS" << std::endl;
        std::this_thread::sleep_for(3s);
        return BT::NodeStatus::SUCCESS;
    }
};

class set_arm_free_ADS: public BT::SyncActionNode
{
    public:
    set_arm_free_ADS(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return { InputPort<std::string>("message") };
        }
    // You must override the virtual function tick()
    BT::NodeStatus tick() override
    {
        std::cout << "set arm free at ADS" << std::endl;
        std::this_thread::sleep_for(3s);
        return BT::NodeStatus::SUCCESS;
    }
};

class sleep_until_charged: public BT::StatefulActionNode
{
    public:
    sleep_until_charged(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config){}

        static PortsList providedPorts(){
            return{ BT::InputPort<int>("msec") };
        }
    // You must override the virtual function tick()
    virtual NodeStatus onStart() override
    {
        
        BT::Blackboard::Ptr masterBlackboard = config().blackboard;
        std::string robot_name = masterBlackboard->get<std::string>("robot_name");
        std::string cart = read_robot_value(robot_name,"cart_on_robot");
        //ToDo:get input and convert into milli seconds
        int msec = get_operation_time(cart);

        if( msec <= 0 ) {
            // No need to go into the RUNNING state
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
      // nothing to do here...
      std::cout << "SleepNode interrupted" << std::endl;
    }
    
    private:
        system_clock::time_point deadline_;
};


int main(int argc, char **argv){

ros::init(argc, argv, "chargepal_bt");
ros::NodeHandle nh;

/* ros::ServiceClient client = nh.serviceClient<chargepal_fake_planning::get_job>("chargepal_planner/get_job");
chargepal_fake_planning::get_job srv


if (client.call(srv))
{
    masterBlackboard->set("job_type",srv.response.job[0]);
    masterBlackboard->set("robot_name",srv.response.job[1]);
    masterBlackboard->set("charger",srv.response.job[2]);
    masterBlackboard->set("source_station",srv.response.job[3]);
    masterBlackboard->set("target_station",srv.response.job[4]);;
}
else
{
  ROS_ERROR("Failed to call service get job");
  return 1;
}
*/

BehaviorTreeFactory factory;
BT::Blackboard::Ptr masterBlackboard = BT::Blackboard::create();
//std::map<std::string,std::string> job = fetch_job()
// fake job
masterBlackboard->set("job_type","BRING_CHARGER");
masterBlackboard->set("robot_name","Chargepal1");
masterBlackboard->set("charger","BAT_1");
masterBlackboard->set("source_station","ADS_1");
masterBlackboard->set("target_station","ADS_2");

factory.registerNodeType<isBRING_CHARGER>("isBRING_CHARGER");
factory.registerNodeType<isRECHARGE_CHARGER>("isRECHARGE_CHARGER");
factory.registerNodeType<isRECHARGE_SELF>("isRECHARGE_SELF");
factory.registerNodeType<isSTOW_CHARGER>("isSTOW_CHARGER");

factory.registerNodeType<isRobotAtADSorBCSorBWS>("isRobotAtADSorBCSorBWS");
factory.registerNodeType<isRobotAtRBS>("isRobotAtRBS");
factory.registerNodeType<isRobotAtBWS>("isRobotAtBWS");
factory.registerNodeType<isRobotAtBCS>("isRobotAtBCS");
factory.registerNodeType<isRobotAtADS>("isRobotAtADS");

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
factory.registerNodeType<pickup_cart>("plugin_BCS");
factory.registerNodeType<plugout_ADS>("plugout_ADS");
factory.registerNodeType<plugout_BCS>("plugout_BCS");
factory.registerNodeType<recovery_arrive_BWS>("recovery_arrive_BWS");
factory.registerNodeType<recovery_arrive_BCS>("recovery_arrive_BCS");
factory.registerNodeType<sleep_until_charged>("sleep_until_charged");
factory.registerNodeType<set_arm_free_BCS>("set_arm_free_BCS");
factory.registerNodeType<set_arm_free_ADS>("set_arm_free_ADS");



std::string package_folder = ros::package::getPath("chargepal_behaviour_tree");

factory.registerBehaviorTreeFromFile(package_folder + "/xml/Main.xml");
factory.registerBehaviorTreeFromFile(package_folder + "/xml/Jobs.xml");
factory.registerBehaviorTreeFromFile(package_folder + "/xml/Actionsets.xml");
factory.registerBehaviorTreeFromFile(package_folder + "/xml/Actions.xml");
factory.registerBehaviorTreeFromFile(package_folder + "/xml/Recovery.xml");


BT::Tree mainTree = factory.createTree("main",masterBlackboard);

BT::NodeStatus status = mainTree.rootNode()->executeTick();

if (status == BT::NodeStatus::SUCCESS) {
        ROS_INFO("Behavior Tree tick succeeded.");
    } else {
        ROS_ERROR("Behavior Tree tick failed.");
    }

    // Exit the ROS node
    ros::shutdown();
    return 0;
}