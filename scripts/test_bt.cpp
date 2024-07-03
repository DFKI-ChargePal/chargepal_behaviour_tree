#include <iostream>
#include <chrono>
#include <ros/ros.h>
#include <ros/package.h>

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

class sample_node_1: public BT::StatefulActionNode
{
    public:
    sample_node_1(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config){}

        static PortsList providedPorts(){
            return{};
        }
    // You must override the virtual function tick()
    virtual NodeStatus onStart() override
    {
        int msec = 5000;

        if( msec <= 0 ) {
            // No need to go into the RUNNING state
            
            return BT::NodeStatus::SUCCESS;
        }
        else {
            std::cout <<"starting node 1" << std::endl;
            // once the deadline is reached, we will return SUCCESS.
            deadline_ = system_clock::now() + milliseconds(msec);
            return BT::NodeStatus::RUNNING;
        }
    }

    /// method invoked by an action in the RUNNING state.
    virtual NodeStatus onRunning() override
    {
        if ( system_clock::now() >= deadline_ ) {
            std::cout <<"finishing node 1" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        else {
            return BT::NodeStatus::RUNNING;
        }
    }

    void onHalted() override
    {
      // nothing to do here...
      
      std::cout << "SleepNode1 interrupted" << std::endl;
    }
    
    private:
        system_clock::time_point deadline_;
};
class sample_node_2: public BT::StatefulActionNode
{
    public:
    sample_node_2(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config){}

        static PortsList providedPorts(){
            return{};
        }
    // You must override the virtual function tick()
    virtual NodeStatus onStart() override
    {
        int msec = 10000;

        if( msec <= 0 ) {
            // No need to go into the RUNNING state
            return BT::NodeStatus::SUCCESS;
        }
        else {
            std::cout <<"starting node 2" << std::endl;
            // once the deadline is reached, we will return SUCCESS.
            deadline_ = system_clock::now() + milliseconds(msec);
            return BT::NodeStatus::RUNNING;
        }
    }

    /// method invoked by an action in the RUNNING state.
    virtual NodeStatus onRunning() override
    {
        if ( system_clock::now() >= deadline_ ) {
            std::cout <<"finishing node 2" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        else {
            return BT::NodeStatus::RUNNING;
        }
    }

    void onHalted() override
    {
      // nothing to do here...
      std::cout << "Contunue_running_2 interrupted" << std::endl;
     
    }
    
    private:
        system_clock::time_point deadline_;
};

class sample_node_3: public BT::SyncActionNode
{
    public:
    sample_node_3(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
    // You must override the virtual function tick()
    BT::NodeStatus tick() override
    {
        std::cout <<"starting node 3" << std::endl;
        ros::Duration(5).sleep();
        std::cout <<"finishing node 3" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};



int main(int argc, char **argv){

ros::init(argc, argv, "test_bt");
ros::NodeHandle nh;

BehaviorTreeFactory factory;


factory.registerNodeType<sample_node_1>("sample_node_1");
factory.registerNodeType<sample_node_2>("sample_node_2");
factory.registerNodeType<sample_node_3>("sample_node_3");

std::string package_folder = ros::package::getPath("chargepal_behaviour_tree");
factory.registerBehaviorTreeFromFile(package_folder + "/xml/Test.xml");
auto mainTree = factory.createTree("test");
BT::Groot2Publisher publisher(mainTree);
mainTree.tickWhileRunning();
BT::NodeStatus status = mainTree.rootNode()->status();
std::cout<< status;
ros::shutdown();
return 0;
}