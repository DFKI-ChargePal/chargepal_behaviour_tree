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

#include "util.h"
#include <string>
#include <vector>
#include <SQLiteCpp/SQLiteCpp.h>
using namespace BT;
using namespace std::chrono_literals;
using namespace std::chrono;

class sleep_until_charged: public BT::StatefulActionNode
{
    public:
    sleep_until_charged(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config){}

        static PortsList providedPorts(){
            return{};
        }
    // You must override the virtual function tick()
    virtual NodeStatus onStart() override
    {
        int msec = 500000;

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

class sample_node: public BT::SyncActionNode
{
    public:
    sample_node(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config){}

        static PortsList providedPorts(){
            return {};
        }
    // You must override the virtual function tick()
    BT::NodeStatus tick() override
    {
        ros::Duration(10).sleep();
        return BT::NodeStatus::FAILURE;
    }
};

int main(int argc, char **argv){

ros::init(argc, argv, "test_bt");
ros::NodeHandle nh;

BehaviorTreeFactory factory;


factory.registerNodeType<sample_node>("sample_node");

std::string package_folder = ros::package::getPath("chargepal_behaviour_tree");
factory.registerBehaviorTreeFromFile(package_folder + "/xml/Test.xml");
auto mainTree = factory.createTree("test");
BT::Groot2Publisher publisher(mainTree);
mainTree.tickWhileRunning();

ros::shutdown();
return 0;
}