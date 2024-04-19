#include "util.h"

class isBRING_CHARGER : public BT::ConditionNode {
public:
  isBRING_CHARGER(const std::string &name, const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string job = masterBlackboard->get<std::string>("job_type");
    std::string robot_name = masterBlackboard->get<std::string>("robot_name");

    if (job == "BRING_CHARGER") {
      std::cout << "BRING CHARGER requested" << std::endl;
      update_gui_config("ongoing_job", "BRING_CHARGER");
      set_robot_value(robot_name, "current_job", "BRING_CHARGER");
      return NodeStatus::SUCCESS;
    } else {
      return NodeStatus::FAILURE;
    }
  }
};

class isRECHARGE_CHARGER : public BT::ConditionNode {
public:
  isRECHARGE_CHARGER(const std::string &name,
                     const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string robot_name = masterBlackboard->get<std::string>("robot_name");
    std::string job = masterBlackboard->get<std::string>("job_type");
    if (job == "RECHARGE_CHARGER") {
      set_robot_value(robot_name, "current_job", "RECHARGE_CHARGER");
      update_gui_config("ongoing_job", "RECHARGE_CHARGER");
      return NodeStatus::SUCCESS;
    } else {
      return NodeStatus::FAILURE;
    }
  }
};

class isRECHARGE_SELF : public BT::ConditionNode {
public:
  isRECHARGE_SELF(const std::string &name, const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string robot_name = masterBlackboard->get<std::string>("robot_name");
    std::string job = masterBlackboard->get<std::string>("job_type");
    if (job == "RECHARGE_SELF") {
      set_robot_value(robot_name, "current_job", "RECHARGE_SELF");
      update_gui_config("ongoing_job", "RECHARGE_SELF");
      return NodeStatus::SUCCESS;
    } else {
      return NodeStatus::FAILURE;
    }
  }
};

class isSTOW_CHARGER : public BT::ConditionNode {
public:
  isSTOW_CHARGER(const std::string &name, const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string robot_name = masterBlackboard->get<std::string>("robot_name");
    std::string job = masterBlackboard->get<std::string>("job_type");
    if (job == "STOW_CHARGER") {
      set_robot_value(robot_name, "current_job", "STOW_CHARGER");
      update_gui_config("ongoing_job", "STOW_CHARGER");
      return NodeStatus::SUCCESS;
    } else {
      return NodeStatus::FAILURE;
    }
  }
};

class isRobotAtADSorBCSorBWS : public BT::ConditionNode {
public:
  isRobotAtADSorBCSorBWS(const std::string &name,
                         const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string robot_name = masterBlackboard->get<std::string>("robot_name");

    std::string location = read_robot_value(robot_name, "robot_location");
    if (location.find("ADS") != std::string::npos ||
        location.find("BCS") != std::string::npos ||
        location.find("BWS") != std::string::npos) {
      // std::cout << "Robot location is ADS or BCS" << std::endl;
      return BT::NodeStatus::SUCCESS;
    } else {
      // std::cout << "Robot location is not ADS or BCS or BWS" << std::endl;
      return NodeStatus::FAILURE;
    }
  }
};

class isRobotAtRBS : public BT::ConditionNode {
public:
  isRobotAtRBS(const std::string &name, const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string robot_name = masterBlackboard->get<std::string>("robot_name");

    std::string location = read_robot_value(robot_name, "robot_location");
    if (location.find("RBS") != std::string::npos) {
      // << "Robot location is at RBS" << std::endl;
      return BT::NodeStatus::SUCCESS;
    } else {
      // std::cout << "Robot location is not at RBS" << std::endl;
      return NodeStatus::FAILURE;
    }
  }
};

class isRobotAtBWS : public BT::ConditionNode {
public:
  isRobotAtBWS(const std::string &name, const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string robot_name = masterBlackboard->get<std::string>("robot_name");

    std::string location = read_robot_value(robot_name, "robot_location");
    if (location.find("BWS") != std::string::npos) {
      // std::cout << "Robot location is at BWS" << std::endl;
      return BT::NodeStatus::SUCCESS;
    } else {
      // std::cout << "Robot location is not at BWS" << std::endl;
      return NodeStatus::FAILURE;
    }
  }
};

class isRobotAtBCS : public BT::ConditionNode {
public:
  isRobotAtBCS(const std::string &name, const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string robot_name = masterBlackboard->get<std::string>("robot_name");

    std::string location = read_robot_value(robot_name, "robot_location");
    if (location.find("BCS") != std::string::npos) {
      // std::cout << "Robot location is at BCS" << std::endl;
      return BT::NodeStatus::SUCCESS;
    } else {
      // std::cout << "Robot location is not at BCS" << std::endl;
      return NodeStatus::FAILURE;
    }
  }
};

class isRobotAtADS : public BT::ConditionNode {
public:
  isRobotAtADS(const std::string &name, const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string robot_name = masterBlackboard->get<std::string>("robot_name");

    std::string location = read_robot_value(robot_name, "robot_location");
    if (location.find("ADS") != std::string::npos) {
      // std::cout << "Robot location is at ADS" << std::endl;
      return BT::NodeStatus::SUCCESS;
    } else {
      // std::cout << "Robot location is not at ADS" << std::endl;
      return NodeStatus::FAILURE;
    }
  }
};

class isArmFree : public BT::ConditionNode {
public:
  isArmFree(const std::string &name, const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string robot_name = masterBlackboard->get<std::string>("robot_name");
    std::cout << "Enter y/n if the arm is free  ";
    char userInput;
    std::cin >> userInput;
    if (userInput == 'y') {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};

class isSameBattery : public BT::ConditionNode {
public:
  isSameBattery(const std::string &name, const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string job_charger = masterBlackboard->get<std::string>("charger");
    std::string robot_name = masterBlackboard->get<std::string>("robot_name");
    std::string cart_on_robot = read_robot_value(robot_name, "cart_on_robot");

    if (job_charger == cart_on_robot) {
      // std::cout << "Same battery requested" << std::endl;
      return BT::NodeStatus::SUCCESS;
    } else {
      // std::cout << "Same battery not requested" << std::endl;
      return NodeStatus::FAILURE;
    }
  }
};

class isDifferentBattery : public BT::ConditionNode {
public:
  isDifferentBattery(const std::string &name,
                     const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string job_charger = masterBlackboard->get<std::string>("charger");
    std::string robot_name = masterBlackboard->get<std::string>("robot_name");
    std::string cart_on_robot = read_robot_value(robot_name, "cart_on_robot");

    if (job_charger != cart_on_robot) {
      // std::cout << "Different battery requested" << std::endl;
      return BT::NodeStatus::SUCCESS;
    } else {
      // std::cout << "Different battery not requested" << std::endl;
      return NodeStatus::FAILURE;
    }
  }
};

class isbattery_ADS_BCS : public BT::ConditionNode {
public:
  isbattery_ADS_BCS(const std::string &name,
                    const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string source_station =
        masterBlackboard->get<std::string>("source_station");
    if (source_station.find("ADS") != std::string::npos ||
        source_station.find("BCS") != std::string::npos) {
      // std::cout << "Requested battery is at ADS or BCS" << std::endl;
      return BT::NodeStatus::SUCCESS;
    } else {
      // std::cout << "Requested battery is not at ADS or BCS" << std::endl;
      return NodeStatus::FAILURE;
    }
  }
};

class isbattery_BWS : public BT::ConditionNode {
public:
  isbattery_BWS(const std::string &name, const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string source_station =
        masterBlackboard->get<std::string>("source_station");
    if (source_station.find("BWS") != std::string::npos) {
      // std::cout << "Requested battery is at BWS" << std::endl;
      return BT::NodeStatus::SUCCESS;
    } else {
      // std::cout << "Requested battery is not at BWS" << std::endl;
      return NodeStatus::FAILURE;
    }
  }
};

class isReady2Plugin : public BT::ConditionNode {
public:
  isReady2Plugin(const std::string &name, const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string target_station =
        masterBlackboard->get<std::string>("target_station");

    bool plugin_proceed = check_ready_to_plugin(target_station);
    if (plugin_proceed) {
      return BT::NodeStatus::SUCCESS;
    } else {
      ros::Duration(10).sleep();
      return NodeStatus::FAILURE;
    }
  }
};

class askAssertLift : public BT::ConditionNode {
public:
  askAssertLift(const std::string &name, const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string robot_name = masterBlackboard->get<std::string>("robot_name");

    std::string assertLift = read_assertLift_value(robot_name);

    if (assertLift == "down") {
      enter_log_file("Assert lift is : " + assertLift);
      return BT::NodeStatus::SUCCESS;
    } else {
      enter_log_file("Assert lift is : " + assertLift);
      return NodeStatus::FAILURE;
    }
  }
};

class isCartPlaced : public BT::ConditionNode {
public:
  isCartPlaced(const std::string &name, const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus tick() override {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string cart_name = masterBlackboard->get<std::string>("charger");
    std::string robot_name = masterBlackboard->get<std::string>("robot_name");

    std::string cart_on_robot = read_robot_value(robot_name, "cart_on_robot");
    std::string robot_on_cart = read_cart_value(cart_name, "robot_on_cart");

    if (cart_on_robot == robot_on_cart && cart_on_robot == "none") {
      // std::cout << "Cart is placed" << std::endl;
      return BT::NodeStatus::SUCCESS;
    } else {
      // std::cout << "Cart is not placed" << std::endl;
      return NodeStatus::FAILURE;
    }
  }
};

class arrive_at_station : public BT::SyncActionNode {
public:
  arrive_at_station(const std::string &name,
                    const BT::NodeConfiguration &config, std::ofstream &logFile)
      : BT::SyncActionNode(name, config), _argLog(logFile) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {

    actionlib::SimpleActionClient<chargepal_actions::ArriveAtStationAction> aas(
        "arrive_at_station", true);
    aas.waitForServer();
    chargepal_actions::ArriveAtStationGoal goal;

    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    job = masterBlackboard->get<std::string>("job_type");
    charger = masterBlackboard->get<std::string>("charger");
    robot_name = masterBlackboard->get<std::string>("robot_name");
    source_station = masterBlackboard->get<std::string>("source_station");
    target_station = masterBlackboard->get<std::string>("target_station");
    robot_location = read_robot_value(robot_name, "robot_location");

    if (robot_location == target_station) {
      goal.target_station = target_station;

    }
    // When the robot is at the source station
    else if (robot_location.find(source_station) != std::string::npos) {
      goal.target_station = target_station;
    }
    // When the robot is not at the source station
    else {
      goal.target_station = source_station;
      if (goal.target_station.find("ADS") != std::string::npos &&
          job == "BRING_CHARGER") {
        goal.target_station = source_station + "_pick";
      }
    }

    // When goal station is BWS
    if (goal.target_station.find("BWS") != std::string::npos) {
      if (job == "BRING_CHARGER") {
        goal.target_station = source_station + "_pick"; //"BWSpick_" + charger;
      } else if (job == "STOW_CHARGER") {
        goal.target_station = target_station; //"BWSplace_" + charger;
      }
    } else if (goal.target_station.find("BCS") != std::string::npos) {
      if (job == "BRING_CHARGER" || job == "STOW_CHARGER") {
        goal.target_station = source_station + "_pick"; //"BCSpick_" + charger;
      } else if (job == "RECHARGE_CHARGER") {
        goal.target_station = target_station; //"BCSplace_" + charger;
      }
    } else if (goal.target_station.find("ADS") != std::string::npos) {
      if (job == "RECHARGE_CHARGER" || job == "STOW_CHARGER") {
        goal.target_station = source_station + "_pick"; //"ADSpick_" + charger;
      }
    }

    aas.sendGoal(goal);
    set_robot_value(robot_name, "ongoing_action",
                    "arrive_at_station_" + goal.target_station);
    enter_log_file("Performing arrive_at_station_" + goal.target_station);
    update_gui_config("ongoing_action",
                      "Performing arrive_at_station_" + goal.target_station);
    bool aas_action = aas.waitForResult(ros::Duration(900.0));
    if (aas_action) {
      chargepal_actions::ArriveAtStationResult result = *aas.getResult();

      bool station_reached = result.station_reached;

      if (station_reached) {
        std::string current_station = result.current_station;
        set_robot_value(robot_name, "ongoing_action", "none");
        set_robot_value(robot_name, "previous_action",
                        "arrive_at_station_" + goal.target_station);
        set_robot_value(robot_name, "robot_location", current_station);

        std::string cart_on_robot =
            read_robot_value(robot_name, "cart_on_robot");
        if (cart_on_robot != "none") {
          set_cart_value(cart_on_robot, "cart_location", current_station);
        }
        return BT::NodeStatus::SUCCESS;
      }
      action_result = result.action_status;
    }
    enter_log_file("arrive_at_station_" + goal.target_station + " status is " +
                   action_result);
    update_gui_config("ongoing_action", "arrive_at_station_" +
                                            goal.target_station +
                                            " status is " + action_result);
    update_gui_config("error_count_arrive_at_station", "");
    set_robot_value(robot_name, "ongoing_action", "none");
    set_robot_value(robot_name, "previous_action",
                    "arrive_at_station_" + goal.target_station + "_" +
                        action_result);
    set_robot_value(robot_name, "robot_location", robot_location);
    masterBlackboard->set("failed_action",
                          "arrive_at_station_" + goal.target_station);
    // enter_log_file(std::endl;
    return BT::NodeStatus::FAILURE;
  }

private:
  std::ofstream &_argLog;

  std::string job, charger, robot_name, source_station, target_station,
      robot_location, action_result;
};

class go_home : public BT::SyncActionNode {
public:
  go_home(const std::string &name, const BT::NodeConfiguration &config,
          std::ofstream &logFile)
      : BT::SyncActionNode(name, config), _argLog(logFile) {}

  static PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {

    actionlib::SimpleActionClient<chargepal_actions::ArriveAtHomeAction> aah(
        "arrive_at_home", true);
    aah.waitForServer();

    masterBlackboard = config().blackboard;
    robot_name = masterBlackboard->get<std::string>("robot_name");
    robot_location = read_robot_value(robot_name, "robot_location");

    for (char c : robot_name) {
      if (std::isdigit(c)) {
        robot_number = std::string(1, c);
        goal.target_station = "RBS_" + robot_number;
      }
    }

    aah.sendGoal(goal);
    set_robot_value(robot_name, "ongoing_action", "go_home");
    enter_log_file("Performing arrive_at_home to " + goal.target_station);
    update_gui_config("ongoing_action",
                      "Performing arrive_at_home to " + goal.target_station);
    bool aah_action = aah.waitForResult(ros::Duration(900.0));
    if (aah_action) {
      result = *aah.getResult();
      bool home_reached = result.station_reached;
      if (home_reached) {
        std::string current_station = result.current_station;
        set_robot_value(robot_name, "ongoing_action", "none");
        set_robot_value(robot_name, "previous_action", "go_home");
        set_robot_value(robot_name, "robot_location", current_station);
        return BT::NodeStatus::SUCCESS;
      }
      action_result = result.action_status;
    }
    enter_log_file("arrive_at_home to" + goal.target_station + " status is " +
                   action_result);
    update_gui_config("ongoing_action", "arrive_at_home to" +
                                            goal.target_station +
                                            " status is " + action_result);
    update_gui_config("error_count_go_home", "");
    set_robot_value(robot_name, "ongoing_action", "none");
    set_robot_value(robot_name, "previous_action",
                    "go_home_" + goal.target_station + "_" + action_result);
    set_robot_value(robot_name, "robot_location", robot_location);
    masterBlackboard->set("failed_action", "go_home_" + goal.target_station);
    return BT::NodeStatus::FAILURE;
  }

private:
  BT::Blackboard::Ptr masterBlackboard;
  std::ofstream &_argLog;
  chargepal_actions::ArriveAtHomeGoal goal;
  chargepal_actions::ArriveAtHomeResult result;

  std::string robot_name, robot_location, action_result, robot_number;
};

class call_for_help : public BT::SyncActionNode {
public:
  call_for_help(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {}

  static PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    int retry_attempt = 0;
    update_gui_config("ongoing_action", "Calling for help...");
    actionlib::SimpleActionClient<chargepal_actions::CallForHelpAction> cfh(
        "call_for_help", true);
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

class drop_cart : public BT::SyncActionNode {
public:
  drop_cart(const std::string &name, const BT::NodeConfiguration &config,
            std::ofstream &logFile)
      : BT::SyncActionNode(name, config), _argLog(logFile) {}

  static PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {

    actionlib::SimpleActionClient<chargepal_actions::PlaceChargerAction> plc(
        "place_charger", true);
    plc.waitForServer();

    masterBlackboard = config().blackboard;
    robot_name = masterBlackboard->get<std::string>("robot_name");
    robot_location = read_robot_value(robot_name, "robot_location");
    cart_on_robot = read_robot_value(robot_name, "cart_on_robot");
    goal.charger_name = cart_on_robot;

    plc.sendGoal(goal);
    set_robot_value(robot_name, "ongoing_action", "place_charger");
    enter_log_file("Performing place_charger of " + goal.charger_name);
    update_gui_config("ongoing_action",
                      "Performing place_charger of " + goal.charger_name);
    bool plc_action = plc.waitForResult(ros::Duration(900.0));
    if (plc_action) {
      result = *plc.getResult();
      bool charger_placed = result.charger_placed;
      if (charger_placed) {
        set_robot_value(robot_name, "cart_on_robot", "none");
        set_robot_value(robot_name, "ongoing_action", "none");
        set_robot_value(robot_name, "previous_action", "place_charger");
        set_cart_value(cart_on_robot, "cart_location", robot_location);
        set_cart_value(cart_on_robot, "robot_on_cart", "none");
        return BT::NodeStatus::SUCCESS;
      }
      action_result = result.action_status;
    }
    recover_status = recover_cart("place_cart");
    if (!recover_status) {
      enter_log_file("place_charger of" + goal.charger_name +
                     " IO recover status is False");
    }
    enter_log_file("place_charger of" + goal.charger_name + " status is " +
                   action_result);
    update_gui_config("error_count_place_cart", "");
    update_gui_config("ongoing_action", "place_charger of" + goal.charger_name +
                                            " status is " + action_result);
    set_robot_value(robot_name, "ongoing_action", "none");
    set_robot_value(robot_name, "previous_action",
                    "place_charger_" + goal.charger_name + "_" + action_result);
    // enter_log_file(std::endl;
    masterBlackboard->set("failed_action",
                          "place_charger of" + goal.charger_name);
    return BT::NodeStatus::FAILURE;
  }

private:
  BT::Blackboard::Ptr masterBlackboard;
  std::ofstream &_argLog;
  bool recover_status;
  chargepal_actions::PlaceChargerGoal goal;
  chargepal_actions::PlaceChargerResult result;

  std::string cart_on_robot, robot_name, robot_location, action_result;
};

class pickup_cart : public BT::SyncActionNode {
public:
  pickup_cart(const std::string &name, const BT::NodeConfiguration &config,
              std::ofstream &logFile)
      : BT::SyncActionNode(name, config), _argLog(logFile) {}

  static PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {

    actionlib::SimpleActionClient<chargepal_actions::PickUpChargerAction> puc(
        "pick_up_charger", true);
    puc.waitForServer();

    masterBlackboard = config().blackboard;
    charger = masterBlackboard->get<std::string>("charger");
    robot_name = masterBlackboard->get<std::string>("robot_name");
    robot_location = read_robot_value(robot_name, "robot_location");

    goal.charger_name = charger;

    puc.sendGoal(goal);
    set_robot_value(robot_name, "ongoing_action", "pickup_charger");
    enter_log_file("Performing pickup_charger of " + goal.charger_name);
    update_gui_config("ongoing_action",
                      "Performing pickup_charger of " + goal.charger_name);
    bool puc_action = puc.waitForResult(ros::Duration(900.0));
    if (puc_action) {
      chargepal_actions::PickUpChargerResult result = *puc.getResult();
      bool charger_picked = result.charger_picked;
      if (charger_picked) {
        set_robot_value(robot_name, "cart_on_robot", charger);
        set_robot_value(robot_name, "ongoing_action", "none");
        set_robot_value(robot_name, "previous_action", "pickup_charger");
        set_cart_value(charger, "cart_location", robot_location);
        set_cart_value(charger, "robot_on_cart", robot_name);
        return BT::NodeStatus::SUCCESS;
      }
      action_result = result.action_status;
    }
    recover_status = recover_cart("pickup_cart");
    if (!recover_status) {
      enter_log_file("pickup_charger of" + goal.charger_name +
                     " IO recover status is False");
    }

    enter_log_file("pickup_charger of" + goal.charger_name + " status is " +
                   action_result);
    update_gui_config("ongoing_action", "pickup_charger of" +
                                            goal.charger_name + " status is " +
                                            action_result);
    update_gui_config("error_count_pickup_cart", "");
    set_robot_value(robot_name, "ongoing_action", "none");
    set_robot_value(robot_name, "previous_action", "pickup_charger_failure");
    // enter_log_file(std::endl;
    masterBlackboard->set("failed_action",
                          "pickup_charger of" + goal.charger_name);
    return BT::NodeStatus::FAILURE;
  }

private:
  BT::Blackboard::Ptr masterBlackboard;
  std::ofstream &_argLog;
  bool recover_status;
  chargepal_actions::PickUpChargerGoal goal;
  std::string charger, robot_name, robot_location, action_result;
};

class plugin_ADS : public BT::SyncActionNode {
public:
  plugin_ADS(const std::string &name, const BT::NodeConfiguration &config,
             std::ofstream &logFile)
      : BT::SyncActionNode(name, config), _argLog(logFile) {}

  static PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    ros::param::get("/sim_flag", sim_flag);
    masterBlackboard = config().blackboard;
    charger = masterBlackboard->get<std::string>("charger");
    robot_name = masterBlackboard->get<std::string>("robot_name");
    target_station = masterBlackboard->get<std::string>("target_station");
    enter_log_file("Performing plugin_ADS at " + target_station);
    update_gui_config("ongoing_action",
                      "Performing plugin_ADS at " + target_station);
    if (sim_flag) {

      set_robot_value(robot_name, "ongoing_action", "plugin_charger_ads");
      ros::Duration(10).sleep();
      set_robot_value(robot_name, "ongoing_action", "none");
      set_robot_value(robot_name, "previous_action", "plugin_charger_ads");
      set_cart_value(charger, "plugged", "true");
      return BT::NodeStatus::SUCCESS;
    } else {
      actionlib::SimpleActionClient<chargepal_actions::PlugInAdsAcAction>
          pi_ads("plug_in_ads_ac", true);
      pi_ads.waitForServer();

      pi_ads.sendGoal(goal);
      set_robot_value(robot_name, "ongoing_action", "plugin_charger_ads");

      pi_ads_action = pi_ads.waitForResult(ros::Duration(900.0));
      if (pi_ads_action) {
        result = *pi_ads.getResult();
        bool plug_in = result.success;
        if (plug_in) {
          set_robot_value(robot_name, "ongoing_action", "none");
          set_robot_value(robot_name, "previous_action", "plugin_charger_ads");
          set_cart_value(charger, "plugged", "true");
          return BT::NodeStatus::SUCCESS;
        }
      }

      /*std::cout << "Enter y/n if Plugin is successful  ";
      char userInput;
      std::cin >> userInput;
      if (userInput == 'y') {
        set_robot_value(robot_name, "ongoing_action", "none");
        set_robot_value(robot_name, "previous_action", "plugin_charger_ads");
        set_cart_value(charger, "plugged", "true");
        return BT::NodeStatus::SUCCESS;
      } else if (userInput == 'n') {
        set_robot_value(robot_name, "ongoing_action", "none");
        set_robot_value(robot_name, "previous_action",
                        "plugin_charger_ads_failure");
        return BT::NodeStatus::FAILURE;
      }*/
      enter_log_file("Plugin_ADS failed at" + target_station);
      update_gui_config("ongoing_action",
                        "Plugin_ADS failed at" + target_station);
      update_gui_config("error_count_plugin_ads", "");
      set_robot_value(robot_name, "ongoing_action", "none");
      set_robot_value(robot_name, "previous_action",
                      "plugin_charger_ads_failure");
      masterBlackboard->set("failed_action", "Plugin at" + target_station);
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  bool sim_flag, pi_ads_action;
  BT::Blackboard::Ptr masterBlackboard;
  std::ofstream &_argLog;
  chargepal_actions::PlugInAdsAcGoal goal;
  chargepal_actions::PlugInAdsAcResult result;
  std::string charger, robot_name, target_station;
};

class plugin_BCS : public BT::SyncActionNode {
public:
  plugin_BCS(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {}

  static PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    int retry_attempt = 0;

    actionlib::SimpleActionClient<chargepal_actions::PlugInBcsAcAction> pi_bcs(
        "plug_in_bcs_ac", true);
    pi_bcs.waitForServer();
    chargepal_actions::PlugInBcsAcGoal goal;

    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string charger = masterBlackboard->get<std::string>("charger");
    std::string robot_name = masterBlackboard->get<std::string>("robot_name");

    pi_bcs.sendGoal(goal);
    set_robot_value(robot_name, "ongoing_action", "plugin_charger_bcs");
    bool pi_bcs_action = pi_bcs.waitForResult(ros::Duration(900.0));
    if (pi_bcs_action) {
      chargepal_actions::PlugInBcsAcResult result = *pi_bcs.getResult();
      bool plug_in = result.success;
      if (plug_in) {
        set_robot_value(robot_name, "ongoing_action", "none");
        set_robot_value(robot_name, "previous_action", "plugin_charger_bcs");
        set_cart_value(charger, "plugged", "true");
        return BT::NodeStatus::SUCCESS;
      }
    }
    set_robot_value(robot_name, "ongoing_action", "none");
    set_robot_value(robot_name, "previous_action",
                    "plugin_charger_bcs_failure");
    return BT::NodeStatus::FAILURE;
  }
};

class plugout_ADS : public BT::SyncActionNode {
public:
  plugout_ADS(const std::string &name, const BT::NodeConfiguration &config,
              std::ofstream &logFile)
      : BT::SyncActionNode(name, config), _argLog(logFile) {}

  static PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    ros::param::get("/sim_flag", sim_flag);
    masterBlackboard = config().blackboard;
    charger = masterBlackboard->get<std::string>("charger");
    robot_name = masterBlackboard->get<std::string>("robot_name");
    location = read_robot_value(robot_name, "robot_location");
    enter_log_file("Performing plugout_ADS at " + location);
    update_gui_config("ongoing_action",
                      "Performing plugout_ADS at " + location);
    if (sim_flag) {
      set_robot_value(robot_name, "ongoing_action", "plugout_charger_ads");
      ros::Duration(10).sleep();
      set_robot_value(robot_name, "ongoing_action", "none");
      set_robot_value(robot_name, "previous_action", "plugout_charger_ads");
      set_cart_value(charger, "plugged", "false");
      return BT::NodeStatus::SUCCESS;
    } else {
      actionlib::SimpleActionClient<chargepal_actions::PlugOutAdsAcAction>
          po_ads("plug_out_ads_ac", true);
      po_ads.waitForServer();
      po_ads.sendGoal(goal);
      set_robot_value(robot_name, "ongoing_action", "plugout_charger_ads");
      po_ads_action = po_ads.waitForResult(ros::Duration(900.0));
      if (po_ads_action) {
        result = *po_ads.getResult();
        bool plug_out = result.success;
        if (plug_out) {
          set_robot_value(robot_name, "ongoing_action", "none");
          set_robot_value(robot_name, "previous_action", "plugout_charger_ads");
          set_cart_value(charger, "plugged", "false");
          return BT::NodeStatus::SUCCESS;
        }
      }

      /*std::cout << "Enter y/n if plugout is successful  ";
      char userInput;
      std::cin >> userInput;
      if (userInput == 'y') {
        set_robot_value(robot_name, "ongoing_action", "none");
        set_robot_value(robot_name, "previous_action", "plugout_charger_ads");
        set_cart_value(charger, "plugged", "false");
        return BT::NodeStatus::SUCCESS;
      } else if (userInput == 'n') {
        set_robot_value(robot_name, "ongoing_action", "none");
        set_robot_value(robot_name, "previous_action",
                        "plugout_charger_ads_failure");
        return BT::NodeStatus::FAILURE;
      }*/
      enter_log_file("Plugout_ADS failed at " + location);
      update_gui_config("ongoing_action", "Plugout_ADS failed at " + location);
      update_gui_config("error_count_plugout_ads", "");
      set_robot_value(robot_name, "ongoing_action", "none");
      set_robot_value(robot_name, "previous_action",
                      "plugout_charger_ads_failure");
      masterBlackboard->set("failed_action", "Plugout at" + location);
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  bool sim_flag, po_ads_action;
  BT::Blackboard::Ptr masterBlackboard;
  std::ofstream &_argLog;
  chargepal_actions::PlugOutAdsAcGoal goal;
  chargepal_actions::PlugOutAdsAcResult result;
  std::string charger, robot_name, location;
};

class plugout_BCS : public BT::SyncActionNode {
public:
  plugout_BCS(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {}

  static PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    int retry_attempt = 0;

    actionlib::SimpleActionClient<chargepal_actions::PlugOutBcsAcAction> po_bcs(
        "plug_out_bcs_ac", true);
    po_bcs.waitForServer();
    chargepal_actions::PlugOutBcsAcGoal goal;

    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string charger = masterBlackboard->get<std::string>("charger");
    std::string robot_name = masterBlackboard->get<std::string>("robot_name");

    po_bcs.sendGoal(goal);
    set_robot_value(robot_name, "ongoing_action", "plugout_charger_bcs");
    bool po_bcs_action = po_bcs.waitForResult(ros::Duration(900.0));
    if (po_bcs_action) {
      chargepal_actions::PlugOutBcsAcResult result = *po_bcs.getResult();
      bool plug_out = result.success;
      if (plug_out) {
        set_robot_value(robot_name, "ongoing_action", "none");
        set_robot_value(robot_name, "previous_action", "plugout_charger_bcs");
        set_cart_value(charger, "plugged", "false");
        return BT::NodeStatus::SUCCESS;
      }
    }
    set_robot_value(robot_name, "ongoing_action", "none");
    set_robot_value(robot_name, "previous_action",
                    "plugout_charger_bcs_failure");
    return BT::NodeStatus::FAILURE;
  }
};

class recovery_arrive_BWS : public BT::SyncActionNode {
public:
  recovery_arrive_BWS(const std::string &name,
                      const BT::NodeConfiguration &config,
                      std::ofstream &logFile)
      : BT::SyncActionNode(name, config), _argLog(logFile) {}

  static PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {

    masterBlackboard = config().blackboard;
    robot_name = masterBlackboard->get<std::string>("robot_name");

    actionlib::SimpleActionClient<chargepal_actions::ArriveAtStationAction> aas(
        "arrive_at_station", true);

    enter_log_file("Asking for free BWS");

    auto free_bws_result = ask_free_BWS(ask_ldb);
    free_bws = free_bws_result.first;
    connection_status = free_bws_result.second;
    enter_log_file("   Received: " + free_bws);
    enter_log_file("   Server connection: " + connection_status);

    while (free_bws != "none" || free_bws != "") {

      while (retry_attempt < 3) {
        aas.waitForServer();
        chargepal_actions::ArriveAtStationGoal goal;
        goal.target_station = free_bws;
        aas.sendGoal(goal);
        set_robot_value(robot_name, "ongoing_action",
                        "recovery_arrive_" + free_bws);
        enter_log_file("Performing arrive_at_station to " +
                       goal.target_station);
        bool aas_action = aas.waitForResult(ros::Duration(900.0));
        if (aas_action) {
          chargepal_actions::ArriveAtStationResult result = *aas.getResult();
          bool station_reached = result.station_reached;
          if (station_reached) {
            std::string current_station = result.current_station;
            set_robot_value(robot_name, "ongoing_action", "none");
            set_robot_value(robot_name, "previous_action",
                            "recovery_arrive_" + free_bws);
            set_robot_value(robot_name, "robot_location", current_station);

            std::string cart_on_robot =
                read_robot_value(robot_name, "cart_on_robot");
            if (cart_on_robot != "none") {
              set_cart_value(cart_on_robot, "cart_location", current_station);
            }
            reset_station_blocker("BWS");
            return BT::NodeStatus::SUCCESS;
          }
          action_result = result.action_status;
        }
        enter_log_file("arrive_at_station to" + goal.target_station +
                       " status is " + action_result);
        retry_attempt += 1;
      }

      auto free_bws_result = ask_free_BWS(ask_ldb);
      free_bws = free_bws_result.first;
      connection_status = free_bws_result.second;
      enter_log_file("   Received: " + free_bws);
      enter_log_file("   Server connection: " + connection_status);
      if (free_bws == "none" || free_bws == "") {
        break;
      }
    }

    enter_log_file("No free BWS left");
    return BT::NodeStatus::FAILURE;
  }

private:
  BT::Blackboard::Ptr masterBlackboard;
  std::ofstream &_argLog;
  int retry_attempt = 0;
  int ldb_connection_error_count = -1;
  bool ask_ldb = true;
  std::string free_bws = "";
  std::string connection_status = "";

  std::string robot_name, robot_location, action_result;
};

class recovery_arrive_BCS : public BT::SyncActionNode {
public:
  recovery_arrive_BCS(const std::string &name,
                      const BT::NodeConfiguration &config,
                      std::ofstream &logFile)
      : BT::SyncActionNode(name, config), _argLog(logFile) {}

  static PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    masterBlackboard = config().blackboard;
    robot_name = masterBlackboard->get<std::string>("robot_name");

    actionlib::SimpleActionClient<chargepal_actions::ArriveAtStationAction> aas(
        "arrive_at_station", true);

    enter_log_file("Asking for free BCS");

    auto free_bcs_result = ask_free_BCS(ask_ldb);
    free_bcs = free_bcs_result.first;
    connection_status = free_bcs_result.second;
    enter_log_file("   Received: " + free_bcs);
    enter_log_file("   Server connection: " + connection_status);

    while (free_bcs != "none" || free_bcs != "") {
      while (retry_attempt < 3) {
        aas.waitForServer();
        chargepal_actions::ArriveAtStationGoal goal;
        goal.target_station = free_bcs;
        aas.sendGoal(goal);
        set_robot_value(robot_name, "ongoing_action",
                        "recovery_arrive_" + free_bcs);
        enter_log_file("Performing arrive_at_station to " +
                       goal.target_station);
        bool aas_action = aas.waitForResult(ros::Duration(900.0));
        if (aas_action) {
          chargepal_actions::ArriveAtStationResult result = *aas.getResult();
          bool station_reached = result.station_reached;
          if (station_reached) {
            std::string current_station = result.current_station;
            set_robot_value(robot_name, "ongoing_action", "none");
            set_robot_value(robot_name, "previous_action",
                            "recovery_arrive_" + free_bcs);
            set_robot_value(robot_name, "robot_location", current_station);

            std::string cart_on_robot =
                read_robot_value(robot_name, "cart_on_robot");
            if (cart_on_robot != "none") {
              set_cart_value(cart_on_robot, "cart_location", current_station);
            }
            reset_station_blocker("BCS");
            return BT::NodeStatus::SUCCESS;
          }
          action_result = result.action_status;
        }
        enter_log_file("arrive_at_station to" + goal.target_station +
                       " status is " + action_result);
        retry_attempt += 1;
      }

      auto free_bcs_result = ask_free_BCS(ask_ldb);
      free_bcs = free_bcs_result.first;
      connection_status = free_bcs_result.second;
      enter_log_file("   Received: " + free_bcs);
      enter_log_file("   Server connection: " + connection_status);
      if (free_bcs == "none" || free_bcs == "") {
        break;
      }
    }

    enter_log_file("No free BCS left");
    return BT::NodeStatus::FAILURE;
  }

private:
  BT::Blackboard::Ptr masterBlackboard;
  std::ofstream &_argLog;
  int retry_attempt = 0;
  int ldb_connection_error_count = -1;
  bool ask_ldb = true;
  std::string free_bcs = "";
  std::string connection_status = "";

  std::string robot_name, robot_location, action_result;
};

class error_count : public BT::SyncActionNode {
public:
  error_count(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {}

  static PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string robot_name = masterBlackboard->get<std::string>("robot_name");
    std::string error_count = read_robot_value(robot_name, "error_count");
    int ec = std::stoi(error_count);
    ec = ec + 1;
    set_robot_value(robot_name, "error_count", std::to_string(ec));
    return BT::NodeStatus::SUCCESS;
  }
};

class time_sleep : public BT::SyncActionNode {
public:
  time_sleep(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {}

  static PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    ros::Duration(10).sleep();
    return BT::NodeStatus::SUCCESS;
  }
};
class sleep_until_charged : public BT::StatefulActionNode {
public:
  sleep_until_charged(const std::string &name,
                      const BT::NodeConfiguration &config)
      : BT::StatefulActionNode(name, config) {}

  static PortsList providedPorts() { return {}; }

  virtual NodeStatus onStart() override {

    BT::Blackboard::Ptr masterBlackboard = config().blackboard;
    std::string robot_name = masterBlackboard->get<std::string>("robot_name");
    std::string cart = read_robot_value(robot_name, "cart_on_robot");
    int msec = get_operation_time(cart);

    if (msec <= 0) {
      return BT::NodeStatus::SUCCESS;
    } else {
      // once the deadline is reached, we will return SUCCESS.
      deadline_ = system_clock::now() + milliseconds(msec);
      return BT::NodeStatus::RUNNING;
    }
  }

  /// method invoked by an action in the RUNNING state.
  virtual NodeStatus onRunning() override {
    if (system_clock::now() >= deadline_) {
      return BT::NodeStatus::SUCCESS;
    } else {
      return BT::NodeStatus::RUNNING;
    }
  }

  void onHalted() override {
    std::cout << "SleepNode interrupted" << std::endl;
  }

private:
  system_clock::time_point deadline_;
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "chargepal_bt");
  ros::NodeHandle nh;
  bool recovery_flag;
  float server_timeout;
  std::ofstream logFile(ros::package::getPath("chargepal_bundle") +
                        "/logs/chargepal_logs.txt");

  ros::package::getPath("chargepal_bundle");
  // logFile.open(ros::package::getPath("chargepal_bundle") +
  //                 "/logs/chargepal_logs.txt",
  //             std::ofstream::out | std::ofstream::app);

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
  factory.registerNodeType<isReady2Plugin>("isReady2Plugin");
  factory.registerNodeType<askAssertLift>("askAssertLift");
  factory.registerNodeType<isCartPlaced>("isCartPlaced");

  factory.registerNodeType<arrive_at_station>("arrive_at_station",
                                              std::ref(logFile));
  factory.registerNodeType<go_home>("go_home", std::ref(logFile));
  factory.registerNodeType<call_for_help>("call_for_help");
  factory.registerNodeType<drop_cart>("drop_cart", std::ref(logFile));
  factory.registerNodeType<pickup_cart>("pickup_cart", std::ref(logFile));
  factory.registerNodeType<plugin_ADS>("plugin_ADS", std::ref(logFile));
  factory.registerNodeType<plugin_BCS>("plugin_BCS");
  factory.registerNodeType<plugout_ADS>("plugout_ADS", std::ref(logFile));
  factory.registerNodeType<plugout_BCS>("plugout_BCS");
  factory.registerNodeType<recovery_arrive_BWS>("recovery_arrive_BWS",
                                                std::ref(logFile));
  factory.registerNodeType<recovery_arrive_BCS>("recovery_arrive_BCS",
                                                std::ref(logFile));
  factory.registerNodeType<sleep_until_charged>("sleep_until_charged");
  factory.registerNodeType<error_count>("error_count");
  factory.registerNodeType<time_sleep>("time_sleep");

  std::string package_folder =
      ros::package::getPath("chargepal_behaviour_tree");

  factory.registerBehaviorTreeFromFile(package_folder + "/xml/Main.xml");
  factory.registerBehaviorTreeFromFile(package_folder + "/xml/Jobs.xml");
  factory.registerBehaviorTreeFromFile(package_folder + "/xml/Actionsets.xml");
  factory.registerBehaviorTreeFromFile(package_folder + "/xml/Actions.xml");
  factory.registerBehaviorTreeFromFile(package_folder + "/xml/Recovery.xml");

  auto mainTree = factory.createTree("main", masterBlackboard);
  BT::Groot2Publisher publisher(mainTree);

  json job_requested;

  while (ros::ok() && job_requested.empty()) {
    try {
      std::string job_input = fetch_job();
      if (!job_input.empty()) {
        job_requested = json::parse(job_input);
      }

      else {
        std::cout << "Waiting for new job";
        ros::Duration(1.0).sleep();
      }
    } catch (const nlohmann::json::exception &e) {
      std::cerr << "Error parsing JSON: " << e.what() << std::endl;
    }

    if (!job_requested.empty()) {
      // logFile.close();
      logFile.open(ros::package::getPath("chargepal_bundle") +
                       "/logs/chargepal_logs.txt",
                   std::ofstream::out | std::ofstream::trunc);
      logFile.close();

      std::string job_type = job_requested["job_type"];
      std::string robot_name = job_requested["robot_name"];
      std::string charger = job_requested["cart"];
      std::string source_station = job_requested["source_station"];
      std::string target_station = job_requested["target_station"];

      masterBlackboard->set("job_type", job_type);
      masterBlackboard->set("robot_name", robot_name);
      masterBlackboard->set("charger", charger);
      masterBlackboard->set("source_station", source_station);
      masterBlackboard->set("target_station", target_station);
      masterBlackboard->set("job_status", 0);
      masterBlackboard->set("failed_action", "");

      ros::param::get("/recovery_enabled", recovery_flag);
      masterBlackboard->set("recovery_enabled", recovery_flag);

      ROS_INFO("Performing: %s", job_type.c_str());
      mainTree.tickWhileRunning();

      BT::NodeStatus status = mainTree.rootNode()->status();

      if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::IDLE ||
          status == BT::NodeStatus::FAILURE) {

        bool job_server_update = false;

        float start_time = ros::Time::now().toSec();
        ros::param::get("/server_timeout", server_timeout);
        while (!job_server_update && ros::ok()) {
          std::string job_status =
              enumToString(masterBlackboard->get<int>("job_status"));

          update_gui_config("ongoing_action", "Job completed");
          enter_log_file("Job completed with status: " + job_status);
          job_server_update = update_job_monitor(job_type, job_status);
          std::cout << "The job update received is  :" << job_server_update
                    << std::endl;

          // server_timeout = ros::Time::now().toSec() - start_time;
        }
        // Call for help if server timeout is > 10 minutes
        job_requested.clear();
        std::cout << "................................" << std::endl;
      }
    }
  }

  ros::spinOnce();

  // Exit the ROS node
  ros::shutdown();
  return 0;
}