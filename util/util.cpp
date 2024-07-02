#include "util.h"

/**
 * @brief Pushes the RDBC (Robot Database Copy) to LDB (Local Database).
 *
 * This function attempts to push the RDBC to LDB by sending a goal to the
 * "db/push_rdbc_to_ldb" action server. It retries up to 3 times if the push
 * fails. The function returns true if the push is successful, and false otherwise.
 *
 * @return True if the RDBC push to LDB is successful, false otherwise.
 */
bool push_rdb_copy()
{
  int retry_count = 0;
  bool push_status = false;
  while (!push_status && retry_count < 3)
  {
    actionlib::SimpleActionClient<chargepal_actions::PushRdbcToLdbAction> prctl(
        "db/push_rdbc_to_ldb", true);
    chargepal_actions::PushRdbcToLdbResult result;
    prctl.waitForServer();

    chargepal_actions::PushRdbcToLdbGoal goal;
    prctl.sendGoal(goal);
    bool prctl_action = prctl.waitForResult(ros::Duration(900.0));
    if (prctl_action)
    {
      result = *prctl.getResult();
      push_status = result.push_success;
    }
    retry_count++;
  }
  return push_status;
}

/**
 * @brief Verifies the connection between the RDB-LDB server.
 *
 * This function checks if the RDB/LDB server is connected by waiting for a specific message on the "/grpc_server_status" topic.
 * It waits for a maximum of `server_timeout` seconds for the server to respond with the message "SERVER_CONNECTED".
 *
 * @param server_timeout The maximum time to wait for the server to respond, in seconds.
 * @return True if the server is connected, false otherwise.
 */
bool verify_rdb_ldb_connection(const int &server_timeout)
{
  bool server_connection = false;
  ros::Time start_time = ros::Time::now();
  while ((ros::Time::now() - start_time).toSec() < server_timeout)
  {

    boost::shared_ptr<const std_msgs::String> msg = ros::topic::waitForMessage<std_msgs::String>("/grpc_server_status", ros::Duration(server_timeout));

    if (msg && msg->data == "SERVER_CONNECTED")
    {

      ROS_INFO("Server is connected!");
      server_connection = true;
      break;
    }
  }
  return server_connection;
}

/**
 * @brief Identifies the job components (robot and carts) based on the given parameters and job request.
 *
 * This function takes a map of ChargePal parameters and a JSON object representing the job request.
 * It identifies the job components and returns a map containing the target components.
 *
 * @param cp_params A map of ChargePal parameters.
 * @param job_requested The JSON object representing the job request.
 * @return A map containing the target components identified for the job.
 */
std::map<std::string, std::vector<std::string>> identify_job_components(std::map<std::string, std::any> cp_params, const json &job_requested)
{
  std::map<std::string, std::vector<std::string>> target_components;
  std::string job_cart = job_requested["cart"];
  target_components["robot_info"] = {std::any_cast<std::string>(cp_params["robot"])};
  target_components["cart_info"] = {};
  target_components["battery_action_info"] = {};

  std::string cart_on_robot = read_robot_value(std::any_cast<std::string>(cp_params["rdb_path"]), std::any_cast<std::string>(cp_params["robot"]), "cart_on_robot");

  if (job_cart != "")
  {
    target_components["cart_info"].push_back(job_cart);
    target_components["battery_action_info"].push_back(job_cart);
  }
  if (cart_on_robot != "")
  {
    // to check if cart_on_robot is not present in cart_info already
    auto it = target_components.find("cart_info");
    if (it != target_components.end())
    {
      const auto &items = it->second;
      bool found = std::find(items.begin(), items.end(), cart_on_robot) != items.end();
      if (!found)
      {
        target_components["cart_info"].push_back(cart_on_robot);
        target_components["battery_action_info"].push_back(cart_on_robot);
      }
    }
  }
  return target_components;
}

/**
 * Converts a variant value to a SetColumnValue object.
 *
 * This function takes a variant value representing an integer, boolean, or string,
 * and converts it to a SetColumnValue object. The SetColumnValue object is a wrapper
 * class that can hold an optional value of type int, bool, or string.
 *
 * @param colval The variant value to be converted.
 * @return The converted SetColumnValue object.
 */
SetColumnValue set_to_column_value_type(const std::variant<int, bool, std::string> &colval)
{
  return std::visit([](auto &&arg) -> SetColumnValue
                    {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, int>) {
            return SetColumnValue(std::optional<int>{arg});
        } else if constexpr (std::is_same_v<T, bool>) {
            return SetColumnValue(std::optional<bool>{arg});
        } else if constexpr (std::is_same_v<T, std::string>) {
            if (arg == "none") {
                return SetColumnValue(std::nullptr_t{});
            } else {
                return SetColumnValue(std::optional<std::string>{arg});
            }
        } }, colval);
}

/**
 * @brief Sets the RDBC values in the specified database.
 *
 * This function updates the values in the RDBC (Robot Database Copy) based on the provided tables.
 * It iterates through each table and column value, constructs an SQL query, and executes the query to update the values in the database.
 * The function also updates the robot charge if the table name is not "battery_action_info".
 * Finally, it pushes RDBC to the server and returns true if the operation is successful, otherwise returns false.
 *
 * @param rdbc_path The path to the RDBC database file.
 * @param robot_name The name of the robot.
 * @param tables A vector of TableInfo objects containing the table and column values to update.
 * @return True if the RDBC values are successfully set, false otherwise.
 */
bool set_rdbc_values(const std::string &rdbc_path, const std::string &robot_name, const std::vector<TableInfo> &tables)
{
  try
  {
    SQLite::Database db(rdbc_path,
                        SQLite::OPEN_READWRITE | SQLite::OPEN_CREATE);

    for (const auto &table : tables)
    {
      const std::string &table_name = table.table_name;
      const std::string &row_name = table.row.row_name;
      const std::string &ldb_column = "ldb_push";
      for (const auto &colval : table.row.column_values)
      {
        if (colval.column_name != "robot_charge")
        {
          std::string column_to_update = colval.column_name;
          SetColumnValue column_value = set_to_column_value_type(colval.value);
          std::string sql_query = "UPDATE " + table_name + " SET " + column_to_update + " = ?," + ldb_column + " = ? WHERE name = ?";
          SQLite::Statement query(db, sql_query);

          // Bind the values based on the type in the variant
          std::visit([&query](auto &&arg)
                     {
            using T = std::decay_t<decltype(arg)>;
            if constexpr (std::is_same_v<T, std::optional<int>>) {
                if (arg.has_value()) {
                    query.bind(1, arg.value());
                } else {
                    query.bind(1);
                }
            } else if constexpr (std::is_same_v<T, std::optional<bool>>) {
                if (arg.has_value()) {
                    query.bind(1, arg.value());
                } else {
                    query.bind(1);
                }
            } else if constexpr (std::is_same_v<T, std::optional<std::string>>) {
                if (arg.has_value()) {
                    query.bind(1, arg.value());
                } else {
                    query.bind(1);
                }
            } else if constexpr (std::is_same_v<T, std::nullptr_t>) {
                query.bind(1);  // Bind NULL value
            } }, column_value);

          query.bind(2, false);
          query.bind(3, row_name);

          query.exec();
          query.reset();
        }
      }

      if (table.table_name != "battery_action_info") // to avoid asynchronous updates through battery actions
      {
        update_robot_charge(rdbc_path, robot_name);
      }
    }
    push_rdb_copy();
    return true;
  }
  catch (const std::exception &e)
  {
    std::cerr << "Exception: " << e.what() << std::endl;
    return false;
  }
}

/**
 * @brief Reads a specific value from the robot_info table in the given SQLite database.
 *
 * This function retrieves a specific value from the robot_info table based on the provided robot name and key.
 * The value is returned as a string.
 *
 * @param db_path The path to the SQLite database file.
 * @param name The name of the robot.
 * @param key The key corresponding to the desired value.
 * @return The value as a string.
 */
std::string read_robot_value(const std::string db_path, const std::string &name, const std::string &key)
{

  std::string value = "";
  std::string robot_name = name;
  std::string selected_column = key;
  SQLite::Database db(db_path);
  SQLite::Statement query(db, "SELECT CASE ? "
                              "WHEN 'robot_location' THEN robot_location "
                              "WHEN 'cart_on_robot' THEN cart_on_robot "
                              "WHEN 'previous_action' THEN previous_action "
                              "WHEN 'ongoing_action' THEN ongoing_action "
                              "WHEN 'current_job' THEN current_job "
                              "WHEN 'error_count' THEN error_count "
                              "ELSE NULL END AS result "
                              "FROM robot_info WHERE name = ?");

  query.bind(1, selected_column);
  query.bind(2, robot_name);

  while (query.executeStep())
  {
    value = query.getColumn(0).getText();
  }
  query.reset();

  std::string value_str = convertToString(value);
  return value_str;
}

/**
 * @brief Reads the value of a specific column from the cart_info table in the SQLite database.
 *
 * This function takes the path to the SQLite database file, the name of the cart, and the key (column name) as input parameters.
 * It executes a SELECT query to retrieve the value of the specified column for the given cart name.
 * The function returns the value as a string.
 *
 * @param db_path The path to the SQLite database file.
 * @param name The name of the cart.
 * @param key The key (column name) to retrieve the value from.
 * @return The value of the specified column as a string.
 */
std::string read_cart_value(const std::string db_path, const std::string &name, const std::string &key)
{

  std::string value = "";
  std::string cart_name = name;
  std::string selected_column = key;
  SQLite::Database db(db_path);
  SQLite::Statement query(db, "SELECT CASE ? "
                              "WHEN 'cart_location' THEN cart_location "
                              "WHEN 'robot_on_cart' THEN robot_on_cart "
                              "WHEN 'plugged' THEN plugged "
                              "ELSE NULL END AS result "
                              "FROM cart_info WHERE name = ?");

  query.bind(1, selected_column);
  query.bind(2, cart_name);

  while (query.executeStep())
  {
    value = query.getColumn(0).getText();
  }
  query.reset();

  std::string value_str = convertToString(value);
  return value_str;
}

/**
 * Updates the charge information of a robot in the database.
 *
 * @param rdbc_path The path to the SQLite database file.
 * @param robot_name The name of the robot.
 */
void update_robot_charge(const std::string &rdbc_path, const std::string &robot_name)
{
  ros::NodeHandle n;
  ros::ServiceClient client_ldb_server =
      n.serviceClient<chargepal_services::readRobotCharge>(
          "/mir_rest_api/robot_charge");
  chargepal_services::readRobotCharge srv_ldb_server;
  srv_ldb_server.request.robot_name = robot_name;

  if (client_ldb_server.call(srv_ldb_server))
  {
    float charge = srv_ldb_server.response.robot_charge;

    SQLite::Database db(rdbc_path,
                        SQLite::OPEN_READWRITE | SQLite::OPEN_CREATE);
    std::string sql_query =
        "UPDATE robot_info SET robot_charge = ?, ldb_push = ? WHERE name = ?";
    SQLite::Statement query(db, sql_query);
    query.bind(1, charge);
    query.bind(2, false);
    query.bind(3, robot_name);
    query.exec();
    query.reset();
  }
}

/**
 * @brief Resets the station blocker for a given station.
 *
 * This function sends a service request to the "/ldb_server/reset_station_blocker" service
 * to reset the station blocker for the specified station.
 *
 * @param station_name The name of the station to reset the blocker for.
 * @return True if the station blocker was successfully reset, false otherwise.
 */
bool reset_station_blocker(const std::string &station_name)
{

  ros::NodeHandle n;
  ros::ServiceClient client_ldb_server =
      n.serviceClient<chargepal_services::resetStationBlocker>(
          "/ldb_server/reset_station_blocker");
  chargepal_services::resetStationBlocker srv_ldb_server;
  srv_ldb_server.request.station = station_name;

  if (client_ldb_server.call(srv_ldb_server))
  {
    std::string connection = srv_ldb_server.response.connection_status;
    return srv_ldb_server.response.success;
  }
  else
  {
    return false;
  }
}

/**
 * @brief Asks the LDB server for free BWS (Battery Waiting Station) and server connection status.
 *
 * This function sends a service request to the LDB server to ask for free BWS. It keeps sending the request until a successful response is received or a maximum number of connection errors is reached.
 *
 * @param ask_ldb A boolean indicating whether to ask the LDB server for free BWS.
 * @return A pair of strings representing the free BWS station name and the connection status. If a successful response is received, the station name and connection status are returned. Otherwise, an empty string is returned for both values.
 */
std::pair<std::string, std::string> ask_free_BWS(const bool &ask_ldb)
{
  std::string connection_status = "";
  std::string free_bws = "";
  bool ldb_check = ask_ldb;
  ros::NodeHandle n;
  int ldb_connection_error_count = -1;
  while (connection_status != " SUCCESSFUL" && ldb_connection_error_count < 5)
  {
    ros::ServiceClient client_ldb_server =
        n.serviceClient<chargepal_services::askFreeBWS>(
            "/ldb_server/ask_free_bws");
    chargepal_services::askFreeBWS srv_ldb_server;

    srv_ldb_server.request.ask_ldb = ask_ldb;
    if (client_ldb_server.call(srv_ldb_server))
    {
      free_bws = srv_ldb_server.response.station_name;
      connection_status = srv_ldb_server.response.connection_status;
      return std::make_pair(free_bws, connection_status);
    }
    else
    {
      ldb_connection_error_count++;
      if (ldb_connection_error_count == 5)
      {
        if (ldb_check)
        {
          ldb_check = false;
          ldb_connection_error_count = 0;
        }
        else
        {
          break;
        }
      }
    }
  }
  return std::make_pair(free_bws, connection_status);
}

/**
 * @brief Asks the LDB server for free BCS (Battery Charging Stations).
 *
 * This function sends a request to the LDB server to ask for free BCS. It keeps
 * sending the request until a successful response is received or a maximum number
 * of connection errors is reached.
 *
 * @param ask_ldb A boolean indicating whether to ask the LDB server for free BCS.
 * @return A pair of strings representing the free BCS and the connection status.
 *         The first string is the name of the free BCS, and the second string is
 *         the connection status.
 */
std::pair<std::string, std::string> ask_free_BCS(const bool &ask_ldb)
{
  std::string connection_status = "";
  std::string free_bcs = "";
  bool ldb_check = ask_ldb;
  ros::NodeHandle n;
  int ldb_connection_error_count = -1;
  while (connection_status != " SUCCESSFUL" && ldb_connection_error_count < 5)
  {
    ros::ServiceClient client_ldb_server =
        n.serviceClient<chargepal_services::askFreeBCS>(
            "/ldb_server/ask_free_bcs");
    chargepal_services::askFreeBCS srv_ldb_server;

    srv_ldb_server.request.ask_ldb = ask_ldb;
    if (client_ldb_server.call(srv_ldb_server))
    {
      free_bcs = srv_ldb_server.response.station_name;
      connection_status = srv_ldb_server.response.connection_status;
      return std::make_pair(free_bcs, connection_status);
    }
    else
    {
      ldb_connection_error_count++;
      if (ldb_connection_error_count == 5)
      {
        if (ldb_check)
        {
          ldb_check = false;
          ldb_connection_error_count = 0;
        }
        else
        {
          break;
        }
      }
    }
  }
  return std::make_pair(free_bcs, connection_status);
}

/**
 * @brief Reads the assert lift value from the robot.
 *
 * This function calls the "/mir_rest_api/assert_lift_value" service to retrieve the assert lift value from the robot.
 *
 * @param robot_name The name of the robot.
 * @return The assert lift value as a string.
 */
std::string read_assertLift_value(const std::string &robot_name)
{
  std::string assert_lift_value = "";
  ros::NodeHandle n;
  ros::ServiceClient client_mir_rapi =
      n.serviceClient<chargepal_services::assertLiftValue>(
          "/mir_rest_api/assert_lift_value");
  chargepal_services::assertLiftValue srv_mir_rapi;
  if (client_mir_rapi.call(srv_mir_rapi))
  {
    assert_lift_value = srv_mir_rapi.response.state;
  }
  else
  {
    ROS_ERROR("assert_lift_value service failed");
  }
  return assert_lift_value;
}

/**
 * @brief Fetches a job from the LDB server.
 *
 * This function calls the "/ldb_server/fetch_job" service to retrieve a job from the LDB server.
 * If the service call is successful, the job is returned as a string. Otherwise, an error message is logged.
 *
 * @return The fetched job as a string.
 */
std::string fetch_job()
{
  std::string job;
  ros::NodeHandle n;
  ros::ServiceClient client_ldb_server =
      n.serviceClient<chargepal_services::fetchJob>("/ldb_server/fetch_job");
  chargepal_services::fetchJob srv_ldb_server;

  try
  {
    if (client_ldb_server.call(srv_ldb_server))
    {

      job = srv_ldb_server.response.job;
    }
    else
    {
      ROS_ERROR("Fetch job service failed");
    }
  }
  catch (...)
  {
    ROS_INFO("Connecting to server failed");
  }

  return job;
}

/**
 * @brief Updates the job monitor by calling the ldb_server service.
 *
 * This function sends a request to the ldb_server service to update the job monitor
 * with the specified job type and job status. It waits for a response from the service
 * and returns true if the update was successful within the specified server timeout.
 *
 * @param server_timeout The maximum time to wait for the update to complete, in seconds.
 * @param robot_name The name of the robot.
 * @param job_type The type of the job to update.
 * @param job_status The status of the job to update.
 * @return True if the job monitor was successfully updated, false otherwise.
 */
bool update_job_monitor(const int &server_timeout, const std::string &robot_name,
                        const std::string &job_type, const std::string &job_status)
{

  ros::Time start_time = ros::Time::now();
  bool update_job = false;
  ros::NodeHandle n;
  while ((ros::Time::now() - start_time).toSec() < server_timeout)
  {
    ros::ServiceClient client_ldb_server =
        n.serviceClient<chargepal_services::updateJobMonitor>(
            "/ldb_server/update_job_monitor");

    chargepal_services::updateJobMonitor srv_ldb_server;
    srv_ldb_server.request.job_type = job_type;
    srv_ldb_server.request.job_status = job_status;

    if (client_ldb_server.call(srv_ldb_server))
    {
      update_job = srv_ldb_server.response.success;
      if (update_job)
      {
        break;
      }
    }
    ros::Duration(30.0).sleep(); // Sleep for 30 seconds and retry
  }

  return update_job;
}

/**
 * @brief Retrieves the operation time for a given cart.
 *
 * This function calls the `/ldb_server/charging_operation_time` service to get the operation time
 * for the specified cart. The operation time is returned in milliseconds.
 *
 * @param cart_name The name of the cart for which to retrieve the operation time.
 * @return The operation time in milliseconds.
 */
int get_operation_time(const std::string &cart_name)
{
  int operation_time = 0;
  ros::NodeHandle n;
  ros::ServiceClient client_ldb_server =
      n.serviceClient<chargepal_services::askOperationTime>(
          "/ldb_server/charging_operation_time");
  chargepal_services::askOperationTime srv_ldb_server;
  srv_ldb_server.request.cart_name = cart_name;
  if (client_ldb_server.call(srv_ldb_server))
  {
    operation_time = srv_ldb_server.response.time_msec;
  }
  else
  {
    ROS_ERROR("asking operation time service failed");
  }
  return operation_time;
}

/**
 * @brief Converts an integer value to its corresponding string representation.
 *
 * This function takes an integer value and returns a string representation based on the following mapping:
 * - 0: "Ongoing"
 * - 1: "Success"
 * - 2: "Recovery"
 * - 3: "Failure"
 *
 * If the input value does not match any of the above cases, an "Unknown job status" message is printed to the console.
 *
 * @param value The integer value to be converted.
 * @return The string representation of the input value.
 */
std::string enumToString(const int value)
{

  std::string set_string = "";
  switch (value)
  {
  case 0:
    set_string = "Ongoing";
    break;
  case 1:
    set_string = "Success";
    break;
  case 2:
    set_string = "Recovery";
    break;
  case 3:
    set_string = "Failure";
    break;
  default:
    std::cout << "Unknown job status" << std::endl;
    break;
  }
  ROS_INFO("Job status is: %s", set_string.c_str());
  return set_string;
}

/**
 * @brief Writes the given content to a log file at the specified file path.
 *
 * @param file_path The path of the log file.
 * @param content The content to be written to the log file.
 */
void enter_log_file(const std::string file_path, const std::string content)
{
  std::ofstream log_file;
  log_file.open(file_path,
                std::ios::app);
  log_file << current_timestamp() << " " << content << std::endl;
  log_file.close();
}

// Function to get the current timestamp as a string
/**
 * Returns the current timestamp as a string in the format "YYYY-MM-DD HH:MM:SS.SSS".
 * The timestamp includes milliseconds.
 *
 * @return The current timestamp as a string.
 */
std::string current_timestamp()
{
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()) %
            1000;

  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S");
  ss << '.' << std::setw(3) << std::setfill('0') << ms.count();
  return ss.str();
}

/**
 * @brief Recovers the specified action for the cart by resetting the io poins.
 *
 * This function calls the "/mir_rest_api/reset_io_for_cart" service to recover the specified action for the cart.
 *
 * @param action_name The name of the action to recover.
 * @return True if the action was successfully recovered, false otherwise.
 */
bool recover_cart(const std::string action_name)
{
  ros::NodeHandle n;
  std::ostringstream oss;
  bool success;
  ros::ServiceClient client_ldb_server =
      n.serviceClient<chargepal_services::resetIoForCart>(
          "/mir_rest_api/reset_io_for_cart");
  chargepal_services::resetIoForCart srv_ldb_server;
  srv_ldb_server.request.action = action_name;

  if (client_ldb_server.call(srv_ldb_server))
  {
    success = srv_ldb_server.response.success;
  }
  else
  {
    success = false;
    ROS_ERROR("%s", oss.str().c_str());
  }
  return success;
}

/**
 * Pulls job related components (robot and cart) from the robot database (RDB) to the robot database copy (RDBC).
 *
 * This function establishes a connection with the action server responsible for pulling
 * data from the RDB to the RDBC. It sends a goal containing the table names and row names
 * to be pulled. The function waits for the action server to complete the goal and returns
 * the result indicating whether the pull operation was successful or not.
 *
 * @param table_rows A map containing the table names as keys and the corresponding row names as values.
 * @return A boolean value indicating whether the pull operation was successful or not.
 */
bool pull_rdb_to_rdbc(std::map<std::string, std::vector<std::string>> &table_rows)
{

  actionlib::SimpleActionClient<chargepal_actions::PullRdbToRdbcAction> prtrc(
      "db/pull_rdb_to_rdbc", true);
  chargepal_actions::PullRdbToRdbcResult result;
  prtrc.waitForServer();

  chargepal_actions::PullRdbToRdbcGoal goal;
  for (const auto &item : table_rows)
  {
    chargepal_actions::TableRow tr;
    tr.table = item.first;
    tr.row_names = item.second;
    goal.parameters.push_back(tr);
  }
  prtrc.sendGoal(goal);
  bool prtrc_action = prtrc.waitForResult(ros::Duration(900.0));
  if (prtrc_action)
  {
    result = *prtrc.getResult();
    return result.pull_success;
  }
  else
  {
    return false;
  }
}

/**
 * @brief Retrieves ROS parameters and stores them in a map.
 *
 * This function retrieves various ROS parameters using the provided NodeHandle object and stores them in a map.
 * The map contains key-value pairs, where the key is a string representing the parameter name and the value is of type std::any.
 *
 * @param nh The ROS NodeHandle object used to retrieve the parameters.
 * @return A map containing the retrieved ROS parameters.
 */
std::map<std::string, std::any> set_ros_params(const ros::NodeHandle &nh)
{
  std::map<std::string, std::any> chargepal_ros_params;
  bool recovery_flag;
  int server_timeout;
  std::string rdbc_path;
  std::string rdb_path;
  std::string log_file_path;
  std::string robot;
  bool sim_flag;

  nh.getParam("/recovery_enabled", recovery_flag);
  nh.getParam("/server_timeout", server_timeout);
  nh.getParam("/rdbc_path", rdbc_path);
  nh.getParam("/rdb_path", rdb_path);
  nh.getParam("/log_file_path", log_file_path);
  nh.getParam("/robot_name", robot);
  nh.getParam("/sim_flag", sim_flag);

  chargepal_ros_params["recovery_flag"] = recovery_flag;
  chargepal_ros_params["server_timeout"] = server_timeout;
  chargepal_ros_params["rdbc_path"] = rdbc_path;
  chargepal_ros_params["rdb_path"] = rdb_path;
  chargepal_ros_params["log_file_path"] = log_file_path;
  chargepal_ros_params["robot"] = robot;
  chargepal_ros_params["sim_flag"] = sim_flag;
  return chargepal_ros_params;
}

/**
 * Calls for help using the specified robot name and text.
 *
 * @param robot_name The name of the robot.
 * @param text The context or message for help.
 */
void calling_help(const std::string &robot_name, const std::string &text)
{
  actionlib::SimpleActionClient<chargepal_actions::CallForHelpAction>
      cfh("call_for_help", true);
  chargepal_actions::CallForHelpGoal goal;
  goal.robot = robot_name;
  goal.context = text;
  cfh.waitForServer();
  cfh.sendGoal(goal);
  cfh.waitForResult(ros::Duration(900.0));
}