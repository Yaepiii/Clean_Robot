#include <explore/explore_server.h>
#include <explore/geometry_tools.h>

#include <thread>

inline static bool operator==(const geometry_msgs::Point& one,
                              const geometry_msgs::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace explore_server
{
void Explore_Server::configCb(ExploreConfig& config, uint32_t level)
{
  ROS_INFO("Dynamic Config Start");
  timeout = config.timeout;
  min_frontier_size_ = config.min_frontier_size;
  visualize_ = config.visualize;
  frontier_type_ = config.frontier_type;
  potential_scale_ = config.potential_scale;
  gain_scale_ = config.gain_scale;
  information_scale_ = config.information_scale;
  orientation_scale_ = config.orientation_scale;
  information_r_ = config.information_r;
}


Explore_Server::Explore_Server()
  : private_nh_("~")
  , tf_listener_(ros::Duration(10.0))
  , costmap_client_(private_nh_, relative_nh_, &tf_listener_)
  , move_base_client_("move_base")
  , prev_distance_(0)
  , last_markers_count_(0)
  , explore_action_server_(relative_nh_,
                        "explore_server",
                        boost::bind(&Explore_Server::goalCb, this, _1),
                        boost::bind(&Explore_Server::cancelGoalCb, this, _1),
                        false) 
{
  private_nh_.param("planner_frequency", planner_frequency_, 1.0);
  private_nh_.param("progress_timeout", timeout, 30.0);
  private_nh_.param("visualize", visualize_, false);
  private_nh_.param("potential_scale", potential_scale_, 1e-3);
  private_nh_.param("gain_scale", gain_scale_, 1.0);
  private_nh_.param("information_scale", information_scale_, 1.2);
  private_nh_.param("orientation_scale", orientation_scale_, 9.7);
  private_nh_.param("min_frontier_size", min_frontier_size_, 0.5);
  private_nh_.param("frontier_type", frontier_type_, std::string("centroid"));
  private_nh_.param("information_r", information_r_, 3.0);


  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_, information_scale_, orientation_scale_,
                                                 min_frontier_size_, information_r_);

  cb = boost::bind(&Explore_Server::configCb, this, _1, _2);
  configServer.setCallback(cb);

  if (visualize_) {
    marker_array_publisher_ =
        private_nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 10);
  }

  ROS_INFO("Waiting to connect to move_base server");
  move_base_client_.waitForServer();
  ROS_INFO("Connected to move_base server");

  explore_action_server_.start();

}

Explore_Server::~Explore_Server()
{
  stop();
}

void Explore_Server::goalCb(GoalHandle gh)
{
  explore_center_ = gh.getGoal()->start_point;

  // set as active goal GoalHandle
  gh.setAccepted();
  active_gh_ = gh;

  // update boundary polygon on costmap, if necessary
  ros::ServiceClient polygon_client =
      relative_nh_.serviceClient<exploration_msgs::SetPolygon>(
          "move_base/global_costmap/polygon_layer/set_polygon");
  exploration_msgs::SetPolygon polygon_service;
  // make use of the polygon service to set the boundary on the polygon using
  // what is stored in the goalhandle
  polygon_service.request.polygon = gh.getGoal()->boundary; // PolygonStamped
  polygon_ = gh.getGoal()->boundary;

  ROS_INFO("size: %ld",polygon_.polygon.points.size());

  if (polygon_client.call(polygon_service)) {
    ROS_INFO("Updating polygon");
  } else {
    ROS_ERROR("Failed to call update polygon service.");
    gh.setAborted();
    return;
  }

  // request a goal from the plugin and send to move_base
  ROS_INFO("Requesting a goal");

  exploring_timer_ = relative_nh_.createTimer(ros::Duration(1. / planner_frequency_),
                                 [this](const ros::TimerEvent&) { makePlan(); });

}

void Explore_Server::cancelGoalCb(GoalHandle gh)
{
  {  // scope the lock
    // grab the move_client mutex, lock it, then cancel all move_base goals
    boost::unique_lock<boost::mutex> lock(move_client_lock_);
    move_base_client_.cancelGoalsAtAndBeforeTime(ros::Time::now());
    lock.unlock();
  }
  ROS_WARN("Current exploration task cancelled");
  // set the GoalHandle as cancelled
  if (gh == active_gh_) {
    gh.setCanceled();
  }
}

void Explore_Server::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  std_msgs::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;

  ROS_DEBUG("visualising %lu frontiers", frontiers.size());
  visualization_msgs::MarkerArray markers_msg;
  std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
  visualization_msgs::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = ros::Time::now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
  m.lifetime = ros::Duration(0);
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double max_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::Marker::ADD;
  size_t id = 1;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::Marker::POINTS;
    m.id = int(id);
    m.pose.position = {};
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.centroid;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::max(std::abs(frontier.cost / max_cost), 0.3);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_.publish(markers_msg);
}

void Explore_Server::makePlan()
{
  progress_timeout_ = ros::Duration(timeout);

  search_.configUpdate(min_frontier_size_, potential_scale_, gain_scale_, information_scale_, orientation_scale_, information_r_);

  // find frontiers
  auto pose = costmap_client_.getRobotPose();
  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.pose);
  for (size_t i = 0; i < frontiers.size(); ++i) {
    ROS_DEBUG("frontier %zd cost: %f", i, frontiers[i].cost);
  }

  if (frontiers.empty()) {
    stop();
    return;
  }
  
  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // find non blacklisted frontier
  auto frontier =
      std::find_if_not(frontiers.begin(), frontiers.end(),
                       [this](const frontier_exploration::Frontier& f) {
                         return goalOnBlacklist(f.centroid);
                       });
  if (frontier == frontiers.end()) {
    stop();
    return;
  }
  
  geometry_msgs::Point target_position;

  if (frontier_type_ == std::string("centroid"))
  {
    target_position = frontier->centroid;
  }
  else if (frontier_type_ == std::string("initial"))
  {
    target_position = frontier->initial;
  }
  else
  {
    target_position = frontier->middle;
  }
  
  // time out if we are not making any progress
  bool same_goal = prev_goal_ == target_position;
  prev_goal_ = target_position;
  if (!same_goal || prev_distance_ > frontier->min_distance) {
    // we have different goal or we made some progress
    last_progress_ = ros::Time::now();
    prev_distance_ = frontier->min_distance;
  }
  // black list if we've made no progress for a long time
  if (ros::Time::now() - last_progress_ > progress_timeout_) {
    frontier_blacklist_.push_back(target_position);
    ROS_DEBUG("Adding current goal to black list");
    makePlan();
    return;
  }

  // we don't need to do anything if we still pursuing the same goal
  if (same_goal) {
    return;
  }

  // send goal to move_base if we have something new to pursue
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.pose.position = target_position;
  goal.target_pose.pose.orientation= 
  	tf::createQuaternionMsgFromYaw(yawOfVector(costmap_client_.getRobotPose().pose.position, goal.target_pose.pose.position));
  goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.target_pose.header.stamp = ros::Time::now();
  move_base_client_.sendGoal(
      goal, [this, target_position](
                const actionlib::SimpleClientGoalState& status,
                const move_base_msgs::MoveBaseResultConstPtr& result) {
        reachedGoal(status, result, target_position);
      });
}

bool Explore_Server::goalOnBlacklist(const geometry_msgs::Point& goal)
{
  constexpr static size_t tolerace = 5;
  costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

void Explore_Server::reachedGoal(const actionlib::SimpleClientGoalState& status,
                          const move_base_msgs::MoveBaseResultConstPtr&,
                          const geometry_msgs::Point& frontier_goal)
{
  ROS_DEBUG("Reached goal with status: %s", status.toString().c_str());
  if (status == actionlib::SimpleClientGoalState::ABORTED) {
    frontier_blacklist_.push_back(frontier_goal);
    ROS_DEBUG("Adding current goal to black list");
  }

  // find new goal immediatelly regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  oneshot_ = relative_nh_.createTimer(
      ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
      true);
}

void Explore_Server::start()
{
  exploring_timer_.start();
}

void Explore_Server::stop()
{
  move_base_client_.cancelAllGoals();
  exploring_timer_.stop();
  ROS_INFO("Exploration stopped.");
}

}  // namespace explore

int main(int argc, char** argv)
{
  ros::init(argc, argv, "explore_server");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  explore_server::Explore_Server eplore_server;
  ros::spin();

  return 0;
}
