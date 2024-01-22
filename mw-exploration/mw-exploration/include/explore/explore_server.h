#ifndef NAV_EXPLORE_H_
#define NAV_EXPLORE_H_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <exploration_msgs/ExploreAction.h>
#include <exploration_msgs/SetPolygon.h>

#include <explore/costmap_client.h>
#include <explore/frontier_search.h>

#include <dynamic_reconfigure/server.h>
#include <mw-exploration/ExploreConfig.h>

namespace explore_server
{
/**
 * @class Explore
 * @brief A class adhering to the robot_actions::Action interface that moves the
 * robot base to explore its environment.
 */
class Explore_Server
{
public:
  Explore_Server();
  ~Explore_Server();

  void start();
  void stop();

private:
  typedef actionlib::ActionServer<exploration_msgs::ExploreAction> ExploreActionServer;
  typedef ExploreActionServer::GoalHandle GoalHandle;

  // ros
  ros::NodeHandle private_nh_;
  ros::NodeHandle relative_nh_;
  ros::Publisher marker_array_publisher_;
  tf::TransformListener tf_listener_;

  // Self-defining
  ExploreActionServer explore_action_server_;
  exploration_msgs::ExploreFeedback feedback_;
  Costmap2DClient costmap_client_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
      move_base_client_;
  boost::mutex move_client_lock_;
  frontier_exploration::FrontierSearch search_;
  ros::Timer exploring_timer_;
  ros::Timer oneshot_;

  std::vector<geometry_msgs::Point> frontier_blacklist_;
  geometry_msgs::Point prev_goal_;      ///<@brief Previous frontier goal
  double prev_distance_;                ///<@brief Previous distance from goal
  ros::Time last_progress_;             ///<@brief Previous time
  size_t last_markers_count_;           ///<@brief markers count
  geometry_msgs::PointStamped explore_center_;  ///<@brief Search starting point
  geometry_msgs::PolygonStamped polygon_;

  GoalHandle active_gh_;
  
  // related to dynamic_reconfigure
  dynamic_reconfigure::Server<ExploreConfig> configServer;
  dynamic_reconfigure::Server<ExploreConfig>::CallbackType cb;

  // parameters
  double planner_frequency_;
  double potential_scale_, gain_scale_, information_scale_, orientation_scale_;
  double information_r_;
  ros::Duration progress_timeout_;
  bool visualize_;
  std::string frontier_type_;
  double timeout;
  double min_frontier_size_;

  /**
   * @brief  Make a global plan
   */
  void makePlan();

  /**
   * @brief Callback for initially starting off the exploration, starts up the
   * planner plugin and requests first movement goal for the robot then sends
   * that goal to move_base.
   * @param gh GoalHandle which includes reference to the goal which has the
   * name of the planner plugin and the inital boundary polygon
   */
  void goalCb(GoalHandle gh);

  /**
   * @brief Method to cancel the running goal
   * @param gh GoalHandle of the goal to be cancelled
   */
  void cancelGoalCb(GoalHandle gh);

  /**
   * @brief  Publish a frontiers as markers
   */
  void visualizeFrontiers(
      const std::vector<frontier_exploration::Frontier>& frontiers);

  /**
   * @brief  Callback to arrive the goal 
   */
  void reachedGoal(const actionlib::SimpleClientGoalState& status,
                   const move_base_msgs::MoveBaseResultConstPtr& result,
                   const geometry_msgs::Point& frontier_goal);

  /**
   * @brief  Method to put the arrived goal on blacklist  
   */
  bool goalOnBlacklist(const geometry_msgs::Point& goal);
  
  /**
   * @brief  Callback of dynamic_reconfigure
   */
  void configCb(ExploreConfig& config, uint32_t level);
 
};
}

#endif
