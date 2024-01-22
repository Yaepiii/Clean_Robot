#ifndef MULTI_GOALS_H_
#define MULTI_GOALS_H_

#include <ros/ros.h>

#include <geometry_msgs/PointStamped.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <mutex>
#include <vector>
#include <string>

namespace multi_goals
{

class MultiGoals
{
public:
  /**
   * @brief Constructor for the classs.
   */
  MultiGoals();

  /**
   * @brief Get the pose of the robot in the global frame of the costmap
   * @return pose of the robot in the global frame of the costmap
   */
  geometry_msgs::Pose getRobotPose() const;

  void stop();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  bool moving_;
  int retry_;
  ros::Subscriber point_;
  ros::Publisher point_viz_pub_;
  ros::WallTimer point_viz_timer_;
  tf::TransformListener tf_listener_;

  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_;
  geometry_msgs::Pose current_pose_;

  std::vector<geometry_msgs::PointStamped> goals;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;
  boost::mutex move_client_lock_;
  ros::Timer nav_timer_;
  ros::Timer oneshot_;

  double goal_yaw;

  /**
   * @brief Publish markers for visualization of points for boundary polygon.
   */
  void vizPubCb();

  /**
   * @brief Build multi goals from points received through rviz gui.
   * @param point Received point from rviz
   */
  void pointCb(const geometry_msgs::PointStamped& point);

  /**
   * @brief Method to send goals to move_base
   */
  void SendGoals();

  /**
     * @brief Callback tied to the completion of a move_base task
     * @param state State from the move_base client
     * @param result Result from the move_base client
     */
  void moveBaseResultCb(const actionlib::SimpleClientGoalState& state,
    const move_base_msgs::MoveBaseResultConstPtr& result);


};
  /**
  * @brief Calculate the square of a number
  * @param x number to be squared
  * @return Square
  */
double square(const double x)
{
  return x * x;
}

/**
* @brief Calculate distance between two points
* @param one Point one
* @param two Point two
* @return Distance between two points
*/
template<typename T, typename S>
double pointsDistance(const T& one, const S& two)
{
  return sqrt(square(one.x - two.x) + square(one.y - two.y) + square(one.z - two.z));
}

/**
* @brief Evaluate whether two points are approximately adjacent, within a specified proximity distance.
* @param one Point one
* @param two Point two
* @param proximity Proximity distance
* @return True if approximately adjacent, false otherwise
*/
template<typename T, typename S>
bool pointsNearby(const T& one, const S& two, const double proximity)
{
  return pointsDistance(one, two) <= proximity;
}

/**
* @brief Calculate the yaw of vector defined by origin and end points
* @param origin Origin point
* @param end End point
* @return Yaw angle of vector
*/
template<typename T, typename S>
double yawOfVector(const T &origin, const S &end)
{
  double delta_x, delta_y;
  delta_x = end.x - origin.x;
  delta_y = end.y - origin.y;

  double yaw = atan(delta_y/delta_x);

  if (delta_x < 0)
  {
    yaw = M_PI-yaw;
  }

  return yaw;
}

}
#endif
