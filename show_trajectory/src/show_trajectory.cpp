#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "show_trajectory");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("trajectory", 10);
  ros::Rate r(10);
  tf::TransformListener listener;

  std::string global_frame_;
  std::string robot_base_frame_;

  n.param("global_frame", global_frame_, std::string("map"));
  n.param("robot_base_frame", robot_base_frame_, std::string("base_link"));

  while (!ros::ok())
  {
    r.sleep();
  }

  visualization_msgs::Marker points, line_strip;
  points.header.frame_id = line_strip.header.frame_id = global_frame_;
  points.header.stamp = line_strip.header.stamp = ros::Time::now();
  points.ns = line_strip.ns = "show_trajectory";
  points.action = line_strip.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

  points.id = 0;
  line_strip.id = 1;

  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;

  line_strip.scale.x = 0.3;

  line_strip.color.r = 1.0;
  line_strip.color.a = 0.7;

  float x(0), y(0);
  int cnt(0);

  while (ros::ok())
  {
    tf::StampedTransform transform;
    try
    {
      ros::Time now = ros::Time::now();
      listener.waitForTransform(global_frame_, robot_base_frame_,
                                now, ros::Duration(3.0)); // 等待3.0s,判断是否有可用的transform
      listener.lookupTransform(global_frame_, robot_base_frame_,
                               now, transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
    x = transform.getOrigin().x();
    y = transform.getOrigin().y();

    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0.1;

    if (cnt > 1)
    {
      line_strip.points.push_back(p);
    }
    else
    {
      cnt++;
    }

    marker_pub.publish(line_strip);
    r.sleep();
  }
}
