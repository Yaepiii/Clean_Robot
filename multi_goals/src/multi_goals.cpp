#include <multi_goals/multi_goals.h>
#include <visualization_msgs/Marker.h>

#include <string>

namespace multi_goals
{
    void MultiGoals::stop()
    {
        move_base_client_.cancelAllGoals();
        nav_timer_.stop();
        ROS_INFO("Navigation Stop.");
    }

    geometry_msgs::Pose MultiGoals::getRobotPose() const
    {
        tf::Stamped<tf::Pose> global_pose;
        global_pose.setIdentity();
        tf::Stamped<tf::Pose> robot_pose;
        robot_pose.setIdentity();
        robot_pose.frame_id_ = robot_base_frame_;
        robot_pose.stamp_ = ros::Time();
        ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

        // get the global pose of the robot
        try
        {
            tf_listener_.transformPose(global_frame_, robot_pose, global_pose);
        }
        catch (tf::LookupException &ex)
        {
            ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot "
                                    "pose: %s\n",
                               ex.what());
            return {};
        }
        catch (tf::ConnectivityException &ex)
        {
            ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n",
                               ex.what());
            return {};
        }
        catch (tf::ExtrapolationException &ex)
        {
            ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n",
                               ex.what());
            return {};
        }
        // check global_pose timeout
        if (current_time.toSec() - global_pose.stamp_.toSec() >
            transform_tolerance_)
        {
            ROS_WARN_THROTTLE(1.0, "Costmap2DClient transform timeout. Current time: "
                                   "%.4f, global_pose stamp: %.4f, tolerance: %.4f",
                              current_time.toSec(), global_pose.stamp_.toSec(),
                              transform_tolerance_);
            return {};
        }

        geometry_msgs::PoseStamped msg;
        tf::poseStampedTFToMsg(global_pose, msg);
        return msg.pose;
    }

    void MultiGoals::vizPubCb()
    {
        visualization_msgs::Marker points, line_strip;

        if (!goals.empty())
        {
            points.header = line_strip.header = goals.front().header;
            points.ns = line_strip.ns = "multi_goals";
            points.lifetime = line_strip.lifetime = ros::Duration();

            points.id = 0;
            line_strip.id = 1;

            points.type = visualization_msgs::Marker::SPHERE_LIST;
            line_strip.type = visualization_msgs::Marker::LINE_STRIP;
            points.action = line_strip.action = visualization_msgs::Marker::ADD;
            points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

            points.scale.x = points.scale.y = 0.1;
            line_strip.scale.x = 0.2;

            for (const auto &point : goals)
            {
                line_strip.points.push_back(point.point);
                points.points.push_back(point.point);
            }
            points.color.a = points.color.g = line_strip.color.g = line_strip.color.a = 1.0;
        }
        else
        {
            points.action = line_strip.action = visualization_msgs::Marker::DELETE;
        }
        // publish points and lines to be viewable in rviz
        point_viz_pub_.publish(points);
        point_viz_pub_.publish(line_strip);
    }

    void MultiGoals::pointCb(const geometry_msgs::PointStamped &point)
    {
        if (goals.size() > 1 && pointsNearby(goals.back().point, point.point, 0.2))
        {
            nav_timer_ = nh_.createTimer(ros::Duration(3.0),
                                         [this](const ros::TimerEvent &)
                                         { SendGoals(); });
            ROS_INFO("Navigation Start!");
        }
        else
        {
            goals.push_back(point);
            ROS_INFO("Please continue to select goals!");
        }
    }

    void MultiGoals::moveBaseResultCb(const actionlib::SimpleClientGoalState &state,
                                      const move_base_msgs::MoveBaseResultConstPtr &result)
    {
        if (state == actionlib::SimpleClientGoalState::ABORTED)
        {
            ROS_ERROR("Failed to move.");
        }
        else if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            std::vector<geometry_msgs::PointStamped>(goals.begin() + 1, goals.end()).swap(goals);

            ROS_INFO("Move base succeeded.");
            retry_ = 5;
            moving_ = false;
        }
        // find new goal immediatelly regardless of planning frequency.
        // execute via timer to prevent dead lock in move_base_client (this is
        // callback for sendGoal, which is called in makePlan). the timer must live
        // until callback is executed.
        oneshot_ = nh_.createTimer(
            ros::Duration(0, 0), [this](const ros::TimerEvent &)
            { SendGoals(); },
            true);
    }

    void MultiGoals::SendGoals()
    {
        if (moving_)
            return;
        if (!goals.empty())
        {
            current_pose_ = getRobotPose();
            geometry_msgs::Point next_position;

            move_base_msgs::MoveBaseGoal goal;

            geometry_msgs::Point target_position = goals[0].point;
            if(goals.size() == 1)
            {
                next_position = target_position;
            }
            else
            {
                next_position = goals[1].point;
                goal_yaw = yawOfVector(target_position, next_position);
            }
            // send goal to move_base if we have something new to pursue
            goal.target_pose.pose.position = target_position;
            goal.target_pose.pose.orientation =
                tf::createQuaternionMsgFromYaw(goal_yaw);
            goal.target_pose.header.frame_id = goals.front().header.frame_id;
            goal.target_pose.header.stamp = ros::Time::now();
            ROS_INFO("Current goals is: %f %f.", target_position.x, target_position.y);
            {
                boost::unique_lock<boost::mutex> lock(move_client_lock_);
                move_base_client_.sendGoal(goal, boost::bind(&MultiGoals::moveBaseResultCb, this, _1, _2), 0, 0);
                moving_ = true;
                lock.unlock();
            }
        }
        else
        {
            // try to get a goal from the planner 5 times but throw an error on the 5th time
            if (retry_ == 0)
            {
                ROS_INFO("No valid goals found, navigation complete.");
                stop();
            }
            else
            {
                ROS_INFO("Failed to get a goal,retry.");
                retry_--;
                MultiGoals::SendGoals();
            }
        }
    }

    MultiGoals::MultiGoals() : nh_(),
                               private_nh_("~"),
                               move_base_client_("move_base"),
                               moving_(false),
                               retry_(5),
                               tf_listener_(ros::Duration(3.0))
    {
        private_nh_.param("global_frame", global_frame_, std::string("map"));
        private_nh_.param("robot_base_frame", robot_base_frame_, std::string("base_link"));
        private_nh_.param("transform_tolerance", transform_tolerance_, 0.3);

        ROS_INFO("Waiting to connect to move_base server");
        move_base_client_.waitForServer();
        ROS_INFO("Connected to move_base server");

        point_ = nh_.subscribe("/clicked_point", 10, &MultiGoals::pointCb, this);
        point_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("multi_goals_marker", 10);
        point_viz_timer_ = nh_.createWallTimer(ros::WallDuration(0.1), boost::bind(&MultiGoals::vizPubCb, this));
        ROS_INFO("Please use the 'Point' tool in Rviz to select Goals.");
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_goals");

    multi_goals::MultiGoals mg;
    ros::spin();
    return 0;
}
