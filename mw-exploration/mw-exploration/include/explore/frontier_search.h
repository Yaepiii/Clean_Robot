#ifndef FRONTIER_SEARCH_H_
#define FRONTIER_SEARCH_H_

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

namespace frontier_exploration
{
/**
 * @brief Represents a frontier
 */
struct Frontier {
  double size;
  double min_distance;
  double information;
  double cost;
  geometry_msgs::Point initial;
  geometry_msgs::Point centroid;
  geometry_msgs::Point middle;
  std::vector<geometry_msgs::Point> points;
  double orientation;
};

/**
 * @brief Thread-safe implementation of a frontier-search task for an input
 * costmap.
 */
class FrontierSearch
{
public:
  FrontierSearch()
  {
  }

  /**
   * @brief Constructor for search task
   * @param costmap Reference to costmap data to search.
   * @param potential_scale Weight related to minimum distance 
   * @param gain_scale Weight related to frontier size
   * @param information_scale Weight related to information gain
   * @param min_frontier_size Consider the frontier as the minimum frontier size
   * @param information_r Information gain detection radius
   */
  FrontierSearch(costmap_2d::Costmap2D* costmap, double potential_scale,
                 double gain_scale,double information_scale,double orientation_scale,
                 double min_frontier_size, double information_r);

  /**
   * @brief Runs search implementation, outward from the start position
   * @param pose Initial pose to search from
   * @return List of frontiers, if any
   */
  std::vector<Frontier> searchFrom(geometry_msgs::Pose pose);

  /**
   * @brief Dynamic reconfigure parameter update
   */
  void configUpdate(double min_frontier_size, double potential_scale,
                    double gain_scale, double information_scale, double orientation_scale, double information_r);

protected:
  /**
   * @brief Starting from an initial cell, build a frontier from valid adjacent
   * cells
   * @param initial_cell Index of cell to start frontier building
   * @param reference Reference index to calculate position from
   * @param frontier_flag Flag vector indicating which cells are already marked
   * as frontiers
   * @return new frontier
   */
  Frontier buildNewFrontier(unsigned int initial_cell, unsigned int reference, geometry_msgs::Pose pose,
                            std::vector<bool>& frontier_flag);

  /**
   * @brief isNewFrontierCell Evaluate if candidate cell is a valid candidate
   * for a new frontier.
   * @param idx Index of candidate cell
   * @param frontier_flag Flag vector indicating which cells are already marked
   * as frontiers
   * @return true if the cell is frontier cell
   */
  bool isNewFrontierCell(unsigned int idx,
                         const std::vector<bool>& frontier_flag);

  /**
   * @brief computes frontier cost
   * @details cost function is defined by potential_scale and gain_scale
   *
   * @param frontier frontier for which compute the cost
   * @return cost of the frontier
   */
  double frontierCost(const Frontier& frontier);

  /**
   * @brief compute information gain
   * @param point Frontier for which compute the information gain
   * @param r Information gain radius
   * @return information gain of the frontier
   */
  double informationGain(geometry_msgs::Point point, double r);

  /**
   * @brief Calculate the yaw of vector defined by origin and end points
   * @param origin Origin point
   * @param end End point
   * @return Yaw angle of vector
   */
  template<typename T, typename S>
  double solveYaw(const T& origin, const S& end)
  {
      double delta_x, delta_y;
      delta_x = end.x - origin.x;
      delta_y = end.y - origin.y;

      double yaw = atan(delta_x / delta_y);

      if (delta_x < 0)
      {
          yaw = M_PI - yaw;
      }

      return yaw;
  }

private:
  costmap_2d::Costmap2D* costmap_;
  unsigned char* map_;
  unsigned int size_x_, size_y_;
  double potential_scale_, gain_scale_, information_scale_, orientation_scale_;
  double min_frontier_size_;
  double information_r_;
  double h_rad = 0.5, h_gain = 1.0;
  double min_distance, max_information, min_orientation, max_size;
};
}
#endif
