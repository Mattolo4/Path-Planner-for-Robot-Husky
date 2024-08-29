#ifndef LOCAL_PLANNER_H_
#define LOCAL_PLANNER_H_

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/costmap_model.h>
#include <tf2_ros/buffer.h>
#include <ros/ros.h>


#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/Odometry.h>

using namespace std;

namespace advrob_controller{



  class AdvRobController : public nav_core::BaseLocalPlanner{

    public:
      AdvRobController();

      AdvRobController(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

      ~AdvRobController();

      void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
      void publishMarker(const geometry_msgs::Pose& pose, ros::Publisher& marker_pub);
      double normalize_angle(double angle);
      bool isCollisionFree(const geometry_msgs::Pose& pose);


      bool isGoalReached();

      void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    private:

      void publishPath();

      void printStats();

      double computeDistance(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2);

      void addOdomPoseToPath();

      /*
      * computeHausdorffDistance
      * Arg1: A - a list of points
      * Arg2: B - a list of points
      * ret: the hausdorff distance
      * 
      * The Hausdorff distance is a mathematical concept used to measure the distance 
      * or dissimilarity between two subsets of a metric space. In simple terms, the Hausdorff distance is the maximum 
      * distance between any point in one set and its closest point in the other set. In other words, it is the largest 
      * value that a "matching" between the points of the two sets can take, where a "matching" pairs each point of one 
      * set with a point of the other set.
      */
      double computeHausdorffDistance(std::vector<geometry_msgs::PoseStamped>& A, std::vector<geometry_msgs::PoseStamped>& B);

      double computePathLength(std::vector<geometry_msgs::PoseStamped>& path);

      double goal_tolerance_;
      double visualization_step_size_;

      // front axle x-shift from "base_link" reference framce
      double shift_; //0.255999
      double current_velocity_;


      costmap_2d::Costmap2DROS* costmap_ros_;
      costmap_2d::Costmap2D* costmap_;

      tf2_ros::Buffer* tf_;
      bool initialized_;
      bool reached_goal_;

      ros::Subscriber odom_sub_;
      nav_msgs::Odometry odom_;
      nav_msgs::Odometry last_odom_;
      bool first_;

      ros::Time last_time;

      ros::Publisher path_pub_;

      std::vector<geometry_msgs::PoseStamped> global_plan_;
      std::vector<geometry_msgs::PoseStamped> driven_path_;

  };
};

#endif