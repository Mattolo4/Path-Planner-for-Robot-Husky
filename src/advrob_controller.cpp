#include <advrob_controller/advrob_controller.h>
#include <tf2/utils.h>
#include <cmath>
#include <visualization_msgs/Marker.h>
#include <limits>

#include <base_local_planner/goal_functions.h>
#include <base_local_planner/trajectory.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>

using namespace ros;
using namespace std;

// Add a publisher for the marker
ros::NodeHandle nh_;
ros::Publisher marker_pub;


PLUGINLIB_EXPORT_CLASS(advrob_controller::AdvRobController, nav_core::BaseLocalPlanner)

namespace advrob_controller{

        AdvRobController::AdvRobController() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

        AdvRobController::AdvRobController(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
         : costmap_ros_(NULL), tf_(NULL), initialized_(false){
                initialize(name, tf, costmap_ros);
         }
         

        AdvRobController::~AdvRobController() {}

        void AdvRobController::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){

          if(!initialized_){
            tf_ = tf;
            costmap_ros_ = costmap_ros;
            first_ = true;
          
            // Edited because of the case of hard turns before the goal:
            // in the case of turning just before gtting to the goal the robot can miss
            // but with this settings it stops any case.
            goal_tolerance_ = 0.05 * 2;
            visualization_step_size_ = 0.05;

            ros::NodeHandle n;

            odom_sub_ = n.subscribe("/ground_truth/state", 2, &AdvRobController::odomCallback, this);
            path_pub_ = n.advertise<nav_msgs::Path>("driven_path", 10);

            // Try to obtain the shift from the center of the robot to the front axle 
            // in the initialize since we need to compute this only once
            try{
              geometry_msgs::TransformStamped front = tf_->lookupTransform("front_right_wheel_link", "base_link_robot",  ros::Time(0));
              shift_ = abs(front.transform.translation.x);
              //cout << "X-Shift: " << shift_ << endl;
            }catch (tf2::TransformException &ex) {
              ROS_ERROR("Transform error: %s", ex.what());
            }

            // To handle the collision detection
            costmap_ = costmap_ros_->getCostmap();

            // Initalizing time to compute delta_time
            last_time = Time::now();
            initialized_ = true;
          }

        }

        bool AdvRobController::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
          if(!initialized_){
            ROS_ERROR("This controller has not been initialized, please call initialize() before using this controller");
            return false;
          }
          //reset the global plan
          global_plan_.clear();
          global_plan_ = orig_global_plan;

          //reset the at goal flag
          reached_goal_ = false;
          driven_path_.clear();
          addOdomPoseToPath();

          return true;
        }

        void AdvRobController::addOdomPoseToPath(){
            last_odom_ = odom_;
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.pose = odom_.pose.pose;
            driven_path_.push_back(pose);
            publishPath();
            printStats();
        }

        double AdvRobController::normalize_angle(double angle) {
          return atan2(sin(angle), cos(angle));
        }

        // Debugging function added in order to visualize: trajectory, tangent, points, poses,..
        void AdvRobController::publishMarker(const geometry_msgs::Pose& pose, ros::Publisher& marker_pub) {
            if (!marker_pub) {
                marker_pub = nh_.advertise<visualization_msgs::Marker>("marker_pose", 10);
            }
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "marker_pose";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose = pose;
            marker.scale.x = 1; 
            marker.scale.y = 0.1; 
            marker.scale.z = 0.1; 

            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
            marker_pub.publish(marker);
        }

        // Check if the predicted pose encounter an o bstacle
        bool AdvRobController::isCollisionFree(const geometry_msgs::Pose& pose){

          // Retrieve footprint informations from costmap
          vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
          // Transform footprint to the robot's current pose
          vector<geometry_msgs::Point> transformed_footprint;
          for (const auto& point : footprint){
            geometry_msgs::Point transformed_point;
            transformed_point.x = pose.position.x + point.x * cos(tf2::getYaw(pose.orientation)) - point.y * sin(tf2::getYaw(pose.orientation));
            transformed_point.y = pose.position.y + point.x * sin(tf2::getYaw(pose.orientation)) + point.y * cos(tf2::getYaw(pose.orientation));
            transformed_footprint.push_back(transformed_point);
          }

          // Check each point of the transformed footprint in the costmap
          for(const auto& point : transformed_footprint){
            unsigned int mx, my; // Future cell coordinates
            
            // Converting world coords to map
            if(!costmap_->worldToMap(point.x, point.y, mx, my)){ // Successfull transformation
              
              return false; // Point is out of costmap bounds;
            }
            // Retrieve the cost of that cell (mx, my)
            unsigned char cost = costmap_->getCost(mx, my);

            // Check if the point is in a free cell or not - HARD CHECK: the robot will move only 
            // in cells with value = 0 (aka free)
            /*if((cost == costmap_2d::LETHAL_OBSTACLE || 
                cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
                cost == costmap_2d::NO_INFORMATION) &&
                cost != costmap_2d::FREE_SPACE){
              
              return false; // Collision/anomaly detected
            }*/
            // More relaxed contraints: stops if it touches and inflation or an obstacle
            if(cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
              return false; // Collision/anomaly detected
            }
          }
          return true;  // No collision detected
        }

        bool AdvRobController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
          if(!initialized_){
            ROS_ERROR("This controller has not been initialized, please call initialize() before using this controller");
            return false;
          }
          // Task 1.1 - Lateral Control

          /*The control gain k determines the influence of the cross-track error on 
          the steering angle. A higher value makes the robot more responsive to cross-track 
          errors but may cause oscillations. If the robot oscillates, reduce k. 
          If the robot is too slow in correcting its path, increase k*/
          const double K         = 1.25;      // Control gain
          // Velocity paramter introduced to improve steering precision at low speed
          const double Ks        = 0.5;       
          /*If the velocity is too high, the robot might overshoot the path. If it’s too low, 
          the robot might not reach the goal in a reasonable amount of time.*/
          // Reference velocity
          const double VELOCITY  = 0.7;       // 0.710011 printing twist linear.x
          /*If this value is too small, the robot might react too quickly to changes in the path. 
          If it’s too large, the robot might not react quickly enough. */
          const int SEGMENT_SIZE = 15;        // segment size to find tangent
          double MAX_ANG_VEL     = 1.1;       // 1.12305 printing twist angular.z  while rotating the robot manually on rviz
          //cout << "Vel: "<< odom_.twist.twist << endl; 
          const double ACC_MAX   = 0.3;       // Max acceleration for the robot
          const double ACC_MIN   = -ACC_MAX;  // Max acceleration for the robot

          // PI controller state  
          const double Kp = 1.3;  // Proportional gain 
          const double Ki = 0.5;  // Integral gain  
          double integral_error = 0.0;

          // put true to print real time stats
          const bool debug = false;
          

          if(!global_plan_.empty()){

            // Compute the midpoint between the front wheels ont hte axle
            geometry_msgs::Pose front_axle = odom_.pose.pose;
            front_axle.position.x += (shift_ * cos(tf2::getYaw(odom_.pose.pose.orientation))); 
            front_axle.position.y += (shift_ * sin(tf2::getYaw(odom_.pose.pose.orientation))); 
            if (debug) publishMarker(front_axle, marker_pub);

            // Find the closest point on the path
            double crossT_error = numeric_limits<double>::max();
            size_t idx;
            geometry_msgs::Pose closest_pose;

            for(size_t i=0; i<global_plan_.size(); ++i){
              geometry_msgs::Pose pose = global_plan_[i].pose;
              double d = computeDistance(pose, front_axle);
              if(d < crossT_error){
                crossT_error = d;   // Cross-track error
                closest_pose = pose; 
                idx = i;
              }
            }
            if (debug) cout << "Cross T error: " << crossT_error << endl;
            if (debug) cout << "Closest point: " << idx << "/" << global_plan_.size() <<  endl;

       
            // DEtermine PSI: heading wrt to the trajectory angle 
            // determine the tangent of the trajectory at the point closest to the vehicle
            size_t start_idx = (idx >= SEGMENT_SIZE) ? idx - SEGMENT_SIZE : 0;
            size_t end_idx   = std::min(idx + SEGMENT_SIZE, global_plan_.size() - 1);
            double dx = global_plan_[end_idx].pose.position.x - global_plan_[start_idx].pose.position.x;
            double dy = global_plan_[end_idx].pose.position.y - global_plan_[start_idx].pose.position.y;
            double tangent_angle = normalize_angle(atan2(dy, dx));
            // To plot the tangent direction
            tf2::Quaternion q;
            q.setRPY(0, 0, tangent_angle);
            closest_pose.orientation = tf2::toMsg(q);
            if (debug) publishMarker(closest_pose, marker_pub);  // Publishing the tangent to trajectory
            if (debug) cout << "tangent_angle: " << tangent_angle << endl;

            // Compute the heading error
            double robot_angle = normalize_angle(tf2::getYaw(odom_.pose.pose.orientation));
            double psi = normalize_angle(tangent_angle - robot_angle); 
            if (debug) cout << "Heading error: "<< psi  << endl;

            // Compute the signed cross-track error in order to keep track the relative position of the robot
            // with the respect to the planned path: if robot is on the right wrt the path (* +1), (* -1) otherwise
            double dx_robot = front_axle.position.x - closest_pose.position.x;
            double dy_robot = front_axle.position.y - closest_pose.position.y;
            double cross_track_error_signed = crossT_error * (dx_robot * dy - dy_robot * dx >= 0 ? 1.0 : -1.0);
            if (debug) cout << "Signed CrossT: "<< cross_track_error_signed  << endl;

            double linear_x = odom_.twist.twist.linear.x;
            double linear_y = odom_.twist.twist.linear.y;
            double linear_z = odom_.twist.twist.linear.z;
            double linear_velocity_magnitude = sqrt(linear_x * linear_x + linear_y * linear_y + linear_z * linear_z);
            current_velocity_ = linear_velocity_magnitude;
            if (debug) cout << "Current velocity:" << current_velocity_ << endl;

            // Compute the velocities control
            double steering = normalize_angle(psi + atan2(K * cross_track_error_signed, Ks + current_velocity_));

            // Clamping the steering value between robot mechanical limit
            steering = max(-MAX_ANG_VEL, min(MAX_ANG_VEL, steering));
            if (debug) cout << "Steering: " << steering << endl << endl;


            // ---------- LONGITUDINAL CONTROL ---------- // 
            // Compute the velocities control
            
            // Velocity error 
            double e_v      = VELOCITY - current_velocity_;
            // Delta time  intra-iterations
            Time current_t  = Time::now();
            double delta_t  = (double(current_t.toSec()) - double(last_time.toSec()));

            // Constraint to clamp the delta_t in [0.01, 1.0] seconds range,
            // it cannot be < 0.5 (about), due to the local planner frequency (20Hz)
            // and preventing it to consider a delta_t too high, since we initialize it in the apposite function,
            // If the NavGoal is set after some time, delta_t will count the milliseconds elapsed from the starting node moment 
            // to the actual goal sending on rviz, for the first iteration 
            delta_t = max(0.01, min(delta_t, 1.));  
            if (debug) cout << "DeltaTime: " << delta_t << endl;

            integral_error += e_v * delta_t;
            if (debug) cout << "integral_error: " << integral_error << endl;

            double acceleration = Kp * e_v + Ki * integral_error;

            // Clamp acceleration within contrained boundaries
            acceleration = max(ACC_MIN, min(ACC_MAX, acceleration));
            if (debug) cout << "Acceleration: " << acceleration << endl;

            // Compute the new velocity
            double velocity = current_velocity_ + acceleration * delta_t;   // V = vi + a * delta_t
            velocity = max(0.1, min(velocity, VELOCITY));   // Ensure velocity is non-negative and within max limit
            if (debug) cout << "velocity: " << velocity << endl; 


            // ----------- Check for collisions ----------- //

            // Get the predicted pose after applying the velocity command
            geometry_msgs::Pose predicted_pose = odom_.pose.pose;
            predicted_pose.position.x += cmd_vel.linear.x * cos(tf2::getYaw(predicted_pose.orientation)) * delta_t;
            predicted_pose.position.y += cmd_vel.linear.x * sin(tf2::getYaw(predicted_pose.orientation)) * delta_t;
            
            // If the predicted pose ends up in some non-free cells the collision is detected
            if (!isCollisionFree(predicted_pose)){
              ROS_WARN("Collision detected, stopping the robot!");
              cmd_vel.linear.x = 0;
              cmd_vel.angular.z = 0;
              return false;
            }

            cmd_vel.linear.x  = velocity;
            cmd_vel.angular.z = steering;
            last_time         = current_t;
          }else{
            ROS_WARN("No plan generated yet!");
          }


          // check if goal is reached
          double d = computeDistance(global_plan_.back().pose, odom_.pose.pose);
          if (debug) cout << "Goal distance: " << d << endl;
          if (d < goal_tolerance_){
            reached_goal_ = true;
            // Stop the robot
            cmd_vel.linear.x  = 0.0;
            cmd_vel.angular.z = 0.0;
          }


          // keep this for visualization purposes and basic statistics
          if (computeDistance(last_odom_.pose.pose, odom_.pose.pose) >= visualization_step_size_){
            addOdomPoseToPath();
          }
          
          return true;
        }

        bool AdvRobController::isGoalReached(){
          if(!initialized_){
            ROS_ERROR("This controller has not been initialized, please call initialize() before using this controller");
            return false;
          }
          return reached_goal_;
        }

        void AdvRobController::publishPath(){
          nav_msgs::Path path;
          path.header.stamp = ros::Time::now();
          path.header.frame_id = odom_.header.frame_id;
          path.poses = driven_path_;

          path_pub_.publish(path);

        }

        void AdvRobController::printStats(){
          double driving_time = driven_path_[driven_path_.size() - 1].header.stamp.toSec() - driven_path_[0].header.stamp.toSec();
          double path_dist = computePathLength(driven_path_);
          double avg_vel = driving_time == 0.0 ? 0.0 : path_dist / driving_time;
          ROS_INFO("\nplan length: %f, driven length: %f, driving time: %f, avg vel: %f, Hausdorff: %f\n", computePathLength(global_plan_), path_dist, driving_time, 
          avg_vel, computeHausdorffDistance(global_plan_, driven_path_));
        }

        void AdvRobController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
          odom_ = *msg;
          if (first_){
            last_odom_ = odom_;
            first_ = false;
          }
        }

        double AdvRobController::computeDistance(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2){
          return std::sqrt(std::pow(pose1.position.x - pose2.position.x, 2) + std::pow(pose1.position.y - pose2.position.y, 2));
        }


        double AdvRobController::computeHausdorffDistance(std::vector<geometry_msgs::PoseStamped>& A, std::vector<geometry_msgs::PoseStamped>& B){
          if (B.size() == 0 || B.size() == 0){
            ROS_WARN("Hausdorff distance cannot be computed. At least one element is required for each list");
            return 0.0;
          }

          double max_dist = 0.0;
          for (const auto& a : A) {
              double min_dist = std::numeric_limits<double>::max();
              for (const auto& b : B) {
                  double dist = computeDistance(a.pose, b.pose);
                  if (dist < min_dist) {
                      min_dist = dist;
                  }
              }
              if (min_dist > max_dist) {
                  max_dist = min_dist;
              }
          }

          for (const auto& b : B){    // also consider the max distance of the other path (in case of different size)
              double min_dist = std::numeric_limits<double>::max();
              for (const auto& a : A) {
                  double dist = computeDistance(a.pose, b.pose);
                  if (dist < min_dist) {
                      min_dist = dist;
                  }
              }
              if (min_dist > max_dist) {
                  max_dist = min_dist;
              }
          }
          return max_dist;
        }

        double AdvRobController::computePathLength(std::vector<geometry_msgs::PoseStamped>& path){
            if (path.size() < 2){
              return 0.0;
            }
            geometry_msgs::PoseStamped last_pose = path[0];
            double length = 0.0;
            
            for (size_t i = 1; i < path.size(); i++){
              length += computeDistance(last_pose.pose, path[i].pose);
              last_pose = path[i];
            }
            return length;
        }
}