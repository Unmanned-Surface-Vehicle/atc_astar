/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <rra_local_planner/rra_planner.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/map_grid_cost_point.h>
#include <cmath>

//for computing path distance
#include <queue>

#include <angles/angles.h>

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>

#include "geometry_msgs/Point.h"

// #include <math.h>       /* atan */

#define PI 3.14159265
#define Kp 0.1
#define Ki 0

#define LINEAR_VEL_CONST                0.075 // Proportional controller gain
#define ANGULAR_VEL_CONST               1.00  // Proportional controller gain
#define COSTMAP_FREE_ACCEPTANCE         1     // value from 0 to 255
#define COSTMAP_OCCUPANCE_ACCEPTANCE    250   // value from 0 to 255
#define POSE_TO_FOLLOW                  15    // 
// #define LOCAL_PATH_MIN_SIZE           00030
#define ARTIFICIAL_TERRAIN_COST_LENGTH  60    // Local costmap units
#define ARTIFICIAL_TERRAIN_COST_WIDTH   36    // Local costmap units

namespace rra_local_planner {

  double euclidian_distance (double goal_x, double goal_y, double current_x, double current_y);
  double linear_vel         (double goal_x, double goal_y, double current_x, double current_y, double constt = 1);
  double angular_vel        (double goal_x, double goal_y, double current_x, double current_y, double self_th, double constt = 1);
  double steering_angle     (double goal_x, double goal_y, double current_x, double current_y);

  void RRAPlanner::reconfigure(RRAPlannerConfig &config)
  {

    boost::mutex::scoped_lock l(configuration_mutex_);

    generator_.setParameters(
        config.sim_time,
        config.sim_granularity,
        config.angular_sim_granularity,
        config.use_rra,
        sim_period_);

    double resolution = planner_util_->getCostmap()->getResolution();
    pdist_scale_ = config.path_distance_bias;
    // pdistscale used for both path and alignment, set  forward_point_distance to zero to discard alignment
    path_costs_.setScale(resolution * pdist_scale_ * 0.5);
    alignment_costs_.setScale(resolution * pdist_scale_ * 0.5);

    gdist_scale_ = config.goal_distance_bias;
    goal_costs_.setScale(resolution * gdist_scale_ * 0.5);
    goal_front_costs_.setScale(resolution * gdist_scale_ * 0.5);

    occdist_scale_ = config.occdist_scale;
    obstacle_costs_.setScale(resolution * occdist_scale_);

    stop_time_buffer_ = config.stop_time_buffer;
    oscillation_costs_.setOscillationResetDist(config.oscillation_reset_dist, config.oscillation_reset_angle);
    forward_point_distance_ = config.forward_point_distance;
    goal_front_costs_.setXShift(forward_point_distance_);
    alignment_costs_.setXShift(forward_point_distance_);
 
    // obstacle costs can vary due to scaling footprint feature
    obstacle_costs_.setParams(config.max_trans_vel, config.max_scaling_factor, config.scaling_speed);

    twirling_costs_.setScale(config.twirling_scale);

    int vx_samp, vy_samp, vth_samp;
    vx_samp = config.vx_samples;
    vy_samp = config.vy_samples;
    vth_samp = config.vth_samples;

    double I = 0;

    if (vx_samp <= 0) {
      ROS_WARN("You've specified that you don't want any samples in the x dimension. We'll at least assume that you want to sample one value... so we're going to set vx_samples to 1 instead");
      vx_samp = 1;
      config.vx_samples = vx_samp;
    }
 
    if (vy_samp <= 0) {
      ROS_WARN("You've specified that you don't want any samples in the y dimension. We'll at least assume that you want to sample one value... so we're going to set vy_samples to 1 instead");
      vy_samp = 1;
      config.vy_samples = vy_samp;
    }
 
    if (vth_samp <= 0) {
      ROS_WARN("You've specified that you don't want any samples in the th dimension. We'll at least assume that you want to sample one value... so we're going to set vth_samples to 1 instead");
      vth_samp = 1;
      config.vth_samples = vth_samp;
    }
 
    vsamples_[0] = vx_samp;
    vsamples_[1] = vy_samp;
    vsamples_[2] = vth_samp;
 

  }

  RRAPlanner::RRAPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util) :
      planner_util_(planner_util),
      obstacle_costs_(planner_util->getCostmap()),
      path_costs_(planner_util->getCostmap()),
      goal_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
      goal_front_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
      alignment_costs_(planner_util->getCostmap())
  {
    ros::NodeHandle private_nh("~/" + name);

    goal_front_costs_.setStopOnFailure( false );
    alignment_costs_.setStopOnFailure( false );

    other_vessel_sub_   = private_nh.subscribe("/diffboat2/state", 1, &RRAPlanner::getOtherVesselOdom_callback, this);
    other_vessel_pos_.x = -1;
    other_vessel_pos_.y = -1;

    //Assuming this planner is being run within the navigation stack, we can
    //just do an upward search for the frequency at which its being run. This
    //also allows the frequency to be overwritten locally.
    std::string controller_frequency_param_name;
    if(!private_nh.searchParam("controller_frequency", controller_frequency_param_name)) {
      sim_period_ = 0.05;
    } else {
      double controller_frequency = 0;
      private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
      if(controller_frequency > 0) {
        sim_period_ = 1.0 / controller_frequency;
      } else {
        ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
        sim_period_ = 0.05;
      }
    }
    ROS_INFO("Sim period is set to %.2f", sim_period_);

    oscillation_costs_.resetOscillationFlags();

    bool sum_scores;
    private_nh.param("sum_scores", sum_scores, false);
    obstacle_costs_.setSumScores(sum_scores);


    private_nh.param("publish_cost_grid_pc", publish_cost_grid_pc_, false);
    map_viz_.initialize(name, planner_util->getGlobalFrame(), boost::bind(&RRAPlanner::getCellCosts, this, _1, _2, _3, _4, _5, _6));

    std::string frame_id;
    private_nh.param("global_frame_id", frame_id, std::string("odom"));

    traj_cloud_ = new pcl::PointCloud<base_local_planner::MapGridCostPoint>;
    traj_cloud_->header.frame_id = frame_id;
    traj_cloud_pub_.advertise(private_nh, "trajectory_cloud", 1);
    private_nh.param("publish_traj_pc", publish_traj_pc_, false);

    // set up all the cost functions that will be applied in order
    // (any function returning negative values will abort scoring, so the order can improve performance)
    std::vector<base_local_planner::TrajectoryCostFunction*> critics;
    critics.push_back(&oscillation_costs_); // discards oscillating motions (assisgns cost -1)
    critics.push_back(&obstacle_costs_); // discards trajectories that move into obstacles
    critics.push_back(&goal_front_costs_); // prefers trajectories that make the nose go towards (local) nose goal
    critics.push_back(&alignment_costs_); // prefers trajectories that keep the robot nose on nose path
    critics.push_back(&path_costs_); // prefers trajectories on global path
    critics.push_back(&goal_costs_); // prefers trajectories that go towards (local) goal, based on wave propagation
    critics.push_back(&twirling_costs_); // optionally prefer trajectories that don't spin

    // trajectory generators
    std::vector<base_local_planner::TrajectorySampleGenerator*> generator_list;
    generator_list.push_back(&generator_);

    scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlanner(generator_list, critics);

    private_nh.param("cheat_factor", cheat_factor_, 1.0);
  }

  // used for visualization only, total_costs are not really total costs
  bool RRAPlanner::getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost) {

    path_cost = path_costs_.getCellCosts(cx, cy);
    goal_cost = goal_costs_.getCellCosts(cx, cy);
    occ_cost = planner_util_->getCostmap()->getCost(cx, cy);
    if (path_cost == path_costs_.obstacleCosts() ||
        path_cost == path_costs_.unreachableCellCosts() ||
        occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return false;
    }

    double resolution = planner_util_->getCostmap()->getResolution();
    total_cost =
        pdist_scale_ * resolution * path_cost +
        gdist_scale_ * resolution * goal_cost +
        occdist_scale_ * occ_cost;
    return true;
  }

  bool RRAPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    oscillation_costs_.resetOscillationFlags();
    return planner_util_->setPlan(orig_global_plan);
  }

  /**
   * This function is used when other strategies are to be applied,
   * but the cost functions for obstacles are to be reused.
   */
  bool RRAPlanner::checkTrajectory(
      Eigen::Vector3f pos,
      Eigen::Vector3f vel,
      Eigen::Vector3f vel_samples){
    oscillation_costs_.resetOscillationFlags();
    base_local_planner::Trajectory traj;
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf::getYaw(goal_pose.pose.orientation));
    base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
    generator_.initialise(pos,
        vel,
        goal,
        &limits,
        vsamples_);
    generator_.generateTrajectory(pos, vel, vel_samples, traj);
    double cost = scored_sampling_planner_.scoreTrajectory(traj, -1);
    //if the trajectory is a legal one... the check passes
    if(cost >= 0) {
      return true;
    }
    ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vel_samples[0], vel_samples[1], vel_samples[2], cost);

    //otherwise the check fails
    return false;
  }


  void RRAPlanner::updatePlanAndLocalCosts(
      tf::Stamped<tf::Pose> global_pose,
      const std::vector<geometry_msgs::PoseStamped>& new_plan,
      const std::vector<geometry_msgs::Point>& footprint_spec) {
    global_plan_.resize(new_plan.size());
    for (unsigned int i = 0; i < new_plan.size(); ++i) {
      global_plan_[i] = new_plan[i];
    }

    obstacle_costs_.setFootprint(footprint_spec);

    // costs for going away from path
    path_costs_.setTargetPoses(global_plan_);

    // costs for not going towards the local goal as much as possible
    goal_costs_.setTargetPoses(global_plan_);

    // alignment costs
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();

    Eigen::Vector3f pos(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), tf::getYaw(global_pose.getRotation()));
    double sq_dist =
        (pos[0] - goal_pose.pose.position.x) * (pos[0] - goal_pose.pose.position.x) +
        (pos[1] - goal_pose.pose.position.y) * (pos[1] - goal_pose.pose.position.y);

    // we want the robot nose to be drawn to its final position
    // (before robot turns towards goal orientation), not the end of the
    // path for the robot center. Choosing the final position after
    // turning towards goal orientation causes instability when the
    // robot needs to make a 180 degree turn at the end
    std::vector<geometry_msgs::PoseStamped> front_global_plan = global_plan_;
    double angle_to_goal = atan2(goal_pose.pose.position.y - pos[1], goal_pose.pose.position.x - pos[0]);
    front_global_plan.back().pose.position.x = front_global_plan.back().pose.position.x +
      forward_point_distance_ * cos(angle_to_goal);
    front_global_plan.back().pose.position.y = front_global_plan.back().pose.position.y + forward_point_distance_ *
      sin(angle_to_goal);

    goal_front_costs_.setTargetPoses(front_global_plan);
    
    // keeping the nose on the path
    if (sq_dist > forward_point_distance_ * forward_point_distance_ * cheat_factor_) {
      double resolution = planner_util_->getCostmap()->getResolution();
      alignment_costs_.setScale(resolution * pdist_scale_ * 0.5);
      // costs for robot being aligned with path (nose on path, not ju
      alignment_costs_.setTargetPoses(global_plan_);
    } else {
      // once we are close to goal, trying to keep the nose close to anything destabilizes behavior.
      alignment_costs_.setScale(0.0);
    }
  }

  /*
   * given the current state of the robot, find a good trajectory
   */
  base_local_planner::Trajectory RRAPlanner::findBestPath(
      tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      tf::Stamped<tf::Pose>& drive_velocities) {

    //make sure that our configuration doesn't change mid-run
    boost::mutex::scoped_lock l(configuration_mutex_);

    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    // Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf::getYaw(goal_pose.pose.orientation));
    base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();

    // ROS_INFO("global_plan size: %d", (int) global_plan_.size());
    // ROS_INFO("first: (%f, %f) last: (%f, %f))", 
    //   global_plan_.back().pose.position.x, 
    //   global_plan_.back().pose.position.y, 
    //   global_plan_.front().pose.position.x, 
    //   global_plan_.front().pose.position.y);

    // ROS_INFO("Costmap size: %d x %d", planner_util_->getCostmap()->getSizeInCellsX(), planner_util_->getCostmap()->getSizeInCellsY());
    // ROS_INFO("Costmap resolution: %f ", planner_util_->getCostmap()->getResolution());

    // Converts Costmap to graph to be used in the A* method
    GridWithWeights* graph = costmapToGrid( planner_util_->getCostmap() );

    // Auxiliaries for world to map coordinates tranform
    int mx, my;

    // Artificial Terrain Cost for COLREGS-COMPLIANCE if is there any other vessel near
    if (isThereAnyOtherVesselNear())
    {
      colregs_encounter_type risk = identifyCOLREGSEncounterType();

      // Artificial Terrain Cost for COLREGS-COMPLIANCE
      geometry_msgs::Point diff2_pos;
      // diff2_pos.x = 6;
      // diff2_pos.y = 4;
      diff2_pos = other_vessel_pos_;
      other_vessel_pos_.x = -1;
      other_vessel_pos_.y = -1;

      std::vector<geometry_msgs::Point> artificial_terrain = createArtificialTerrainCost(diff2_pos);

      // std::cout << "Artificial Terrain: " << std::endl;
      // for (auto pos = artificial_terrain.end()-1; pos != artificial_terrain.begin(); pos--)
      // {
      //   std::cout << "( " << pos->x << ", " << pos->y << " )" << std::endl;
      // }

      for (auto pos = artificial_terrain.begin(); pos != artificial_terrain.end(); pos++)
      {

        planner_util_->getCostmap()->worldToMapEnforceBounds( pos->x, 
                                                              pos->y, 
                                                              mx, 
                                                              my);

        // ROS_INFO("Before set (%d, %d): %d", mx, my, planner_util_->getCostmap()->getCost(mx, my));
        // planner_util_->getCostmap()->setCost(mx, my, (unsigned char)255);
        // ROS_INFO("After set (%d, %d): %d", mx, my, planner_util_->getCostmap()->getCost(mx, my));

        Pos auxPosi;
        auxPosi.x = mx;
        auxPosi.y = my;
        double cost = COSTMAP_OCCUPANCE_ACCEPTANCE;
        auxPosi.cost = cost;
        // graph->walls.insert(auxPosi);
        graph->forests.insert(auxPosi);

      }

    }

    // Creates data structures to be populated in the A*
    std::unordered_map<Pos, Pos>    came_from;                                // Path
    std::unordered_map<Pos, double> cost_so_far;                              // A*'s exploration phase util    

    // Gets closer global plan position in local frame reference
    Pos static astar_goal{-1, -1}, aux;
    planner_util_->getCostmap()->worldToMapEnforceBounds(   goal_pose.pose.position.x, 
                                                            goal_pose.pose.position.y, 
                                                            mx, 
                                                            my);
    aux.x = mx;
    aux.y = my;

    Pos current_pos;
    
    planner_util_->getCostmap()->worldToMapEnforceBounds(   global_pose.getOrigin().getX(), 
                                                            global_pose.getOrigin().getY(), 
                                                            mx, 
                                                            my);    
    current_pos.x = mx;
    current_pos.y = my;
    
    std::vector<geometry_msgs::Point> local_path_at_global_frame;

    if ( isAStarGoalValid(aux) ){ // new valid A* goal
      astar_goal = aux;
      // Gets robot current position in local frame reference

      // ROS_INFO("A* goal:    (%f, %f)", (double) astar_goal.x, (double) astar_goal.y);
      // ROS_INFO("Robot pos:  (%f, %f)", (double) current_pos.x, (double) current_pos.y);
      AStar::AStar astar;                                                                 // A* handler
      astar.AStarSearch(*(graph), current_pos, astar_goal, came_from, cost_so_far);       // A* method execution
      // std::cout << "Reconstructing path" << std::endl;
      std::vector<Pos> local_path = astar.reconstruct_path(current_pos, astar_goal, came_from); // Util for easier path use
      // std::cout << "Finished path reconstruction" << std::endl;
      // std::vector<geometry_msgs::Point> local_path_at_global_frame;

      // ROS_INFO("Local path:");
      // for (auto pos = local_path.begin(); pos != local_path.end(); pos++)
      // {
      //   std::cout << "( " << pos->x << ", " << pos->y << ")->";
      // }
      // std::cout << std::endl;


      // std::cout << "Converting local plan from local coordinations to global cordenates" << std::endl;
      for (auto pos = local_path.begin(); pos != local_path.end(); pos++)
      {

        geometry_msgs::Point global_pos;
        // // (Xg*resolution + Xcostmap, Yg*resolution + Ycostmap)
        planner_util_->getCostmap()->mapToWorld(  pos->x, 
                                                  pos->y, 
                                                  global_pos.x, 
                                                  global_pos.y);
        local_path_at_global_frame.push_back(global_pos);

      }
      // std::cout << "Finished conversion" << std::endl;

      // ROS_INFO("Local path at global frame:");
      // for (auto pos = local_path_at_global_frame.begin(); pos != local_path_at_global_frame.end(); pos++)
      // {
      //   std::cout << "( " << pos->x << ", " << pos->y << ")->";
      // }
      // std::cout << std::endl;

      std::cout << "Drawing cenario" << std::endl;
      draw_grid(*graph, 2, nullptr, nullptr, &local_path);
      std::cout << "Finished drawing" << std::endl;

      result_traj_.cost_ = 12;

      // Populating result_traj_ for debugging propose
      result_traj_.resetPoints();
      for (auto p = local_path_at_global_frame.begin(); p != local_path_at_global_frame.end(); p++)
      {
        result_traj_.addPoint(p->x, p->y, steering_angle(p->x, p->y, global_pose.getOrigin().getX(), global_pose.getOrigin().getX()));
      }

      Eigen::Vector3f pos(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), tf::getYaw(global_pose.getRotation()));
      // debrief stateful scoring functions
      oscillation_costs_.updateOscillationFlags(pos, &result_traj_, planner_util_->getCurrentLimits().min_trans_vel);

      

      // if we don't have a legal trajectory, we'll just command zero
      // if (result_traj_.cost_ < 0) {
      //   drive_velocities.setIdentity();
      // } else {
      //   tf::Vector3 start(result_traj_.xv_, result_traj_.yv_, 0);
      //   drive_velocities.setOrigin(start);
      //   tf::Matrix3x3 matrix;
      //   matrix.setRotation(tf::createQuaternionFromYaw(result_traj_.thetav_));
      //   drive_velocities.setBasis(matrix);
      // }

    } else{

      result_traj_.cost_ = -7;
      
    }
    // else if ( (astar_goal.x == -1) && (astar_goal.y == -1) ) // New A* goal is not valid. Is last A* goal invalid?
    // {
    //   result_traj_.cost_ = -7;
    //   result_traj_.resetPoints();
    //   drive_velocities.setIdentity();
    //   return result_traj_;
    // }

    // Creates cmd_vel populating drive_velocities with translation and rotation matrixes
    unsigned short int local_path_index_to_follow = 0;
    local_path_index_to_follow = local_path_at_global_frame.size() >= POSE_TO_FOLLOW ? POSE_TO_FOLLOW -1 : local_path_at_global_frame.size() -1;

    // ROS_INFO("POSE %d IN PATH: (%f, %f))", local_path_index_to_follow, local_path_at_global_frame[local_path_index_to_follow].x, local_path_at_global_frame[local_path_index_to_follow].y);

    // local_path_index_to_follow = local_path_at_global_frame.size() < LOCAL_PATH_MIN_SIZE ? (local_path_at_global_frame.size() -1) : (local_path_at_global_frame.size() -1)/2;    
    

    if (result_traj_.cost_ < 0)
    {

      drive_velocities.setIdentity();

    } else {

      tf::Vector3 start(
                        linear_vel(
                                    local_path_at_global_frame[local_path_index_to_follow].x, 
                                    local_path_at_global_frame[local_path_index_to_follow].y, 
                                    global_pose.getOrigin().getX(), 
                                    global_pose.getOrigin().getY(), 
                                    LINEAR_VEL_CONST
                                  ), 
                        0, 
                        0);
      drive_velocities.setOrigin(start);

      tf::Matrix3x3 matrix;
      matrix.setRotation(
                          tf::createQuaternionFromYaw(
                                                      angular_vel(local_path_at_global_frame[local_path_index_to_follow].x, 
                                                      local_path_at_global_frame[local_path_index_to_follow].y, global_pose.getOrigin().getX(), 
                                                      global_pose.getOrigin().getY(), 
                                                      tf::getYaw(global_pose.getRotation()), 
                                                      ANGULAR_VEL_CONST)
                                                      )
                        );
      drive_velocities.setBasis(matrix);

    }

    return result_traj_;
  }

  double euclidian_distance(double goal_x, double goal_y, double self_x, double self_y){
    return sqrt(  pow((goal_x - self_x), 2) + 
                  pow((goal_y - self_y), 2));
  }

  // As outlined in Section 4.1.1, a simple heading proportional
  // controller based on Bertaska2015Experimental and Go to Goal ROS move_base tutorial
  double linear_vel(double goal_x, double goal_y, double self_x, double self_y, double constt){
    return constt * euclidian_distance(goal_x, goal_y, self_x, self_y);
  }

  double angular_vel(double goal_x, double goal_y, double self_x, double self_y, double self_th, double constt){
    return constt * (steering_angle(goal_x, goal_y, self_x, self_y) - self_th);

    // double err = steering_angle(goal_pos, self_x, self_y) - self_th;
    // double P = Kp * err;
    // I += Ki * err;

    // return P + I;yy
  }

  double steering_angle(double goal_x, double goal_y, double self_x, double self_y){
    double angle = atan2( goal_y - self_y, goal_x - self_x);
  
    if (angle >= PI)
    {
      angle = angle - 2 * PI;
    }

    // ROS_INFO("Wanted pos: (%f, %f)\nCurrent pos: (%f, %f)\nSteering Angle: %f", goal_x, goal_y, self_x, self_y, angle);

    return angle;
    // return atan2((double) goal_pos.y, (double) goal_pos.x) - atan2((double) goal_pos.y, (double) goal_pos.x);
  }

  GridWithWeights* RRAPlanner::costmapToGrid(costmap_2d::Costmap2D *costmap){

    Pos auxPosi;
    GridWithWeights *grid_p = new GridWithWeights(costmap->getSizeInCellsX(), costmap->getSizeInCellsY());

    // ROS_INFO("Costmap size: %d x %d", costmap->getSizeInCellsX(), costmap->getSizeInCellsY());

    for (size_t j = 0; j < costmap->getSizeInCellsY(); j++)
    {
      for (size_t i = 0; i < costmap->getSizeInCellsX(); i++)
      {
        if ( double(costmap->getCost(i, j)) > COSTMAP_OCCUPANCE_ACCEPTANCE)
        {
          auxPosi.x = i;
          auxPosi.y = j;
          grid_p->walls.insert(auxPosi);
          ROS_INFO("WALL (%d, %d) - cost: %f", i, j, double(costmap->getCost(i, j)));
        }else if ( double(costmap->getCost(i, j)) > COSTMAP_FREE_ACCEPTANCE)
        {
          auxPosi.x = i;
          auxPosi.y = j;
          double cost = double(costmap->getCost(i, j));
          auxPosi.cost = cost;
          grid_p->forests.insert(auxPosi);
          // ROS_INFO("FOREST (%d, %d) - cost: %f, cost: %f", i, j, auxPosi.cost, cost);
        } 
      }
    }

    return grid_p;

  };

  bool RRAPlanner::isAStarGoalValid(Pos astar_goal){

    return planner_util_->getCostmap()->getCost(astar_goal.x, astar_goal.y) <= COSTMAP_OCCUPANCE_ACCEPTANCE;

  }

  std::vector<geometry_msgs::Point> RRAPlanner::createArtificialTerrainCost(geometry_msgs::Point otherVesselPos)  {

    std::vector<geometry_msgs::Point> artificial_terrain  ;

    geometry_msgs::Point pos  ;

    short int width, length, x_offset;

    // Hardcoded ajustments
    width     = ARTIFICIAL_TERRAIN_COST_WIDTH/2;
    length    = ARTIFICIAL_TERRAIN_COST_LENGTH +2;
    x_offset  = 1; 

    for (short int j = - width/2; j < width/2; j++)
    {

      for (unsigned short int i = 1; i <= length; i++)
      { 
        double res = planner_util_->getCostmap()->getResolution();
        pos.x = otherVesselPos.x + j*res - x_offset*res;
        pos.y = otherVesselPos.y + i*res;
        artificial_terrain.push_back(pos);
      }

    }
    
    

    return artificial_terrain;

  }

  void RRAPlanner::getOtherVesselOdom_callback(const nav_msgs::Odometry::ConstPtr& usv_position_msg){

    other_vessel_pos_ = usv_position_msg->pose.pose.position;
    // ROS_INFO("USV current position: X: %f, Y: %f", usv_current_pos.x, usv_current_pos.y);
    // ROS_INFO("C_x - N_x: %f - %f --- C_y - N_y: %f - %f", usv_current_pos.x, next_goal.x, usv_current_pos.y, next_goal.y);

  }

  // probably could be improved calling only "wordToMap"
  bool RRAPlanner::isThereAnyOtherVesselNear(){
    
    if ( (other_vessel_pos_.x != -1) && (other_vessel_pos_.y != -1)) // Is there any publishing
    {
      unsigned int dummy_unsigned_int_x, dummy_unsigned_int_y;
      // identify if other vessel is near (inside local costmap region)
      return planner_util_->getCostmap()->worldToMap(other_vessel_pos_.x, other_vessel_pos_.y, dummy_unsigned_int_x, dummy_unsigned_int_y);
    }
  
  }

  colregs_encounter_type RRAPlanner::identifyCOLREGSEncounterType(){

    return HeadOn;
  }

};
