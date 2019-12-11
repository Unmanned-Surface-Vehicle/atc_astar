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

// #include <math.h>       /* atan */

#define PI 3.14159265
#define Kp 0.1
#define Ki 0

int   mapSize;
bool  *occupancyGridMap;

// Cost of non connected nodes
float infinity = std::numeric_limits<float>::infinity();

namespace rra_local_planner {

  double euclidian_distance (Pos pos, double x, double y);
  double linear_vel         (Pos pos, double x, double y, double constt = 1);
  double angular_vel        (Pos pos, double x, double y, double self_th, double constt = 1);
  double steering_angle     (Pos pos, double x, double y);


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

    Eigen::Vector3f pos(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), tf::getYaw(global_pose.getRotation()));
    Eigen::Vector3f vel(global_vel.getOrigin().getX(), global_vel.getOrigin().getY(), tf::getYaw(global_vel.getRotation()));
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    // Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf::getYaw(goal_pose.pose.orientation));
    base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();

    std::vector<geometry_msgs::PoseStamped> plan;

    originX     = planner_util_->getCostmap()->getOriginX() + 0.7;
    originY     = planner_util_->getCostmap()->getOriginY() + 0.9;
    ROS_INFO("Origin: (%f, %f)", originX, originY);

    width       = planner_util_->getCostmap()->getSizeInCellsX();
    height      = planner_util_->getCostmap()->getSizeInCellsY();
    resolution  = planner_util_->getCostmap()->getResolution();
    mapSize     = width * height;

    ROS_INFO("WidthxHeight: %d x %d", width, height);
    ROS_INFO("Res: %f Size: %d", resolution, mapSize);

    occupancyGridMap = new bool[mapSize];
    for (unsigned int iy = 0; iy < planner_util_->getCostmap()->getSizeInCellsY(); iy++)
    {
      for (unsigned int ix = 0; ix < planner_util_->getCostmap()->getSizeInCellsX(); ix++)
      {
        unsigned int cost = static_cast<int>(planner_util_->getCostmap()->getCost(ix, iy));

        if (cost == 0)
          occupancyGridMap[iy * width + ix] = true;
        else
          occupancyGridMap[iy * width + ix] = false;
      }
    }

    // Convert the start and goal positions
    float startX = global_pose.getOrigin().getX();
    float startY = global_pose.getOrigin().getY();
    float goalX = goal_pose.pose.position.x;
    float goalY = goal_pose.pose.position.y;

    ROS_INFO("Start: (%f, %f)", startX, startY);
    ROS_INFO("Goal: (%f, %f)", goalX, goalY);

    // Convert to map coordinates relative to costmap origin
    convertToMapCoordinates(startX, startY);
    convertToMapCoordinates(goalX, goalY);

    int startGridSquare;
    int goalGridSquare;

    if (isCoordinateInBounds(startX, startY) && isCoordinateInBounds(goalX, goalY))
    {
      startGridSquare = getGridSquareIndex(startX, startY);
      goalGridSquare = getGridSquareIndex(goalX, goalY);
    }
    else
    {
      ROS_WARN("(A* Planner) The start or goal is out of the map!");
    }

    // Call global planner
    if (isStartAndGoalValid(startGridSquare, goalGridSquare))
    {
      std::vector<int> bestPath;
      bestPath.clear();

      // Runs planner
      bestPath = runAStarOnGrid(startGridSquare, goalGridSquare);

      // Check if planner found a path
      if (bestPath.size() > 0)
      {
        // Convert the path
        for (int i = 0; i < bestPath.size(); i++)
        {

          float x = 0.0;
          float y = 0.0;

          float previous_x = 0.0;
          float previous_y = 0.0;

          int index = bestPath[i];
          int previous_index;
          getGridSquareCoordinates(index, x, y);

          if (i != 0)
          {
            previous_index = bestPath[i - 1];
          }
          else
          {
            previous_index = index;
          }

          getGridSquareCoordinates(previous_index, previous_x, previous_y);

          // Orient the robot towards target
          tf::Vector3 vectorToTarget;
          vectorToTarget.setValue(x - previous_x, y - previous_y, 0.0);
          float angle = atan2((double)vectorToTarget.y(), (double)vectorToTarget.x());

          geometry_msgs::PoseStamped pose = goal_pose;

          pose.pose.position.x = x;
          pose.pose.position.y = y;
          pose.pose.position.z = 0.0;

          pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);

          plan.push_back(pose);
        }
      }
      else
      {
        ROS_WARN("(A* Planner) Failed to find a path, choose another goal position!");
      }
    } else
    {
      ROS_WARN("(A* Planner) Not valid start or goal!");
    }

    // ROS_INFO("global_plan size: %d", global_plan_.size());
    // ROS_INFO("first: (%f, %f) last: (%f, %f))",
    //   global_plan_.back().pose.position.x,
    //   global_plan_.back().pose.position.y,
    //   global_plan_.front().pose.position.x,
    //   global_plan_.front().pose.position.y);

    // Converts Costmap to graph to be used in the A* method
    // GridWithWeights* graph = costmapToGrid( planner_util_->getCostmap() );

    // Creates data structures to be populated in the A*
    // std::unordered_map<Pos, Pos>    came_from;                                // Path
    // std::unordered_map<Pos, double> cost_so_far;                              // A*'s exploration phase util

    // Gets closer global plan position
    // Pos astar_goal;
    // astar_goal.x = goal_pose.pose.position.x;
    // astar_goal.y = goal_pose.pose.position.y;
    // Gets robot current position
    // Pos current_pos;
    // current_pos.x = global_pose.getOrigin().getX();
    // current_pos.y = global_pose.getOrigin().getY();

    // A*
    // ROS_INFO("A* goal:    (%f, %f)", (double) astar_goal.x, (double) astar_goal.y);
    // ROS_INFO("Robot pos:  (%f, %f)", (double) current_pos.x, (double) current_pos.y);
    // AStar::AStar astar;                                                                 // A* handler
    // astar.AStarSearch(*(graph), current_pos, astar_goal, came_from, cost_so_far);       // A* method execution
    // std::std::vector<Pos> path = astar.reconstruct_path(current_pos, astar_goal, came_from); // Util for easier path use

    if (global_plan_.size() > 0)
    {
      result_traj_.cost_ = 12;
    }else{
      result_traj_.cost_ = -7;
    }

    // result_traj_.cost_ = 12;                                                            // Legacy behaviour maintence

    // for (auto pos = path.begin(); pos != path.end(); pos++)
    // {
    //   // std::cout << "( " << pos->x << ", " << pos->y << ")->" << std::endl;
    // }

    // Populating result_traj_ for debugging propose
    // result_traj_.resetPoints();
    // for (auto p = path.begin(); p != path.end(); p++)
    // {
    //   Pos tempPos;
    //   tempPos.x = p->x;
    //   tempPos.y = p->y;
    //   result_traj_.addPoint(p->x, p->y, steering_angle(tempPos, current_pos.x, current_pos.y));
    // }

    // Populating result_traj_ for debugging propose
    result_traj_.resetPoints();
    for (auto p = plan.begin(); p != plan.end(); p++)
    {
      Pos tempPos;
      tempPos.x = p->pose.position.x;
      tempPos.y = p->pose.position.y;
      // result_traj_.addPoint(p->x, p->y, steering_angle(tempPos, current_pos.x, current_pos.y));
      result_traj_.addPoint(p->pose.position.x, p->pose.position.y, steering_angle(tempPos, global_pose.getOrigin().getX(), global_pose.getOrigin().getY()));
    }

    // debrief stateful scoring functions
    // oscillation_costs_.updateOscillationFlags(pos, &result_traj_, planner_util_->getCurrentLimits().min_trans_vel);

    //if we don't have a legal trajectory, we'll just command zero
    if (result_traj_.cost_ < 0) {
      drive_velocities.setIdentity();
    } else {
      Pos goal;
      goal.x = global_plan_.back().pose.position.x;
      goal.y = global_plan_.back().pose.position.y;  
      // goal.x = plan.back().pose.position.x;
      // goal.y = plan.back().pose.position.y;
      // goal.x = path[path_index].x;
      // goal.y = path[path_index].y;
      
      // ROS_INFO("Current: (%f, %f) Goal: (%f, %f)", (double) current_pos.x, (double) current_pos.y, (double) goal.x, (double) goal.y);
      tf::Vector3 start(linear_vel(goal, global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), 0.1), 0, 0);
      drive_velocities.setOrigin(start);

      tf::Matrix3x3 matrix;
      matrix.setRotation(tf::createQuaternionFromYaw(angular_vel(goal, global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), tf::getYaw(global_pose.getRotation()), 1)));
      drive_velocities.setBasis(matrix);

    }

    return result_traj_;
  }

  double euclidian_distance(Pos goal_pos, double self_x, double self_y){
    return sqrt(  pow(((double) goal_pos.x - self_x), 2) + 
                  pow(((double) goal_pos.y - self_y), 2));
  }

  // As outlined in Section 4.1.1, a simple heading proportional
  // controller was utilized Bertaska2015Experimental and Go to Goal ROS move_base tutorial
  double linear_vel(Pos goal_pos, double self_x, double self_y, double constt){
    return constt * euclidian_distance(goal_pos, self_x, self_y);
  }

  double angular_vel(Pos goal_pos, double self_x, double self_y, double self_th, double constt){
    return constt * (steering_angle(goal_pos, self_x, self_y) - self_th);

    // double err = steering_angle(goal_pos, self_x, self_y) - self_th;
    // double P = Kp * err;
    // I += Ki * err;

    // return P + I;yy
  }

  double steering_angle(Pos goal_pos, double self_x, double self_y){
    double angle = atan2((double) goal_pos.y - self_y, (double) goal_pos.x - self_x);

    if (angle > PI)
    {
      angle = angle - 2 * PI;
    }

    // ROS_INFO("Wanted pos: (%f, %f)\nCurrent pos: (%f, %f)\nSteering Angle: %f", goal_pos.x, goal_pos.y, self_x, self_y, angle);

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
        if ( costmap->getCost(i, j) > 1)
        {
          auxPosi.x = i;
          auxPosi.y = j;
          grid_p->walls.insert(auxPosi);
          // ROS_INFO("Pos (%d, %d) - cost: %f", i, j, (double) costmap->getCost((int)i, (int)j));
        }
      }
    }

    return grid_p;

  };

  /**
    Adjust start and goal regarding origin point on map
  **/
  void RRAPlanner::convertToMapCoordinates(float &x, float &y)
  {
    x = x - originX;
    y = y - originY;
  }

  /**
    Get index of grid square on map given square coordinates
  **/
  int RRAPlanner::getGridSquareIndex(float x, float y)
  {
    int gridSquare;

    float newX = x / (resolution);
    float newY = y / (resolution);

    gridSquare = calculateGridSquareIndex(newY, newX);

    return gridSquare;
  }

  /**
    Get gridSquare coordinates given index
  **/
  void RRAPlanner::getGridSquareCoordinates(int index, float &x, float &y)
  {
    x = getGridSquareColIndex(index) * resolution;

    y = getGridSquareRowIndex(index) * resolution;

    x = x + originX;
    y = y + originY;
  }

  /**
    Check if gridSquare coordinates are in map bounds
  **/
  bool RRAPlanner::isCoordinateInBounds(float x, float y)
  {
    bool valid = true;

    if (x > (width * resolution) || y > (height * resolution))
    {
      valid = false;

      if(x > (width * resolution)){
        ROS_INFO("%f > %f", x, width * resolution);
      }else if(y > (height * resolution)){
        ROS_INFO("%f > %f", y, height * resolution);
      }

    }

    return valid;
  }

  /**
    Runs A* algorithm to find best path to goal on grid
  **/
  std::vector<int> RRAPlanner::runAStarOnGrid(int startGridSquare, int goalGridSquare)
  {
    std::vector<int> bestPath;

    // Initialize g_score matrix with infinity for every point
    float g_score[mapSize];
    for (uint i = 0; i < mapSize; i++)
    {
      g_score[i] = infinity;
    }

    // Call method for finding path
    bestPath = findPath(startGridSquare, goalGridSquare, g_score);

    return bestPath;
  }

  /**
    Generates the path for the bot towards the goal
  **/
  std::vector<int> RRAPlanner::findPath(int startGridSquare, int goalGridSquare, float g_score[])
  {
    // value++;
    std::vector<int> bestPath;
    std::vector<int> emptyPath;
    GridSquare gridSq;

    std::multiset<GridSquare> openSquaresList;
    int currentGridSquare;

    // Calculate g_score and f_score of the start position
    g_score[startGridSquare] = 0;
    gridSq.currentGridSquare = startGridSquare;
    gridSq.fCost = g_score[startGridSquare] + calculateHScore(startGridSquare, goalGridSquare);

    // Add the start gridSquare to the open list
    openSquaresList.insert(gridSq);
    currentGridSquare = startGridSquare;

    // While the open list is not empty and till goal square is reached continue the search
    while (!openSquaresList.empty() && g_score[goalGridSquare] == infinity)
    {
      // Choose the gridSquare that has the lowest cost fCost in the open set
      currentGridSquare = openSquaresList.begin()->currentGridSquare;

      // Remove that gridSquare from the openList
      openSquaresList.erase(openSquaresList.begin());

      // Search the neighbors of that gridSquare
      std::vector<int> neighborGridSquares;
      neighborGridSquares = findFreeNeighborGridSquare(currentGridSquare);
      for (uint i = 0; i < neighborGridSquares.size(); i++) // For each neighbor of gridSquare
      {
        // If the g_score of the neighbor is equal to INF: unvisited gridSquare
        if (g_score[neighborGridSquares[i]] == infinity)
        {
          g_score[neighborGridSquares[i]] = g_score[currentGridSquare] + getMoveCost(currentGridSquare, neighborGridSquares[i]);
          addNeighborGridSquareToOpenList(openSquaresList, neighborGridSquares[i], goalGridSquare, g_score);
        }
      }
    }

    if (g_score[goalGridSquare] != infinity) // If goal gridSquare has been reached
    {
      bestPath = constructPath(startGridSquare, goalGridSquare, g_score);
      return bestPath;
    }
    else
    {
      ROS_INFO("(A* Planner) Failure to find a path!");
      return emptyPath;
    }
  }

  /**
    Constructs the path found by findPath function by returning std::vector of gridSquare indices that lie on path
  **/
  std::vector<int> RRAPlanner::constructPath(int startGridSquare, int goalGridSquare, float g_score[])
  {
    std::vector<int> bestPath;
    std::vector<int> path;

    path.insert(path.begin() + bestPath.size(), goalGridSquare);
    int currentGridSquare = goalGridSquare;

    while (currentGridSquare != startGridSquare)
    {
      std::vector<int> neighborGridSquares;
      neighborGridSquares = findFreeNeighborGridSquare(currentGridSquare);

      std::vector<float> gScoresNeighbors;
      for (uint i = 0; i < neighborGridSquares.size(); i++)
        gScoresNeighbors.push_back(g_score[neighborGridSquares[i]]);

      int posMinGScore = distance(gScoresNeighbors.begin(), min_element(gScoresNeighbors.begin(), gScoresNeighbors.end()));
      currentGridSquare = neighborGridSquares[posMinGScore];

      // Insert the neighbor in the path
      path.insert(path.begin() + path.size(), currentGridSquare);
    }

    for (uint i = 0; i < path.size(); i++)
    {
      bestPath.insert(bestPath.begin() + bestPath.size(), path[path.size() - (i + 1)]);
    }

    return bestPath;
  }

  /**
    Add unexplored neighbors of currentGridSquare to openlist
  **/
  void RRAPlanner::addNeighborGridSquareToOpenList(std::multiset<GridSquare> &openSquaresList, int neighborGridSquare, int goalGridSquare, float g_score[])
  {
    GridSquare gridSq;
    gridSq.currentGridSquare = neighborGridSquare; //insert the neighborGridSquare
    gridSq.fCost = g_score[neighborGridSquare] + calculateHScore(neighborGridSquare, goalGridSquare);
    openSquaresList.insert(gridSq);
  }

  /**
    Find free neighbors of currentGridSquare 
  **/
  std::vector<int> RRAPlanner::findFreeNeighborGridSquare(int gridSquare)
  {
    int rowIndex = getGridSquareRowIndex(gridSquare);
    int colIndex = getGridSquareColIndex(gridSquare);
    int neighborIndex;
    std::vector<int> freeNeighborGridSquares;

    for (int i = -1; i <= 1; i++)
      for (int j = -1; j <= 1; j++)
      {
        // Check whether the index is valid
        if ((rowIndex + i >= 0) && (rowIndex + i < height) && (colIndex + j >= 0) && (colIndex + j < width) && (!(i == 0 && j == 0)))
        {
          neighborIndex = ((rowIndex + i) * width) + (colIndex + j);

          if (isFree(neighborIndex))
            freeNeighborGridSquares.push_back(neighborIndex);
        }
      }

    return freeNeighborGridSquares;
  }

  /**
    Checks if start and goal positions are valid and not unreachable.
  **/
  bool RRAPlanner::isStartAndGoalValid(int startGridSquare, int goalGridSquare)
  {
    bool isvalid = true;
    bool isFreeStartGridSquare = isFree(startGridSquare);
    bool isFreeGoalGridSquare = isFree(goalGridSquare);

    if (startGridSquare == goalGridSquare)
    {
      isvalid = false;
      ROS_INFO("startGridSquare equal to goalGridSquare");
    }
    else
    {
      if (!isFreeStartGridSquare && !isFreeGoalGridSquare)
      {
        isvalid = false;
        if (!isFreeStartGridSquare)
        {
          ROS_INFO("Start not free");
        }
        else if (!isFreeGoalGridSquare)
        {
          ROS_INFO("Goal not free");
        }
        

      }
      else
      {
        if (!isFreeStartGridSquare)
        {
          isvalid = false;
          ROS_INFO("Start not free");          
        }
        else
        {
          if (!isFreeGoalGridSquare)
          {
            isvalid = false;
            ROS_INFO("Goal not free");
          }
          else
          {
            // if (findFreeNeighborGridSquare(goalGridSquare).size() == 0)
            // {
            //   isvalid = false;
            //   ROS_INFO("No free Neighbor from Goal");
            // }
            // else
            // {
              if (findFreeNeighborGridSquare(startGridSquare).size() == 0)
              {
                isvalid = false;
                ROS_INFO("No free Neighbor from Start");
              }
            // }
          }
        }
      }
    }

    return isvalid;
  }

  /**
    Calculate cost of moving from currentGridSquare to neighbour
  **/
  float RRAPlanner::getMoveCost(int i1, int j1, int i2, int j2)
  {
    // Start cost with maximum value
    float moveCost = infinity;
    
    // If gridSquare(i2,j2) exists in the diagonal of gridSquare(i1,j1)
    if ((j2 == j1 + 1 && i2 == i1 + 1) || (i2 == i1 - 1 && j2 == j1 + 1) || (i2 == i1 - 1 && j2 == j1 - 1) || (j2 == j1 - 1 && i2 == i1 + 1))
    {
      moveCost = 1.4;
    }
    // If gridSquare(i2,j2) exists in the horizontal or vertical line with gridSquare(i1,j1)
    else
    {
      if ((j2 == j1 && i2 == i1 - 1) || (i2 == i1 && j2 == j1 - 1) || (i2 == i1 + 1 && j2 == j1) || (i1 == i2 && j2 == j1 + 1))
      {
        moveCost = 1;
      }
    }

    return moveCost;
  }

  /**
    Calculate cost of moving from currentGridSquare to neighbour
  **/
  float RRAPlanner::getMoveCost(int gridSquareIndex1, int gridSquareIndex2)
  {
    int i1 = 0, i2 = 0, j1 = 0, j2 = 0;

    i1 = getGridSquareRowIndex(gridSquareIndex1);
    j1 = getGridSquareColIndex(gridSquareIndex1);
    i2 = getGridSquareRowIndex(gridSquareIndex2);
    j2 = getGridSquareColIndex(gridSquareIndex2);

    return getMoveCost(i1, j1, i2, j2);
  }

  /**
    Calculate H-Score
  **/
  float RRAPlanner::calculateHScore(int gridSquareIndex, int goalGridSquare)
  {
    int x1 = getGridSquareRowIndex(goalGridSquare);
    int y1 = getGridSquareColIndex(goalGridSquare);
    int x2 = getGridSquareRowIndex(gridSquareIndex);
    int y2 = getGridSquareColIndex(gridSquareIndex);
    return abs(x1 - x2) + abs(y1 - y2);
  }

  /**
    Calculates the gridSquare index from square coordinates
  **/
  int RRAPlanner::calculateGridSquareIndex(int i, int j)
  {
    return (i * width) + j;
  }

  /**
    Calculates gridSquare row from square index
  **/
  int RRAPlanner::getGridSquareRowIndex(int index) //get the row index from gridSquare index
  {
    return index / width;
  }

  /**
    Calculates gridSquare column from square index
  **/
  int RRAPlanner::getGridSquareColIndex(int index) //get column index from gridSquare index
  {
    return index % width;
  }

  /**
    Checks if gridSquare at (i,j) is free
  **/
  bool RRAPlanner::isFree(int i, int j)
  {
    int gridSquareIndex = (i * width) + j;

    return occupancyGridMap[gridSquareIndex];
  }

  /**
    Checks if gridSquare at index gridSquareIndex is free
  **/
  bool RRAPlanner::isFree(int gridSquareIndex)
  {
    return occupancyGridMap[gridSquareIndex];
  }

};

/**
  Operator for comparing cost among two gridSquares.
**/
bool operator<(GridSquare const &c1, GridSquare const &c2) { return c1.fCost < c2.fCost; }
