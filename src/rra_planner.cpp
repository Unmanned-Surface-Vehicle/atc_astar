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

namespace rra_local_planner {

  double euclidian_distance (double goal_x, double goal_y, double current_x, double current_y);
  double linear_vel         (double goal_x, double goal_y, double current_x, double current_y, double constt = 1);
  // double angular_vel        (double goal_x, double goal_y, double current_x, double current_y, double self_th, double constt = 1);
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

    other_vessel_sub_     = private_nh.subscribe("/diffboat2/state", 1, &RRAPlanner::getOtherVesselOdom_callback, this);
    other_vessel_pose_.position.x   = -1;
    other_vessel_pose_.position.y   = -1;

    last_astar_goal_.x    = -1;
    last_astar_goal_.y    = -1;

    pid_I_linear_   = 0;
    pid_I_angular_  = 0;

    distance_pub_ = private_nh.advertise<std_msgs::Float64>("distance", 1);
    tcpa_pub_ = private_nh.advertise<std_msgs::Float64>("t_cpa", 1);
    dcpa_pub_ = private_nh.advertise<std_msgs::Float64>("d_cpa", 1);

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


  /* Given both own vessel's and other vessel's current position and 
   * velocity vector, this routine evaluate the collision risk index (CRI)
   * thorugh closest point of approach (CPA) technique, which is calculated
   * as follows:
   * 
   * if || va - vb || <= 0.35 than tcpa = 0
   * else tcpa = ((pa - pb)x(va - vb))/||va - vb||^2
   * dcpa = ||(pa + va * tcpa) - (pb - vb * tcpa) ||
   * if tcpa <=20 and dcpa <= 9 than there is risk of collision and the routine
   * returns TRUE.
   */

  bool RRAPlanner::cpaCollisionRiskIndex(void)
  {
    std_msgs::Float64 msg;
    bool riskOfCollision = false;
    double norm_va_vb = 0;
    double dot_product = 0;
    double pos_result_x = 0;
    double pos_result_y = 0;
    double vel_result_x = 0;
    double vel_result_y = 0;
    double t_cpa, d_cpa = 0;
    double own_vessel_component_x = 0;
    double own_vessel_component_y = 0;
    double other_vessel_component_x = 0;
    double other_vessel_component_y = 0;

    norm_va_vb = euclidian_distance(global_vel_.position.x, 
                                    global_vel_.position.y, 
                                    other_vessel_vel_.linear.x, 
                                    other_vessel_vel_.linear.y);

    if(norm_va_vb > 0.35)
    {
      pos_result_x = global_pose_.position.x - other_vessel_pose_.position.x;
      pos_result_y = global_pose_.position.y - other_vessel_pose_.position.y;

      vel_result_x = global_vel_.position.x - other_vessel_vel_.linear.x;
      vel_result_y = global_vel_.position.y - other_vessel_vel_.linear.y;

      dot_product = (pos_result_x * vel_result_x) + (pos_result_y * vel_result_y);

      t_cpa = fabs(dot_product / pow(norm_va_vb, 2));
    }
    else
    {
      t_cpa = 0;
    }

    own_vessel_component_x = global_pose_.position.x + global_vel_.position.x * t_cpa;
    own_vessel_component_y = global_pose_.position.y + global_vel_.position.y * t_cpa;
    
    other_vessel_component_x = other_vessel_pose_.position.x + other_vessel_vel_.linear.x * t_cpa;
    other_vessel_component_y = other_vessel_pose_.position.y + other_vessel_vel_.linear.y * t_cpa;

    d_cpa = euclidian_distance(other_vessel_component_x, other_vessel_component_y, own_vessel_component_x, own_vessel_component_y);

    // ROS_INFO("Positions:");
    // ROS_INFO("pa: (%f, %f)", global_pose_.position.x, global_pose_.position.y);
    // ROS_INFO("pb: (%f, %f)\n", other_vessel_pose_.position.x, other_vessel_pose_.position.y);
    // ROS_INFO("Velocities");
    // ROS_INFO("va (%f, %f)", global_vel_.position.x, global_vel_.position.y);
    // ROS_INFO("vb (%f, %f)\n", other_vessel_vel_.linear.x, other_vessel_vel_.linear.y);
    // ROS_INFO("Results");
    // ROS_INFO("pa - pb: (%f, %f)", pos_result_x, pos_result_y);
    // ROS_INFO("va - vb: (%f, %f)", vel_result_x, vel_result_y);
    // ROS_INFO("dot_product: %f", dot_product);
    // ROS_INFO("norm_va_vb: %f", norm_va_vb);
    // ROS_INFO("pow(norm_va_vb, 2): %f\n", pow(norm_va_vb, 2));
    // ROS_INFO("pa + va*tcpa: (%f, %f)", own_vessel_component_x, own_vessel_component_y);
    // ROS_INFO("pb + vb*tcpa: (%f, %f)", other_vessel_component_x, other_vessel_component_y);

    // ROS_INFO("t_cpa: %f", t_cpa);
    // ROS_INFO("d_cpa: %f\n", d_cpa);

    msg.data = t_cpa;
    tcpa_pub_.publish(msg);
    msg.data = d_cpa;
    dcpa_pub_.publish(msg);

    return (t_cpa <= 20 && d_cpa <= 9 ? true : false);
  }

  /*
   * given the current state of the robot and the global plan, generates linear and angular velocities command
   * uses last position of global plan inside local costmap as A* goal
   * 
    // * if A* goal is not valid AND last goal is not valid then do nothing
    // * else if A* goal is not valid AND last go is valid then use last cmd_vel
    // * else plan and generate new cmd_vel
    // **** creates artificial terrain cost
    // **** A* search
    // **** generate cmd_vel
   * 
   */
  base_local_planner::Trajectory RRAPlanner::findBestPath(
      tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      tf::Stamped<tf::Pose>& drive_velocities) 
  {

    global_vel_.position.x = global_vel.getOrigin().getX();
    global_vel_.position.y = global_vel.getOrigin().getY();

    global_pose_.position.x = global_pose.getOrigin().getX();
    global_pose_.position.y = global_pose.getOrigin().getY();

    //make sure that our configuration doesn't change mid-run
    boost::mutex::scoped_lock l(configuration_mutex_);

    geometry_msgs::Point                    astar_local_goal_global_frame = global_plan_.back().pose.position;  // Local goal
    base_local_planner::LocalPlannerLimits  limits                        = planner_util_->getCurrentLimits();  // TODO: Evaluate need. (probably can be removed)
    costmap_2d::Costmap2D                   *local_costmap_2d             = planner_util_->getCostmap();        // Auxiliary for local costmap access
    int                                     mx, my;                                                             // General auxiliaries variables for world to map coordinates tranform
    Pos                                     current_astar_goal;                                                 // Current A* goal
    Pos                                     current_pos;
    GridWithWeights*                        graph;                                                              // A* main strucutre for goal search
    std::unordered_map<Pos, Pos>            came_from;                                                          // A* Path
    std::unordered_map<Pos, double>         cost_so_far;                                                        // A*'s exploration phase util    
    std::vector<geometry_msgs::Point>       local_path_at_global_frame;
    AStar::AStar                            astar;                                                              // A* handler

    // ROS_INFO("global_plan size: %d", (int) global_plan_.size());
    // ROS_INFO("first: (%f, %f) last: (%f, %f))", 
    //   global_plan_.back().pose.position.x, 
    //   global_plan_.back().pose.position.y,  
    //   global_plan_.front().pose.position.x, 
    //   global_plan_.front().pose.position.y);
    
    //-----------------------------* Evaluate if goal and current position are valid for planning

    // Gets closer global plan position in local frame reference
    local_costmap_2d->worldToMapEnforceBounds(  astar_local_goal_global_frame.x, 
                                                astar_local_goal_global_frame.y, 
                                                current_astar_goal.x, 
                                                current_astar_goal.y);

    // Gets robot current position in local frame reference
    local_costmap_2d->worldToMapEnforceBounds(  global_pose.getOrigin().getX(), 
                                                global_pose.getOrigin().getY(), 
                                                current_pos.x, 
                                                current_pos.y);

    
    // ROS_INFO("A* goal:    (%d, %d)", current_astar_goal.x, current_astar_goal.y);
    // searches for farest valid A* goal
    std::vector<geometry_msgs::PoseStamped>::iterator g_itr;
    g_itr = global_plan_.end();

    // if (!isAValidPlanningPosition(current_astar_goal))
    // {
    //   ROS_INFO("Invalid current A* goal, before while");
    // }
    // if (g_itr != global_plan_.begin())
    // {
    //   ROS_INFO("g_itr != global_plan_.begin(), before while");
    // }
    
    while (!isAValidPlanningPosition(current_astar_goal) && (g_itr != global_plan_.begin()) )
    {

      astar_local_goal_global_frame = (*g_itr).pose.position;

      // Gets closer global plan position in local frame reference
      local_costmap_2d->worldToMapEnforceBounds(  astar_local_goal_global_frame.x, 
                                                  astar_local_goal_global_frame.y, 
                                                  current_astar_goal.x, 
                                                  current_astar_goal.y);
      g_itr--;
    }

    // if (!isAValidPlanningPosition(current_astar_goal))
    // {
    //   ROS_INFO("Invalid current A* goal, after while");
    // }

    // if (!isAValidPlanningPosition(current_pos))
    // {
    //   ROS_INFO("Invalid current pos, after while");
    // }

    if ( !isAValidPlanningPosition( current_astar_goal ) || !isAValidPlanningPosition( current_pos ) )  // * if A* goal or current pos are NOT valid (out of local costmap OR in occupied cell)
    {

      // ROS_INFO("A* goal or current pos INVALID");
      // ROS_INFO("Last A* goal: (%d, %d)", last_astar_goal_.x, last_astar_goal_.y);

      if ( last_astar_goal_.x != -1 && last_astar_goal_.y != -1 ) // * if A* goal or current pos ARE NOT valid AND last goal is valid then use last cmd_vel
      {
        // ROS_INFO("Last A* goal valid");

        drive_velocities = last_drive_velocities_;
        result_traj_.cost_ = 12;

      }else                                       // * if A* goal os current pos ARE NOT valid AND last goal IS NOT valid then do not generate any velocity
      {
        // ROS_INFO("Last A* goal INVALID");

        drive_velocities.setIdentity();
        result_traj_.cost_ = -7;

      }
      
      return result_traj_;

    } // else valid goal
    
    //-----------------------------* plan and generate new cmd_vel

    // Converts Costmap to graph to be used in the A* method
    graph = costmapToGrid( local_costmap_2d );
    // std::cout << "Finished conversion" << std::endl;
    // ROS_INFO("Costmap size: %d x %d", planner_util_->getCostmap()->getSizeInCellsX(), planner_util_->getCostmap()->getSizeInCellsY());
    // ROS_INFO("Costmap resolution: %f ", planner_util_->getCostmap()->getResolution());

    //-----------------------------* creates artificial terrain cost
    // Artificial Terrain Cost for COLREGS-COMPLIANCE if is there any other vessel near
    bool weHaveCompany = false;
    bool weAreInDanger = false;

    weHaveCompany = isThereAnyOtherVesselNear();
    weAreInDanger = cpaCollisionRiskIndex();

    if (weHaveCompany)
    {
      if(first_colregs_identified_ == null)
      {
        // colregs_encounter_type risk = identifyCOLREGSEncounterType(global_pose);
        first_colregs_identified_ = identifyCOLREGSEncounterType(global_pose);
      }

      // Artificial Terrain Cost for COLREGS-COMPLIANCE
      geometry_msgs::Point diff2_pos;
      diff2_pos = other_vessel_pose_.position;
      other_vessel_pose_.position.x = -1;
      other_vessel_pose_.position.y = -1;

      if(first_sector_detected_ == 0)
      {
        unsigned short int sector = 0;
        double ori = (180.0 / M_PI) * tf::getYaw(global_pose.getRotation());

        // ROS_INFO("ori: %f", ori);

        if (ori >= -45 && ori < 45)
        {
          sector = 1;
        }
        else if (ori >= 45 && ori < 135)
        {
          sector = 2;
        }
        else if ((ori >= 135 && ori < 180) || (ori >= -180 && ori < -135))
        {
          sector = 3;
        }
        else if (ori >= -135 && ori < -45)
        {
          sector = 4;
        }

        first_sector_detected_ = sector;
      }

      if(weAreInDanger)
      {
        ROS_INFO("Creating virtual obstacles!");
        std::vector<geometry_msgs::Point> artificial_terrain = createArtificialTerrainCost(diff2_pos, first_colregs_identified_, first_sector_detected_);

        // std::cout << "Artificial Terrain: " << std::endl;
        // for (auto pos = artificial_terrain.end()-1; pos != artificial_terrain.begin(); pos--)
        // {
        //   std::cout << "( " << pos->x << ", " << pos->y << " )" << std::endl;
        // }

        for (auto pos = artificial_terrain.begin(); pos != artificial_terrain.end(); pos++)
        {

          local_costmap_2d->worldToMapEnforceBounds(  pos->x, 
                                                      pos->y, 
                                                      mx, 
                                                      my);

          // ROS_INFO("Before set (%d, %d): %d", mx, my, local_costmap_2d->getCost(mx, my));
          // local_costmap_2d->setCost(mx, my, (unsigned char)255);
          // ROS_INFO("After set (%d, %d): %d", mx, my, local_costmap_2d->getCost(mx, my));

          // if ((euclidian_distance(diff2_pos.x, diff2_pos.y, global_pose.getOrigin().getX(), global_pose.getOrigin().getY()) > CRITICAL_DISTANCE) ||
          //     (std::fabs(diff2_pos.x - global_pose.getOrigin().getX()) > CRITICAL_DISTANCE) ||
          //     (std::fabs(diff2_pos.y - global_pose.getOrigin().getY()) > CRITICAL_DISTANCE))
          // {
          // if (euclidian_distance(diff2_pos.x, diff2_pos.y, global_pose.getOrigin().getX(), global_pose.getOrigin().getY()) > CRITICAL_DISTANCE)         
          // {

            graph->walls.insert(Pos{mx, my, 0, double(COSTMAP_OCCUPANCE_ACCEPTANCE) +1.0});

            // }
        }  
      }
    }
    else
    {
      first_sector_detected_ = 0;
      first_colregs_identified_ = null;
    }

    // ROS_INFO("A* goal: (%d, %d)", current_astar_goal.x, current_astar_goal.y);

    //-----------------------------* A* search

    // current_pos.x = mx;
    // current_pos.y = my;
    // ROS_INFO("Robot pos:  (%f, %f)", (double) current_pos.x, (double) current_pos.y);

    astar.AStarSearch(*(graph), current_pos, current_astar_goal, came_from, cost_so_far);                             // A* method execution
    // std::cout << "Reconstructing path" << std::endl;
    std::vector<Pos> local_path_at_local_frame = astar.reconstruct_path(current_pos, current_astar_goal, came_from);  // Util for easier path use
    result_traj_.cost_ = 12;
    // std::cout << "Finished path reconstruction" << std::endl;

    // ROS_INFO("Local path at local frame:");
    // for (auto pos = local_path_at_local_frame.begin(); pos != local_path_at_local_frame.end(); pos++)
    // {
    //   std::cout << "( " << pos->x << ", " << pos->y << ")->";
    // }
    // std::cout << std::endl;

    // std::cout << "Drawing cenario" << std::endl;
    // draw_grid(*graph, 2, nullptr, nullptr, &local_path_at_local_frame);
    // std::cout << "Finished drawing" << std::endl;

    // std::cout << "Converting local plan from local coordinations to global cordenates" << std::endl;
    for (auto pos = local_path_at_local_frame.begin(); pos != local_path_at_local_frame.end(); pos++)
    {

      geometry_msgs::Point global_pos;
      // // (Xg*resolution + Xcostmap, Yg*resolution + Ycostmap)
      local_costmap_2d->mapToWorld( pos->x, 
                                    pos->y, 
                                    global_pos.x, 
                                    global_pos.y);
      local_path_at_global_frame.push_back(global_pos);

    }
    // std::cout << "Finished conversion" << std::endl;

    // Populating result_traj_ for debugging propose
    result_traj_.resetPoints();
    for (auto p = local_path_at_global_frame.begin(); p != local_path_at_global_frame.end(); p++)
    {
      result_traj_.addPoint(p->x, p->y, steering_angle(p->x, p->y, global_pose.getOrigin().getX(), global_pose.getOrigin().getX()));
    }


    //-----------------------------* generate cmd_vel

    Eigen::Vector3f pos(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), tf::getYaw(global_pose.getRotation()));
    // debrief stateful scoring functions
    oscillation_costs_.updateOscillationFlags(pos, &result_traj_, planner_util_->getCurrentLimits().min_trans_vel);

    // Creates cmd_vel populating drive_velocities with translation and rotation matrixes
    unsigned short int local_path_index_to_follow = 0;
    local_path_index_to_follow = local_path_at_global_frame.size() >= POSE_TO_FOLLOW ? POSE_TO_FOLLOW -1 : local_path_at_global_frame.size() -1;

    double ang = steering_angle(  local_path_at_global_frame[local_path_index_to_follow].x, 
                                  local_path_at_global_frame[local_path_index_to_follow].y, 
                                  global_pose.getOrigin().getX(), 
                                  global_pose.getOrigin().getY() 
                                ) - tf::getYaw(global_pose.getRotation());

    tf::Vector3 start(0, 0, 0);  
    if (fabs(ang) <= STEERING_ANGLE*M_PI/180 )
    {
          start = tf::Vector3(
                      linear_vel(
                                  local_path_at_global_frame[local_path_index_to_follow].x, 
                                  local_path_at_global_frame[local_path_index_to_follow].y, 
                                  global_pose.getOrigin().getX(), 
                                  global_pose.getOrigin().getY(), 
                                  PID_Kp_LINEAR
                                ), 
                      0, 
                      0);

          drive_velocities.setOrigin(start);
      
    }else if ( last_astar_goal_.x != -1 && last_astar_goal_.y != -1 )
    {

      drive_velocities.setOrigin(last_drive_velocities_.getOrigin());

    }else{ drive_velocities.setOrigin(start); }
    
    // drive_velocities.setOrigin(start);    
    // drive_velocities.setOrigin(last_drive_velocities_.getOrigin());        

    tf::Matrix3x3 matrix;
    matrix.setRotation(
                        tf::createQuaternionFromYaw(
                                                    angular_vel(local_path_at_global_frame[local_path_index_to_follow].x, 
                                                    local_path_at_global_frame[local_path_index_to_follow].y, global_pose.getOrigin().getX(), 
                                                    global_pose.getOrigin().getY(), 
                                                    tf::getYaw(global_pose.getRotation()), 
                                                    PID_Kp_ANGULAR)
                                                    )
                      );
    drive_velocities.setBasis(matrix);

    last_astar_goal_.x = astar_local_goal_global_frame.x;
    last_astar_goal_.y = astar_local_goal_global_frame.y;
    last_drive_velocities_ = drive_velocities;

    return result_traj_;
  }

  double euclidian_distance(double goal_x, double goal_y, double self_x, double self_y){
    return sqrt(  pow((goal_x - self_x), 2) + 
                  pow((goal_y - self_y), 2));
  }

  // As outlined in Section 4.1.1, a simple heading proportional
  // controller based on Bertaska2015Experimental and Go to Goal ROS move_base tutorial
  double RRAPlanner::linear_vel(double goal_x, double goal_y, double self_x, double self_y, double constt){
    return constt * euclidian_distance(goal_x, goal_y, self_x, self_y);

    // double err = euclidian_distance(goal_x, goal_y, self_x, self_y);
    // double P = PID_Kp_LINEAR * err;
    // pid_I_linear_ += PID_Ki_LINEAR * err;

    // return P + pid_I_linear_;

  }

  double RRAPlanner::angular_vel(double goal_x, double goal_y, double self_x, double self_y, double self_th, double constt){
    return constt * (steering_angle(goal_x, goal_y, self_x, self_y) - self_th);

    // double err = steering_angle(goal_x, goal_y, self_x, self_y) - self_th;
    // double P = PID_Kp_ANGULAR * err;
    // pid_I_angular_ += PID_Ki_ANGULAR * err;

    // return P + pid_I_angular_;
  }

  double steering_angle(double goal_x, double goal_y, double self_x, double self_y){

    
    double angle = atan2( goal_y - self_y, goal_x - self_x);

    angle = angle == M_PI ? -M_PI : angle;                                              // angle inside [-PI, PI)
    if ( (angle >= M_PI - (5 / (180/M_PI)) ) || (angle <= -M_PI + (5 / (180/M_PI))) ){  // if between [175, -175]
      angle = -M_PI + (5 / (180/M_PI));
    }

    // if (angle >= M_PI)
    // {
    //   angle = angle - 2 * M_PI;
    // }

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
          // ROS_INFO("WALL (%d, %d) - cost: %f", i, j, double(costmap->getCost(i, j)));
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

  bool RRAPlanner::isAValidPlanningPosition(Pos astar_goal){

    return planner_util_->getCostmap()->getCost(astar_goal.x, astar_goal.y) <= COSTMAP_OCCUPANCE_ACCEPTANCE;
    
  }

  std::vector<geometry_msgs::Point> RRAPlanner::createArtificialTerrainCost(geometry_msgs::Point otherVesselPos, colregs_encounter_type risk, unsigned short int sector)  {

    std::vector<geometry_msgs::Point> artificial_terrain  ;

    geometry_msgs::Point pos  ;

    //// identify ori
    // double otherVessel_orientation = (180.0 / M_PI) * tf::getYaw(other_vessel_pose_.orientation);

    // ROS_INFO("Other Vessel ori: %f", otherVessel_orientation);
    
    // double obstacle_orientation = otherVessel_orientation < 90 ? otherVessel_orientation + 90 : otherVessel_orientation - 270;

    // ROS_INFO("Obstacle ori: %f", obstacle_orientation);

    // if Headon
    //// creates obstacle based on ori
    // else if Crossing_Rigth
    //// creates obstacle based on ori
    // else if Crossing_Left
    //// creates obstacle based on ori
    // else if Overtaking
    //// creates obstacle based on ori

    short int width, length, x_offset;

    switch (risk)
    {
    case HeadOn:
      switch (sector)
      {
      case 1:
        ROS_INFO("HeadOn S1");
        // Hardcoded ajustments
        width     = ARTIFICIAL_TERRAIN_COST_WIDTH/2;
        length    = ARTIFICIAL_TERRAIN_COST_LENGTH +2;
        x_offset  = 1;

        // Artificial obstacle creation
        for (short int j = -width / 2; j < width / 2; j++)
        {

          for (unsigned short int i = 1; i <= length; i++)
          { 
            double res = planner_util_->getCostmap()->getResolution();

            pos.x = otherVesselPos.x + j*res - x_offset*res;
            pos.y = otherVesselPos.y + i*res;

            artificial_terrain.push_back(pos);
          }

        }
        break;
      
      case 2:
        ROS_INFO("HeadOn S2");
        // Hardcoded ajustments
        width     = ARTIFICIAL_TERRAIN_COST_WIDTH/2;
        length    = ARTIFICIAL_TERRAIN_COST_LENGTH +2;
        x_offset  = 1;

        // Artificial obstacle creation
        for (short int j = -width / 2; j < width / 2; j++)
        {

          for (unsigned short int i = 1; i <= length; i++)
          { 
            double res = planner_util_->getCostmap()->getResolution();

            pos.x = otherVesselPos.x - i*res;
            pos.y = otherVesselPos.y + j*res - x_offset*res;

            artificial_terrain.push_back(pos);
          }

        }
        break;
      
      case 3:
        ROS_INFO("HeadOn S3");
        // Hardcoded ajustments
        width     = ARTIFICIAL_TERRAIN_COST_WIDTH/2;
        length    = ARTIFICIAL_TERRAIN_COST_LENGTH +2;
        x_offset  = 1;

        // Artificial obstacle creation
        for (short int j = -width / 2; j < width / 2; j++)
        {

          for (unsigned short int i = 1; i <= length; i++)
          { 
            double res = planner_util_->getCostmap()->getResolution();

            pos.x = otherVesselPos.x + j*res - x_offset*res;
            pos.y = otherVesselPos.y - i*res;

            artificial_terrain.push_back(pos);
          }

        }
        break;
      
      case 4:
        ROS_INFO("HeadOn S4");
        // Hardcoded ajustments
        width     = ARTIFICIAL_TERRAIN_COST_WIDTH/2;
        length    = ARTIFICIAL_TERRAIN_COST_LENGTH +2;
        x_offset  = 1;

        // Artificial obstacle creation
        for (short int j = -width / 2; j < width / 2; j++)
        {

          for (unsigned short int i = 1; i <= length; i++)
          { 
            double res = planner_util_->getCostmap()->getResolution();

            pos.x = otherVesselPos.x + i*res;
            pos.y = otherVesselPos.y + j*res - x_offset*res;

            artificial_terrain.push_back(pos);
          }

        }
        break;
      
      default:
        ROS_INFO("HeadOn No Section");
        break;
      }
      break;
    
    case Left:
      switch (sector)
      {
      case 1:
        ROS_INFO("Left S1");
        break;
      
      case 2:
        ROS_INFO("Left S2");
        break;
      
      case 3:
        ROS_INFO("Left S3");
        break;
      
      case 4:
        ROS_INFO("Left S4");
        break;
      
      default:
        ROS_INFO("Left No Section");
        break;
      }
      break;
    
    case Right:
      switch (sector)
      {
      case 1:
        ROS_INFO("Right S1");
        // Hardcoded ajustments
        width     = ARTIFICIAL_TERRAIN_COST_WIDTH/2;
        length    = ARTIFICIAL_TERRAIN_COST_LENGTH +2;
        x_offset  = 1;

        // Artificial obstacle creation
        for (short int j = -width / 2; j < width / 2; j++)
        {

          for (unsigned short int i = 1; i <= length; i++)
          { 
            double res = planner_util_->getCostmap()->getResolution();

            pos.x = otherVesselPos.x + j*res - x_offset*res;
            pos.y = otherVesselPos.y + i*res;

            artificial_terrain.push_back(pos);
          }

        }
        break;
      
      case 2:
        ROS_INFO("Right S2");
        // Hardcoded ajustments
        width     = ARTIFICIAL_TERRAIN_COST_WIDTH/2;
        length    = ARTIFICIAL_TERRAIN_COST_LENGTH +2;
        x_offset  = 1;

        // Artificial obstacle creation
        for (short int j = -width / 2; j < width / 2; j++)
        {

          for (unsigned short int i = 1; i <= length; i++)
          { 
            double res = planner_util_->getCostmap()->getResolution();

            pos.x = otherVesselPos.x - i*res;
            pos.y = otherVesselPos.y + j*res - x_offset*res;

            artificial_terrain.push_back(pos);
          }

        }
        break;
      
      case 3:
        ROS_INFO("Right S3");
        // Hardcoded ajustments
        width     = ARTIFICIAL_TERRAIN_COST_WIDTH/2;
        length    = ARTIFICIAL_TERRAIN_COST_LENGTH +2;
        x_offset  = 1;

        // Artificial obstacle creation
        for (short int j = -width / 2; j < width / 2; j++)
        {

          for (unsigned short int i = 1; i <= length; i++)
          { 
            double res = planner_util_->getCostmap()->getResolution();

            pos.x = otherVesselPos.x + j*res - x_offset*res;
            pos.y = otherVesselPos.y - i*res;

            artificial_terrain.push_back(pos);
          }

        }
        break;
      
      case 4:
        ROS_INFO("Right S4");
        // Hardcoded ajustments
        width     = ARTIFICIAL_TERRAIN_COST_WIDTH/2;
        length    = ARTIFICIAL_TERRAIN_COST_LENGTH +2;
        x_offset  = 1;

        // Artificial obstacle creation
        for (short int j = -width / 2; j < width / 2; j++)
        {

          for (unsigned short int i = 1; i <= length; i++)
          { 
            double res = planner_util_->getCostmap()->getResolution();

            pos.x = otherVesselPos.x + i*res;
            pos.y = otherVesselPos.y + j*res - x_offset*res;

            artificial_terrain.push_back(pos);
          }

        }
        break;
      
      default:
        ROS_INFO("Right No Section");
        break;
      }
      break;
    
    case Overtaking:
      switch (sector)
      {
      case 1:
        ROS_INFO("Overtaking S1");
        // Hardcoded ajustments
        width     = ARTIFICIAL_TERRAIN_COST_WIDTH/2;
        length    = ARTIFICIAL_TERRAIN_COST_LENGTH +2;
        x_offset  = 1;

        // Artificial obstacle creation
        for (short int j = -width / 2; j < width / 2; j++)
        {

          for (unsigned short int i = 1; i <= length; i++)
          { 
            double res = planner_util_->getCostmap()->getResolution();

            pos.x = otherVesselPos.x + i*res;
            pos.y = otherVesselPos.y + j*res - x_offset*res;

            artificial_terrain.push_back(pos);
          }

        }
        break;
      
      case 2:
        ROS_INFO("Overtaking S2");
        // Hardcoded ajustments
        width     = ARTIFICIAL_TERRAIN_COST_WIDTH/2;
        length    = ARTIFICIAL_TERRAIN_COST_LENGTH +2;
        x_offset  = 1;

        // Artificial obstacle creation
        for (short int j = -width / 2; j < width / 2; j++)
        {

          for (unsigned short int i = 1; i <= length; i++)
          { 
            double res = planner_util_->getCostmap()->getResolution();

            pos.x = otherVesselPos.x + j*res - x_offset*res;
            pos.y = otherVesselPos.y + i*res;

            artificial_terrain.push_back(pos);
          }

        }
        break;
      
      case 3:
        ROS_INFO("Overtaking S3");
        // Hardcoded ajustments
        width     = ARTIFICIAL_TERRAIN_COST_WIDTH/2;
        length    = ARTIFICIAL_TERRAIN_COST_LENGTH +2;
        x_offset  = 1;

        // Artificial obstacle creation
        for (short int j = -width / 2; j < width / 2; j++)
        {

          for (unsigned short int i = 1; i <= length; i++)
          { 
            double res = planner_util_->getCostmap()->getResolution();

            pos.x = otherVesselPos.x - i*res;
            pos.y = otherVesselPos.y + j*res - x_offset*res;

            artificial_terrain.push_back(pos);
          }

        }
        break;
      
      case 4:
        ROS_INFO("Overtaking S4");
        // Hardcoded ajustments
        width     = ARTIFICIAL_TERRAIN_COST_WIDTH/2;
        length    = ARTIFICIAL_TERRAIN_COST_LENGTH +2;
        x_offset  = 1;

        // Artificial obstacle creation
        for (short int j = -width / 2; j < width / 2; j++)
        {

          for (unsigned short int i = 1; i <= length; i++)
          { 
            double res = planner_util_->getCostmap()->getResolution();

            pos.x = otherVesselPos.x + j*res - x_offset*res;
            pos.y = otherVesselPos.y - i*res;

            artificial_terrain.push_back(pos);
          }

        }
        break;
      
      default:
        ROS_INFO("Overtaking No Section");
        break;
      }
      break;
    
    default:
      ROS_INFO("NO RISK");
      break;
    }

    // // Hardcoded ajustments
    // width     = ARTIFICIAL_TERRAIN_COST_WIDTH/2;
    // length    = ARTIFICIAL_TERRAIN_COST_LENGTH +2;
    // x_offset  = 1;

    // // Artificial obstacle creation
    // for (short int j = -width / 2; j < width / 2; j++)
    // {

    //   for (unsigned short int i = 1; i <= length; i++)
    //   { 
    //     double res = planner_util_->getCostmap()->getResolution();
    //     pos.x = otherVesselPos.x + j*res - x_offset*res;
    //     pos.y = otherVesselPos.y + i*res;
    //     artificial_terrain.push_back(pos);
    //   }

    // }
    
    

    return artificial_terrain;

  }

  void RRAPlanner::getOtherVesselOdom_callback(const nav_msgs::Odometry::ConstPtr& usv_position_msg){

    other_vessel_pose_  = usv_position_msg->pose.pose;
    other_vessel_vel_   = usv_position_msg->twist.twist;
    // ROS_INFO("USV current position: X: %f, Y: %f", usv_current_pos.x, usv_current_pos.y);
    // ROS_INFO("C_x - N_x: %f - %f --- C_y - N_y: %f - %f", usv_current_pos.x, next_goal.x, usv_current_pos.y, next_goal.y);

    publishDistance(euclidian_distance(other_vessel_pose_.position.x, other_vessel_pose_.position.y, global_pose_.position.x, global_pose_.position.y));
  }

  // probably could be improved calling only "wordToMap"
  bool RRAPlanner::isThereAnyOtherVesselNear(){
    
    if ( (other_vessel_pose_.position.x != -1) && (other_vessel_pose_.position.y != -1)) // Is there any publishing
    {
      unsigned int dummy_unsigned_int_x, dummy_unsigned_int_y;
      // identify if other vessel is near (inside local costmap region)
      return planner_util_->getCostmap()->worldToMap(other_vessel_pose_.position.x, other_vessel_pose_.position.y, dummy_unsigned_int_x, dummy_unsigned_int_y);
    }
  
  }

colregs_encounter_type RRAPlanner::identifyCOLREGSEncounterType(tf::Stamped<tf::Pose>& global_pose)
  {

    // double tCPA, dCPA;
    // double x, y, vx, vy;
    // double PAx, PAy, PBx, PBy, VAx, VAy, VBx, VBy;

    // PAx = global_pose.getOrigin().getX();
    // PAy = global_pose.getOrigin().getY();
    // PBx = other_vessel_pose_.position.x;
    // PBy = other_vessel_pose_.position.y;
    // VAx = global_vel_.position.x;
    // VAy = global_vel_.position.y;
    // VBx = other_vessel_vel_.linear.x;
    // VBy = other_vessel_vel_.linear.y;

    // x   = PAx - PBx;
    // y   = PAy - PBy;
    // vx  = VAx - VBx;
    // vy  = VAy - VBy;

    // std::vector<double> a{x, y};
    // std::vector<double> b{vx, vy};
    // // double r1 = std::inner_product(a.begin(), a.end(), b.begin(), 0);
    // double r1 = 0;
    // r1 = r1 + a[0] * b[0] + a[1] * b[1];
    // double magnitude = sqrt((vx * vx) + (vy * vy));

    // tCPA = fabs(r1) / magnitude;

    // x = PAx + VAx * tCPA;
    // y = PAy + VAy * tCPA;

    // x -= PBx + VBx * tCPA;
    // y -= PBy + VBy * tCPA;

    // dCPA = sqrt((x * x) + (y * y));

    // ROS_INFO("r1: %f", r1);
    // ROS_INFO("magnitude: %f", magnitude);
    // ROS_INFO("tCPA: %f", tCPA);
    // ROS_INFO("dCPA: %f", dCPA);

    double steering_ang   = atan2(global_pose.getOrigin().getY() - other_vessel_pose_.position.y, global_pose.getOrigin().getX() - other_vessel_pose_.position.x);
    steering_ang = steering_ang == M_PI ? -M_PI : steering_ang;                       // inside [-PI, PI)
    steering_ang = steering_ang >= M_PI ? steering_ang - 2*M_PI : steering_ang;       // inside [-PI, PI)
    steering_ang = steering_ang  < -M_PI ? steering_ang + 2*M_PI : steering_ang;      // inside [-PI, PI)

    double other_ori      = tf::getYaw(other_vessel_pose_.orientation);               // other vessel heading (rad)
    other_ori = other_ori == M_PI ? -M_PI : other_ori;                                // other vessel heading inside [-PI, PI)
    other_ori = other_ori >= M_PI ? other_ori - 2*M_PI : other_ori;                   // other vessel heading inside [-PI, PI)
    other_ori = other_ori  < -M_PI ? other_ori + 2*M_PI : other_ori;                  // other vessel heading inside [-PI, PI)

    double bearing_angle  = steering_ang - other_ori;                                 // relative bearing (rad)
    bearing_angle = bearing_angle == M_PI ? -M_PI : bearing_angle;                    // relative bearing inside [-PI, PI)
    bearing_angle = bearing_angle >= M_PI ? bearing_angle - 2*M_PI : bearing_angle;   // relative bearing inside [-PI, PI)
    bearing_angle = bearing_angle  < -M_PI ? bearing_angle + 2*M_PI : bearing_angle;  // relative bearing inside [-PI, PI)

    // ROS_INFO("Steering angle:   %f", (180.0 / M_PI) * steering_ang);
    // ROS_INFO("Other vessel ori: %f", (180.0 / M_PI) * other_ori);
    // ROS_INFO("Bearing angle:    %f", (180.0 / M_PI) * bearing_angle);

    bearing_angle *= (180.0 / M_PI);                                                  // rad to degree

    // COLREGS decision

    if ( (bearing_angle >= -15.0) && (bearing_angle < 15.0) )
    {

      ROS_INFO("Head On");
      return HeadOn;

    }else if ( (bearing_angle >= 15.0) && (bearing_angle < 112.5) )
    {

      ROS_INFO("Crossing from RIGHT");
      return Right;

    } else if ( ((bearing_angle >= 112.5) && (bearing_angle < 180.0)) ||  ((bearing_angle >= - 180.0) && (bearing_angle < - 112.5)) )
    {

      double usv_ori    = (180.0 / M_PI)*tf::getYaw(global_pose.getRotation());
      double other_ori  = (180.0 / M_PI)*tf::getYaw(other_vessel_pose_.orientation);

      usv_ori   = usv_ori   < 0 ? usv_ori   + 360 : usv_ori;
      other_ori = other_ori < 0 ? other_ori + 360 : other_ori;

      // if ( fabs(usv_ori - other_ori) > 90 )
      // {
      //   return null;
      // }
    
      ROS_INFO("Overtaking");
      return Overtaking;
    }else if ( (bearing_angle >= -112.5) && (bearing_angle < -15) )
    {

      ROS_INFO("Crossing from LEFT");
      return Left;

    }

    return null;
  }

  void RRAPlanner::publishDistance(double dist){

    std_msgs::Float64 msg;
    msg.data = dist;
    distance_pub_.publish(msg);

  }

};

//   colregs_encounter_type RRAPlanner::identifyCOLREGSEncounterType(tf::Stamped<tf::Pose>& global_pose)
//   {

//     // double tCPA, dCPA;
//     // double x, y, vx, vy;
//     // double PAx, PAy, PBx, PBy, VAx, VAy, VBx, VBy;

//     // PAx = global_pose.getOrigin().getX();
//     // PAy = global_pose.getOrigin().getY();
//     // PBx = other_vessel_pose_.position.x;
//     // PBy = other_vessel_pose_.position.y;
//     // VAx = global_vel_.position.x;
//     // VAy = global_vel_.position.y;
//     // VBx = other_vessel_vel_.linear.x;
//     // VBy = other_vessel_vel_.linear.y;

//     // x   = PAx - PBx;
//     // y   = PAy - PBy;
//     // vx  = VAx - VBx;
//     // vy  = VAy - VBy;

//     // std::vector<double> a{x, y};
//     // std::vector<double> b{vx, vy};
//     // // double r1 = std::inner_product(a.begin(), a.end(), b.begin(), 0);
//     // double r1 = 0;
//     // r1 = r1 + a[0] * b[0] + a[1] * b[1];
//     // double magnitude = sqrt((vx * vx) + (vy * vy));

//     // tCPA = fabs(r1) / magnitude;

//     // x = PAx + VAx * tCPA;
//     // y = PAy + VAy * tCPA;

//     // x -= PBx + VBx * tCPA;
//     // y -= PBy + VBy * tCPA;

//     // dCPA = sqrt((x * x) + (y * y));

//     // ROS_INFO("r1: %f", r1);
//     // ROS_INFO("magnitude: %f", magnitude);
//     // ROS_INFO("tCPA: %f", tCPA);
//     // ROS_INFO("dCPA: %f", dCPA);

//     double other_ori = tf::getYaw(other_vessel_pose_.orientation);  // rad
//     other_ori *= (180.0 / M_PI);                                      // degree
//     other_ori = other_ori < 0 ? other_ori + 360 : other_ori;        // positive degree
//     other_ori /= (180.0 / M_PI);                                      // positive rad

//     double bearing_angle = atan2( global_pose.getOrigin().getY() - other_vessel_pose_.position.y,
//                                   global_pose.getOrigin().getX() - other_vessel_pose_.position.x) -
//                                   other_ori;

//     // double bearing_angle =  atan2(other_vessel_pose_.position.y - global_pose.getOrigin().getY(),
//     //                              other_vessel_pose_.position.x - global_pose.getOrigin().getX()) 
//     //                         - tf::getYaw(global_pose.getRotation());

//     bearing_angle = (180.0 / M_PI) * bearing_angle; // rad to degree

//     if ( fabs(bearing_angle) > 360 )                    // angles bigger than 360 adjustment
//     {
//       int fact = (int)abs(bearing_angle / 360);
//       if (bearing_angle < 0)
//       {
//         bearing_angle = bearing_angle - fact * (-360);
//       }
//       else
//       {
//         bearing_angle = bearing_angle - fact * (360);
//       }
//     }

//     if (bearing_angle < -180)
//     {
//       bearing_angle += 360;
//     }

//     ROS_INFO("Steering angle:   %f", (180.0 / M_PI) * atan2( global_pose.getOrigin().getY() - other_vessel_pose_.position.y, global_pose.getOrigin().getX() - other_vessel_pose_.position.x));
//     ROS_INFO("Other vessel ori: %f", (180.0 / M_PI) * other_ori);
//     ROS_INFO("Bearing angle:    %f", bearing_angle);


//     double steering_ang_2   = atan2(global_pose.getOrigin().getY() - other_vessel_pose_.position.y, global_pose.getOrigin().getX() - other_vessel_pose_.position.x);  //rad
//     double other_ori_2      = tf::getYaw(other_vessel_pose_.orientation); // rad
//     double bearing_angle_2  = steering_ang_2 - other_ori_2; // rad
//     ROS_INFO("Steering angle:   %f", (180.0 / M_PI) * steering_ang_2);
//     ROS_INFO("Other vessel ori: %f", (180.0 / M_PI) * other_ori_2);
//     ROS_INFO("Bearing angle:    %f", (180.0 / M_PI) * bearing_angle_2);

//     if ( (bearing_angle >= -15.0) && (bearing_angle < 15.0) )
//     {

//       ROS_INFO("Head On");
//       return HeadOn;

//     }else if ( (bearing_angle >= 15.0) && (bearing_angle < 112.5) )
//     {

//       ROS_INFO("Crossing from RIGHT");
//       return Right;

//     } else if ( ((bearing_angle >= 112.5) && (bearing_angle < 180.0)) ||  ((bearing_angle >= - 180.0) && (bearing_angle < - 112.5)) )
//     {

//       double usv_ori    = (180.0 / M_PI)*tf::getYaw(global_pose.getRotation());
//       double other_ori  = (180.0 / M_PI)*tf::getYaw(other_vessel_pose_.orientation);

//       usv_ori   = usv_ori   < 0 ? usv_ori   + 360 : usv_ori;
//       other_ori = other_ori < 0 ? other_ori + 360 : other_ori;

//       if ( fabs(usv_ori - other_ori) > 90 )
//       {
//         return null;
//       }
    
//       ROS_INFO("Overtaking");
//       return Overtaking;
//     }else if ( (bearing_angle >= -112.5) && (bearing_angle < -15) )
//     {

//       ROS_INFO("Crossing from LEFT");
//       return Left;

//     }

//     return null;
//   }

// };
