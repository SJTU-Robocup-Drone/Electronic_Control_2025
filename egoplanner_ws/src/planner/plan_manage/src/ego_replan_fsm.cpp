
#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    trigger_ = false;

    // new-added for escaping
    odom_cnt = 0;
    is_in_inflated_zone_ = 0;
    has_original_target_ = false;
    escape_target_ = Eigen::Vector3d::Zero();
    original_target_ = Eigen::Vector3d::Zero();
    escape_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 50);
    escape_state_pub_ = nh.advertise<std_msgs::Bool>("/escape_state", 10);
    self_trig_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/control/move_base_simple/goal", 10);
    adjusted_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/adjusted_goal", 10);

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);
    nh.param("fsm/emergency_time_", emergency_time_, 1.0);

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("/odom_world", 1, &EGOReplanFSM::odometryCallback, this);

    bspline_pub_ = nh.advertise<ego_planner::Bspline>("/planning/bspline", 10);
    data_disp_pub_ = nh.advertise<ego_planner::DataDisp>("/planning/data_display", 100);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
      waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &EGOReplanFSM::waypointCallback, this);
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      ros::Duration(1.0).sleep();
      while (ros::ok() && !have_odom_)
        ros::spinOnce();
      planGlobalTrajbyGivenWps();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps[i](0) = waypoints_[i][0];
      wps[i](1) = waypoints_[i][1];
      wps[i](2) = waypoints_[i][2];

      end_pt_ = wps.back();
    }
    bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      std::vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      // if (exec_state_ == WAIT_TARGET)
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      // else if (exec_state_ == EXEC_TRAJ)
      //   changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      ros::Duration(0.001).sleep();
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
      ros::Duration(0.001).sleep();
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  bool EGOReplanFSM::isSafe(Eigen::Vector3d &pos, int safe_radius)
  {
    auto grid_map = planner_manager_->grid_map_;
    Eigen::Vector3i current_idx;
    grid_map->posToIndex(pos, current_idx);
    for(int x = -safe_radius; x <= safe_radius; x++){
      for(int y = -safe_radius; y <= safe_radius; y++){
        Eigen::Vector3i candidate_idx = current_idx + Eigen::Vector3i(x, y, 0);
        Eigen::Vector3d candidate_pos;
        grid_map->indexToPos(candidate_idx,candidate_pos);
        if (grid_map->getInflateOccupancy(candidate_pos)) return false;
      }
    }
    return true;
  }

  void EGOReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
  {
    /* old version
    if (msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {


      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;


      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }*/

    // new-added code
    if (msg->poses[0].pose.position.z < -0.1) // 高度有效性检查
      return;
    if (msg->poses.size() < 1){ // 检查消息中是否有有效的目标点
      ROS_WARN("blank waypoints, return directly");
      return;
    }
    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, msg->poses[0].pose.position.z;

    // check whether the target is in obstacle area.
    // if it's trapped in obstacles, it should be moved to a safe area.
    auto grid_map = planner_manager_->grid_map_;
    int target_in_inflated_zone_ = !isSafe(end_pt_, 1);
    ROS_INFO("get isSafe of target = %d", !target_in_inflated_zone_);
    if (target_in_inflated_zone_) {
      ROS_WARN("Target is not safe! Try to adjust the target point.");
      // finding a new target
      if (adjustTarget(end_pt_)) {
        ROS_INFO("Find a safe goal. ");

        geometry_msgs::PoseStamped adjusted_goal_msg;
        adjusted_goal_msg.header.stamp = ros::Time::now();
        adjusted_goal_msg.header.frame_id = "map";
        adjusted_goal_msg.pose.position.x = end_pt_(0);
        adjusted_goal_msg.pose.position.y = end_pt_(1);
        adjusted_goal_msg.pose.position.z = end_pt_(2);
        adjusted_goal_pub.publish(adjusted_goal_msg);
      } else {
        ROS_ERROR("Failed to find safe goal! Attempting original goal.");
      }
    }

    // 保存原始目标点
    original_target_ = end_pt_;
    has_original_target_ = true;

    // 检测是否在障碍物膨胀区内
    is_in_inflated_zone_ = !isSafe(odom_pos_, 1);
    ROS_INFO("get isSafe of drone = %d", !is_in_inflated_zone_);
    if (is_in_inflated_zone_) {
      ROS_WARN("Drone is not safe! Starting escape procedure.");
    
      // 生成脱困目标点（膨胀区外最近安全点）
      if (findEscapeTarget(escape_target_)) {
        // 切换到脱困状态
        changeFSMExecState(ESCAPING, "ESCAPE");
      } else {
        ROS_ERROR("Failed to find escape target! Attempting original goal.");
      }
    }

    bool success = false;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    // 六个参数分别是起始位置、速度、加速度和目标位置、速度、加速度

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0); // 可视化目标点；四个参数分别为目标点，颜色，尺寸和标识

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1; // 采样时间步长
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t); // 向下取整计算以时间为采样依据的采样点的数量
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t); // 通过时间获得采样点
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET) // 如果当前状态是等待目标点，则切换到生成新轨迹状态
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ) // 如果当前状态是执行轨迹，则切换到重新规划轨迹状态
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0); // 可视化全局路径列表；三个参数分别为路径点数组，路径点间隔和标识
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  bool EGOReplanFSM::findEscapeTarget(Eigen::Vector3d& escape_target) {
    auto grid_map = planner_manager_->grid_map_;
    const int search_radius = 10; // 最大搜索半径（栅格单位）
  
    // 获取当前位置栅格坐标
    Eigen::Vector3i current_idx;
    grid_map->posToIndex(odom_pos_,current_idx);
  
    // 螺旋搜索安全点
    for (int r = 1; r <= search_radius; ++r) {
      for (int x = -r; x <= r; ++x) {
        for (int y = -r; y <= r; ++y) {
          if (abs(x) != r && abs(y) != r) continue; // 只检查边界
        
          Eigen::Vector3i candidate_idx = current_idx + Eigen::Vector3i(x, y, 0);
          Eigen::Vector3d candidate_pos;
          grid_map->indexToPos(candidate_idx,candidate_pos);
        
          // 检查是否安全且不在膨胀区内(包括周围四个点)
          if (isSafe(candidate_pos, 1)) {
            Eigen::Vector3i escape_idx = current_idx + Eigen::Vector3i(x,y,0);
            grid_map->indexToPos(escape_idx, escape_target);
            escape_target_(2) = 1.0;
            ROS_INFO_THROTTLE(1.0, "Find escaping target at (%.2f, %.2f, %.2f)", escape_target(0), escape_target(1), escape_target(2));
            return true;
          }
        }
      }
    }
    return false; // 未找到安全点

  }// new-added code

  bool EGOReplanFSM::adjustTarget(Eigen::Vector3d& target){
    /* old version
    auto grid_map = planner_manager_->grid_map_;
    const int search_radius = 10; // 最大搜索半径（栅格单位）
    // get target's 栅格坐标
    Eigen::Vector3i target_idx;
    grid_map->posToIndex(target,target_idx);

    // 螺旋搜索安全点
    for (int r = 1; r <= search_radius; ++r) {
      for (int x = -r; x <= r; ++x) {
        for (int y = -r; y <= r; ++y) {
          if (abs(x) != r && abs(y) != r) continue; // 只检查边界
        
          Eigen::Vector3i candidate_idx = target_idx + Eigen::Vector3i(x, y, 0);
          Eigen::Vector3d candidate_pos;
          grid_map->indexToPos(candidate_idx,candidate_pos);
        
          // 检查是否安全且不在膨胀区内
          if (!grid_map->getInflateOccupancy(candidate_pos)) {
            Eigen::Vector3i escape_idx = target_idx + Eigen::Vector3i(x,y,0);
            grid_map->indexToPos(escape_idx, target);
            target(2) = 1.0;
            ROS_INFO_THROTTLE(1.0, "Find safe target(terminal point) at (%.2f, %.2f, %.2f)", target(0), target(1), target(2));
            return true;
          }
        }
      }
    }*/
    auto grid_map = planner_manager_->grid_map_;
    Eigen::Vector3d direction = (odom_pos_ - target);
    direction(2) = 0; // restrict vector "direction" in 2 dimensions
    direction.normalize();
    cout << direction << endl;

    constexpr double step = 0.1;
    for(int i = 1; i <= 10; ++i){
      Eigen::Vector3d candidate_pos = target + direction * i * step;
      bool candidate_in_inflated_zone = !isSafe(candidate_pos, 1);
      if(!candidate_in_inflated_zone){
        target = candidate_pos;
        ROS_INFO_THROTTLE(1.0, "Find safe target(terminal point) at (%.2f, %.2f, %.2f)", target(0), target(1), target(2));
        return true;
      }
    }
    return false; // 未找到安全点
  }// new added

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
    if((exec_state_ == REPLAN_TRAJ || exec_state_ == EXEC_TRAJ) && ++odom_cnt >= 10){
      odom_cnt = 0;
      auto grid_map = planner_manager_->grid_map_;
      is_in_inflated_zone_ = !isSafe(odom_pos_, 1);
      if(is_in_inflated_zone_){
        ROS_WARN_THROTTLE(1.0, "Drone in inflated zone! Switching to ESCAPING.");
        if (findEscapeTarget(escape_target_)) {
          changeFSMExecState(ESCAPING, "ESCAPE");
        } else {
          ROS_ERROR("Failed to find escape target! Attempting original goal.");
        }
      }
    }
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) // 状态机状态转换函数；两个参数分别是新状态和调用位置的字符串描述（第二个参数只用于调试输出）
  {

    if (new_state == exec_state_) // 处理和存储连续调用次数
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "ESCAPING"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state; // 切换状态机状态
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl; // 输出状态切换信息
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls() // 获取连续调用次数和当前状态
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState() // 输出当前状态机状态
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "ESCAPING"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e) // 状态机主回调函数（由定时器控制执行）
  {
    // 每100次调用打印一次状态机状态和相关信息
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    // 状态机状态处理
    switch (exec_state_)
    {
    case INIT:
    {
      if(escape_state_msg_.data == true){
        trigger_ = false;
        escape_state_msg_.data = false;
        escape_state_pub_.publish(escape_state_msg_);
        ROS_INFO("Switching to INIT state and planning to original target.");
        ros::Duration(1.0).sleep(); // wait for the above message to be sent
        original_target_pose_.header.stamp = ros::Time::now();
        original_target_pose_.header.frame_id = "map";
        original_target_pose_.pose.position.x = end_pt_(0);
        original_target_pose_.pose.position.y = end_pt_(1);
        original_target_pose_.pose.position.z = end_pt_(2);
        self_trig_pub_.publish(original_target_pose_);
      }
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      // 根据连续调用次数决定是否使用随机多项式初始化
      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else if(timesOfConsecutiveStateCalls().first <= 20)
        flag_random_poly_init = true;
      else{
        ROS_ERROR("Too many consecutive calls of GEN_NEW_TRAJ, switching to ESCAPING");
        findEscapeTarget(escape_target_);
        changeFSMExecState(ESCAPING, "FSM");
        break;
      }

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true; // 允许紧急制动
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM"); // 路径生成失败则重试
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj()) // 从当前轨迹重规划
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM"); // 重规划失败则重试
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur); // 根据当前时间获取当前位置（时间的零点是无人机到达当前轨迹起始点的时间）

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2)
      {
        have_target_ = false;

        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_) // 接近终点，保持执行轨迹状态
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_) // 接近起点，保持执行轨迹状态
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM"); // 远离起点和终点，切换到重规划状态
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        // callEmergencyStop(odom_pos_); // 执行紧急制动
        // changeFSMExecState(ESCAPING, "FSM");
        findEscapeTarget(escape_target_);
        changeFSMExecState(ESCAPING, "FSM");
      }
      else
      {
        if (odom_vel_.norm() < 0.2)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM"); // 制动成功后切换至生成新轨迹状态
      }

      flag_escape_emergency_ = false; // 重置标志位避免重复调用
      break;
    }

    case ESCAPING: {
      escape_state_msg_.data = true;
      escape_state_pub_.publish(escape_state_msg_);
      // 检查是否离开膨胀区
      auto grid_map = planner_manager_->grid_map_;
      is_in_inflated_zone_ = !isSafe(odom_pos_, 1);
      /*static int out_of_inflated_zone_time = 0;
      constexpr int escaping_threshold = 10;
      if (is_in_inflated_zone_ != 1) { // 已成功逃离膨胀区，正常规划轨迹
        ++ out_of_inflated_zone_time;
        if(out_of_inflated_zone_time >= escaping_threshold){
          ROS_INFO("Successfully escaped inflated zone. Switching to INIT and Planning to original target.");
          changeFSMExecState(INIT, "FSM");
          out_of_inflated_zone_time = 0;
        }
        else{
          ROS_INFO("Times out of inflated zone = %d",out_of_inflated_zone_time);
        }
      } else { // 仍在膨胀区内，继续执行脱困轨迹和寻找安全点
        out_of_inflated_zone_time = 0;
        escape_state_msg_.data = true;
        escape_state_pub_.publish(escape_state_msg_);

        escape_pose.position.x = escape_target_(0);
        escape_pose.position.y = escape_target_(1);
        escape_pose.position.z = escape_target_(2);
        escape_pose_pub_.publish(escape_pose);
        ROS_INFO_THROTTLE(1.0, "Escaping to (%.2f, %.2f, %.2f)", escape_pose.position.x, escape_pose.position.y, escape_pose.position.z);

        findEscapeTarget(escape_target_); 
      }*/
      // double dist = (odom_pos_ - escape_target_).norm();
      // constexpr double threshold = 0.1;
      if (!is_in_inflated_zone_){
        // if(dist < threshold){
          ROS_INFO("Successfully escaped dangerous zone. Switching to INIT and Planning to original target.");
          changeFSMExecState(INIT, "FSM");
        // }
      }
      else{
        escape_pose.header.stamp = ros::Time::now();
        escape_pose.header.frame_id = "map";
        escape_pose.pose.position.x = escape_target_(0);
        escape_pose.pose.position.y = escape_target_(1);
        escape_pose.pose.position.z = escape_target_(2);
        escape_pose_pub_.publish(escape_pose);
        ROS_INFO_THROTTLE(1.0, "Escaping to (%.2f, %.2f, %.2f)", escape_pose.pose.position.x, escape_pose.pose.position.y, escape_pose.pose.position.z);

        // findEscapeTarget(escape_target_);
      }
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);
  }// new-added code

  bool EGOReplanFSM::planFromCurrentTraj()
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      //changeFSMExecState(EXEC_TRAJ, "FSM");
      if (!success)
      {
        success = callReboundReplan(true, true);
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      if (map->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t)))
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {

      auto info = &planner_manager_->local_data_;

      /* publish traj */
      ego_planner::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      bspline_pub_.publish(bspline);

      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }

    return plan_success;
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    ego_planner::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_.publish(bspline);

    return true;
  }

  void EGOReplanFSM::getLocalTarget()
  {
    double t;

    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        // todo
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        return;
      }
      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }
      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
      // cout << "AA" << endl;
    }
  }

} // namespace ego_planner
