
// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>
#include "visualization_msgs/Marker.h" // zx-todo
#include<iostream>

namespace ego_planner
{

  // SECTION interfaces for setup and query

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() {}

  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    /* read algorithm parameters */

    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
//cout<< "y==========================================================="<<pp_.max_vel_<<endl;
//cout<< pp_.max_acc_<<endl;


  //double  pp_.max_vel_=2.0;
  //double  pp_.max_acc_=3.0;
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
    nh.param("manager/use_distinctive_trajs", pp_.use_distinctive_trajs, true);
    nh.param("manager/drone_id", pp_.drone_id, -1);

    local_data_.traj_id_ = 0;

    env_ptr_.reset(new tgk_planner::OccMap);
    env_ptr_->init(nh);

      vis_ptr_.reset(new VisualRviz(nh));

    pos_checker_ptr_.reset(new tgk_planner::PosChecker);
    pos_checker_ptr_->init(nh);
    pos_checker_ptr_->setMap(env_ptr_);

    

    front_end_planner_ptr2_.reset(new tgk_planner::KRRTPlanner(nh));
    front_end_planner_ptr2_->init(nh);
    front_end_planner_ptr2_->setPosChecker(pos_checker_ptr_);
    front_end_planner_ptr2_->setVisualizer(vis_ptr_);
    // obj_predictor_.reset(new fast_planner::ObjPredictor(nh));
    // obj_predictor_->init();
    // obj_pub_ = nh.advertise<visualization_msgs::Marker>("/dynamic/obj_prdi", 10); // zx-todo

    bspline_optimizer_.reset(new BsplineOptimizer);
    bspline_optimizer_->setParam(nh);
    bspline_optimizer_->setEnvironment(pos_checker_ptr_);
    bspline_optimizer_->setEnvironment(pos_checker_ptr_, obj_predictor_);
    bspline_optimizer_->a_star_.reset(new AStar);
    bspline_optimizer_->a_star_->initGridMap(pos_checker_ptr_, Eigen::Vector3i(100, 100, 100));

    visualization_ = vis;
  }

  // !SECTION

  // SECTION rebond replanning

  bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
                                        //true和false分别代表0和1, lie  xiang liang
  {
    static int count = 0;
    printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n", pp_.drone_id, count++);
    // cout.precision(3);
     cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()   << endl;

    ///if ((start_pt - local_target_pt).norm() < 0.2)
   /// {
      //cout << "Close to goal" << endl;
      ///continous_failures_count_++;
      ///return false;
   /// }

  
    bspline_optimizer_->setLocalTargetPt(local_target_pt);
 
    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt, t_refine;
     //roslib给用户提供了ros::Time and ros::Duration两个类来描述时刻以及时间间隔两个概念，其中Duration可以是负数。Time和Duration拥有一样的成员：
//int32 sec
//int32 nsec


/*************************xiu gai*******************/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.5 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

    if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
    // 按顺序从最小快照 traj 生成的初始路径。 
      {
        flag_first_call = false;
        flag_force_polynomial = false;

       PolynomialTraj gl_traj;
        Trajectory traj_;

        double dist = (start_pt - local_target_pt).norm();
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;
         cout<<"start_pt111:::::::::::::::::"<< start_pt<<endl;
        if (!flag_randomPolyTraj)
        {
          
          
         bool result = front_end_planner_ptr2_->plan(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0);
        if (result)
      {
            
         front_end_planner_ptr2_->getTraj(traj_);
     }
        
         else
        {
           return false;
         }
           
         ///gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
     else
        {
         Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
        Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
         Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                           (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                                            (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);
         Eigen::MatrixXd pos(3, 3);
         //在始末位置之间随机插入一个点，
         pos.col(0) = start_pt;
         pos.col(1) = random_inserted_pt;
         pos.col(2) = local_target_pt;
         Eigen::VectorXd t(2);
         t(0) = t(1) = time *2;
         cout<<"start_pt222:::::::::::::::::"<< pos.col(0) <<endl;
      
      
       for(auto j_3= 0; j_3< t.size(); ++j_3)
      {
        double  t_jj_33=t(j_3);
      bool result = front_end_planner_ptr2_->plan( pos.col(0) , start_vel, start_acc, pos.col(2), local_target_vel, Eigen::Vector3d::Zero(), t_jj_33, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0);
     if (result)
      {
            
         front_end_planner_ptr2_->getTraj(traj_);
        }
         else
         {
         return false;
         }
      }
   ///gl_traj=   PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);


      }

        double t;
       bool flag_too_far;
        ts *= 1.5; // ts will be dividlocal_data_ed by 1.5 in the next
        do
       {
          ts /= 1.5;
         point_set.clear();
         flag_too_far = true;
     
          ///Eigen::Vector3d last_pt = gl_traj.evaluate(0);
         Eigen::Vector3d last_pt = traj_.getPos(0);
          for (t = 0; t < time; t += ts)
          {
            ///Eigen::Vector3d pt = gl_traj.evaluate(t);
           Eigen::Vector3d pt = traj_. getPos(t);

               if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
               {
               flag_too_far =false ;
          
              //cout<<"y2==============================="<<flag_too_far<<endl;
              
              break;
            
              }

              last_pt = pt;
              point_set.push_back(pt);
              ///cout<<"point_setbuzhengchang"<<point_set.size()<<endl;
             }
//.........................................................................................................
       } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;
        //start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(traj_.getVel(0));
        start_end_derivatives.push_back(local_target_vel);
        //start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        //start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
       start_end_derivatives.push_back(traj_.getAcc(0));
       start_end_derivatives.push_back(traj_.getAcc(t));
      }
      else // Initial path generated from previous trajectory.
      //从先前轨迹生成的初始路径。
      {
        double t;
        double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();
      //  cout<<"y6==============================="<<endl;
        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
         // cout<< "YIN===================="<<segment_point[1]<<endl;
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;
     //cout<<"y3==============================="<<endl;
        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / 2*pp_.max_vel_ ;
       
        if (poly_time > ts)
        {
        PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                   local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                   local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                      local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

      


          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              ///segment_point.push_back(traj_. getPos(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              ROS_ERROR("pseudo_arc_length is empty, return!");
              continous_failures_count_++;
              return false;
            }
          }
        }

        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        //cout<<"y4==============================="<<endl;
        size_t id = 0;
       //bujinru......................................................................................
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              //cout<<"point_sizebuzhengchang22222"<<point_set.size()<<endl;
              sample_length += cps_dist;
               //cout<<"CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"<<endl;

            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);
           ///cout<<"DDDDDDDDDDDDDDDDDDDDDDD"<<endl;
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help
        //  cout<<"y7==============================="<<endl;
        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3) // The initial path is unnormally too long!
        {
          
        //   cout<<"BBBBBBBBBBBBBBBBBBBBBBBBBB"<<endl;
          flag_force_polynomial = true;
          flag_regenerate = true;
          //初始路径异常长！ 
        }
      }
    } while (flag_regenerate);
//..........................................................................
    Eigen::MatrixXd ctrl_pts, ctrl_pts_temp;
    // cout<<"AAAAAAAAAAAAAAAAAAAAA"<<endl;
   //  cout << "ts:   " << ts<< endl;
    cout << "point_set:   " << point_set.size() << endl;
   //   cout << "point_set11:   " << point_set[0] << endl;
   // cout << "start_end_derivatives:   " <<start_end_derivatives[0]<< endl;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
    cout << "ctrl_pts:   " << ctrl_pts.size()<< endl;
    vector<std::pair<int, int>> segments;
    segments = bspline_optimizer_->initControlPoints(ctrl_pts, true);

    t_init = ros::Time::now() - t_start;
    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;

    if (pp_.use_distinctive_trajs)
    {
       cout << "enter" << endl;
      std::vector<ControlPoints> trajs = bspline_optimizer_->distinctiveTrajs(segments);
      cout << "\033[1;33m"
           << "multi-trajs=" << trajs.size() << "\033[1;0m" << endl;

      double final_cost, min_cost = 999999.0;
      for (int i = trajs.size() - 1; i >= 0; i--)
      {
        if (bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts_temp, final_cost, trajs[i], ts))//////////////////////////////huaile
        {

          cout << "traj " << trajs.size() - i << " success." << endl;

          flag_step_1_success = true;
          if (final_cost < min_cost)
          {
            min_cost = final_cost;
            ctrl_pts = ctrl_pts_temp;
          }

          // visualization
          point_set.clear();
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          {
            point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_trajs.push_back(point_set);
        }
        else
        {
          cout << "traj " << trajs.size() - i << " failed." << endl;
        }
      }
      
        static double cccc;
        cccc+=final_cost; 
        cout<<   "cost1="   <<cccc<<endl;

      t_opt = ros::Time::now() - t_start;

      visualization_->displayMultiInitPathList(vis_trajs, 0.2); // This visuallization will take up several milliseconds.
      //此可视化将花费几毫秒
    }
    else
    {
      flag_step_1_success = bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
      t_opt = ros::Time::now() - t_start;
      //static int vis_id = 0;
      visualization_->displayInitPathList(point_set, 0.2, 0);
    }

    cout << "plan_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
      visualization_->displayOptimalList(ctrl_pts, 0);
      continous_failures_count_++;
      return false;
    }

    t_start = ros::Time::now();

    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
  ////  static bool print_once = true;
    ///if (print_once)
   /// {
      ///print_once = false;
      ////ROS_ERROR("IN SWARM MODE, REFINE DISABLED!");
    ////}
    // disable refine in swarm scenario
    //在 swarm 场景中禁用细化
    double ratio;
    bool flag_step_2_success = true;
    if (!pos.checkFeasibility(ratio, false))
     {
     cout << "Need to reallocate time." << endl;

    Eigen::MatrixXd optimal_control_points;
     flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
     if (flag_step_2_success)
     pos = UniformBspline(optimal_control_points, 3, ts);
    }

    if (!flag_step_2_success)
    {
      printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
      continous_failures_count_++;
      return false;
     }

    t_refine = ros::Time::now() - t_start;

    // save planned results
    //保存计划结果
    updateTrajInfo(pos, ros::Time::now());

   static  double tttt,tttt1,tttt2,tttt3,tttt4; 
    static double sum_time = 0;
    static int count_success = 0;
    sum_time += (t_init + t_opt + t_refine).toSec();
     tttt+=sum_time;
     tttt1+=t_init.toSec();
     tttt2+=t_opt.toSec();
     tttt3+=t_refine.toSec();
     cout << "total time:\033[42m" <<tttt<<"tttt1:"<<tttt1<<"tttt2:"<<tttt2<<"tttt3:"<<tttt3<<"tttt4:"<<tttt1+tttt2;
    count_success++;
    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << ",avg_time=" << sum_time / count_success << endl;
    cout<<"\033[0m,optimize:" << (t_init + t_opt).toSec() <<endl;
    // success. YoY
    continous_failures_count_ = 0;
    return true;
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), ros::Time::now());

    return true;
  }

  bool EGOPlannerManager::checkCollision(int drone_id)
  {
    if (local_data_.start_time_.toSec() < 1e9) // It means my first planning has not started
      return false;

    double my_traj_start_time = local_data_.start_time_.toSec();
    double other_traj_start_time = swarm_trajs_buf_[drone_id].start_time_.toSec();

    double t_start = max(my_traj_start_time, other_traj_start_time);
    double t_end = min(my_traj_start_time + local_data_.duration_ * 2 / 3, other_traj_start_time + swarm_trajs_buf_[drone_id].duration_);

    for (double t = t_start; t < t_end; t += 0.03)
    {
      if ((local_data_.position_traj_.evaluateDeBoorT(t - my_traj_start_time) - swarm_trajs_buf_[drone_id].position_traj_.evaluateDeBoorT(t - other_traj_start_time)).norm() < bspline_optimizer_->getSwarmClearance())
      {
        return true;
      }
    }

    return false;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    total_len += (start_pos - waypoints[0]).norm();
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
      total_len += (waypoints[i + 1] - waypoints[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // for ( int i=0; i<inter_points.size(); i++ )
    // {
    //   cout << inter_points[i].transpose() << endl;
    // }

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;





   /// PolynomialTraj gl_traj;
  Trajectory traj_;
  
 for(auto j_3= 0; j_3< time.size(); ++j_3)
 {
    if (pos.cols() >= 3)
    ///  gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    {
  double time_budgrt = 0.1;
  
    bool result = front_end_planner_ptr2_->plan(pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time_budgrt, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0);
    if (result)
    {
      front_end_planner_ptr2_->getTraj(traj_);
    }
     else
     {
       return false;
     }
    }
    else if (pos.cols() == 2)
   {
      double time_budgrt = 0.1;
    
     bool result = front_end_planner_ptr2_->plan(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time_budgrt, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0);
     if(result)
     {
       front_end_planner_ptr2_->getTraj(traj_);
     }
     else
     {
       return false;
     }
   }   ///gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
  else
  {
    return false;
  }
 }




    auto time_now = ros::Time::now();
    ///global_data_.setGlobalTraj(gl_traj, time_now);
   global_data_.setGlobalTraj(traj_, time_now);
    return true;
  }

  bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    //写位置矩阵
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;









   /// PolynomialTraj gl_traj;
  Trajectory traj_;
 for(auto j_1= 0; j_1< time.size(); ++j_1)
 {
  ////if (pos.cols() >= 3)
    ///  gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
   /// {
    ///double time_budgrt_1 =time(j_1);
    double time_budgrt_1 =time(0);
    bool result = front_end_planner_ptr2_->plan(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time_budgrt_1, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0);
    if (result)
    {
      front_end_planner_ptr2_->getTraj(traj_);
    }
     else
     {
       return false;
     }
    }





    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(traj_, time_now);

    return true;
  }



  bool EGOPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void EGOPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

void EGOPlannerManager::planYaw(const Eigen::Vector3d& start_yaw) {
  ROS_INFO("yaw replan");
  auto t1 = ros::Time::now();
  // calculate waypoints of heading

  auto&  pos      = local_data_.position_traj_;

  double duration = pos.getTimeSum();

  Eigen::MatrixXd optimal_control_points;

  double dt_yaw  = 0.3;

  int    seg_num = ceil(duration / dt_yaw);

  dt_yaw         = duration / seg_num;

  const double            forward_t = 2.0;

  double                  last_yaw  = start_yaw(0);

  vector<Eigen::Vector3d> waypts;
 
  vector<int>             waypt_idx;

  // seg_num -> seg_num - 1 points for constraint excluding the boundary states

  for (int i = 0; i < seg_num; ++i) {
    double          tc = i * dt_yaw;
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
    double          tf = min(duration, tc + forward_t);
    Eigen::Vector3d pf = pos.evaluateDeBoorT(tf);
    Eigen::Vector3d pd = pf - pc;

    Eigen::Vector3d waypt;
    
    if (pd.norm() > 1e-6) {
      waypt(0) = atan2(pd(1), pd(0));
      waypt(1) = waypt(2) = 0.0;
      calcNextYaw(last_yaw, waypt(0));
    
    } else {
      waypt = waypts.back();
    }
    waypts.push_back(waypt);
    waypt_idx.push_back(i);
  }

  // calculate initial control points with boundary state constraints

  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw, 1.0,
      dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
  yaw.block(0, 0, 3, 1) = states2pts * start_yaw;

  Eigen::Vector3d end_v = local_data_.velocity_traj_.evaluateDeBoorT(duration - 0.1);
  Eigen::Vector3d end_yaw(atan2(end_v(1), end_v(0)), 0, 0);
  calcNextYaw(last_yaw, end_yaw(0));
  yaw.block(seg_num, 0, 3, 1) = states2pts * end_yaw;
  

  // solve
  bspline_optimizer_->setWaypoints(waypts, waypt_idx);
   
  ///int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS;
  ///yaw           = bspline_optimizer_->BsplineOptimizeTraj(yaw, dt_yaw, cost_func, 1, 1);
  yaw           =bspline_optimizer_->BsplineOptimizeTrajReboundRebound(yaw, dt_yaw);

  
  // update traj info



  local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  local_data_.yawdot_traj_    = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();

  vector<double> path_yaw;
  for (int i = 0; i < waypts.size(); ++i) path_yaw.push_back(waypts[i][0]);
  plan_data_.path_yaw_    = path_yaw;
  plan_data_.dt_yaw_      = dt_yaw;
  plan_data_.dt_yaw_path_ = dt_yaw;

  std::cout << "plan heading: " << (ros::Time::now() - t1).toSec() << std::endl;
}

void EGOPlannerManager::calcNextYaw(const double& last_yaw, double& yaw) {
  // round yaw to [-PI, PI]

  double round_last = last_yaw;

  while (round_last < -M_PI) {
    round_last += 2 * M_PI;
  }
  while (round_last > M_PI) {
    round_last -= 2 * M_PI;
  }

  double diff = yaw - round_last;

  if (fabs(diff) <= M_PI) {
    yaw = last_yaw + diff;
  } else if (diff > M_PI) {
    yaw = last_yaw + diff - 2 * M_PI;
  } else if (diff < -M_PI) {
    yaw = last_yaw + diff + 2 * M_PI;
  }
}





  void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace ego_planner
