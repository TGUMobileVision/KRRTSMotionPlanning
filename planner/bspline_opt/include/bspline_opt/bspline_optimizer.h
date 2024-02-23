#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <bspline_opt/uniform_bspline.h>
//#include <plan_env/grid_map.h>
#include <plan_env/obj_predictor.h>
#include <ros/ros.h>
#include "bspline_opt/lbfgs.hpp"
#include <traj_utils/plan_container.hpp>
#include "occ_grid/pos_checker.h"
#include "occ_grid/occ_map.h"


// Gradient and elasitc band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point
//// 梯度和弹性带优化
// 输入：一个带符号的距离场和一个点序列
// 输出：优化后的点序列
// 点的格式：N x 3 矩阵，每一行是一个点
namespace ego_planner
{

  class ControlPoints
  {
  public:
    double clearance;
    int size;
    Eigen::MatrixXd points;
    std::vector<std::vector<Eigen::Vector3d>> base_point; // The point at the statrt of the direction vector (collision point) //方向向量的起点（碰撞点）
    std::vector<std::vector<Eigen::Vector3d>> direction;  // Direction vector, must be normalized.  //方向向量，必须归一化。
    std::vector<bool> flag_temp;                          // A flag that used in many places. Initialize it everytime before using it. //在许多地方使用的标志。 每次使用前都要初始化。
    // std::vector<bool> occupancy;

    void resize(const int size_set)
    {
      size = size_set;

      base_point.clear();
      direction.clear();
      flag_temp.clear();
      // occupancy.clear();

      points.resize(3, size_set);
      base_point.resize(size);
      direction.resize(size);
      flag_temp.resize(size);
      // occupancy.resize(size);
    }

    void segment(ControlPoints &buf, const int start, const int end)
    {
      if (start < 0 || end >= size || points.rows() != 3)
      {
        ROS_ERROR("Wrong segment index! start=%d, end=%d", start, end);
        return;
      }

      buf.resize(end - start + 1);
      buf.points = points.block(0, start, 3, end - start + 1);
      buf.clearance = clearance;
      buf.size = end - start + 1;
      for (int i = start; i <= end; i++)
      {
        buf.base_point[i - start] = base_point[i];
        buf.direction[i - start] = direction[i];

        // if ( buf.base_point[i - start].size() > 1 )
        // {
        //   ROS_ERROR("buf.base_point[i - start].size()=%d, base_point[i].size()=%d", buf.base_point[i - start].size(), base_point[i].size());
        // }
      }

      // cout << "RichInfoOneSeg_temp, insede" << endl;
      // for ( int k=0; k<buf.size; k++ )
      //   if ( buf.base_point[k].size() > 0 )
      //   {
      //     cout << "###" << buf.points.col(k).transpose() << endl;
      //     for (int k2 = 0; k2 < buf.base_point[k].size(); k2++)
      //     {
      //       cout << "      " << buf.base_point[k][k2].transpose() << " @ " << buf.direction[k][k2].transpose() << endl;
      //     }
      //   }
    }
  };

  class BsplineOptimizer
  {

  public:



  static const int SMOOTHNESS;
  static const int DISTANCE;
  static const int FEASIBILITY;
  static const int ENDPOINT;
  static const int GUIDE;
  static const int WAYPOINTS;

  static const int GUIDE_PHASE;
  static const int NORMAL_PHASE;

    BsplineOptimizer() {}
    ~BsplineOptimizer() {}

    /* main API */
    void setEnvironment(const tgk_planner::PosChecker::Ptr &checker);
    void setEnvironment(const tgk_planner::PosChecker::Ptr &checker, const fast_planner::ObjPredictor::Ptr mov_obj);
    void setParam(ros::NodeHandle &nh);
    Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd &points, const double &ts,
                                        const int &cost_function, int max_num_id, int max_time_id);

    /* helper function */

    // required inputs
    void setControlPoints(const Eigen::MatrixXd &points);
    void setBsplineInterval(const double &ts);
    void setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr);
    void setDroneId(const int drone_id);

    // optional inputs
    void setGuidePath(const vector<Eigen::Vector3d> &guide_pt);
    void setWaypoints(const vector<Eigen::Vector3d> &waypts,
                      const vector<int> &waypt_idx); // N-2 constraints at most
    void setLocalTargetPt(const Eigen::Vector3d local_target_pt)
     { 
          ///cout<<"********************youhuaqian"<< local_target_pt<<endl;
       local_target_pt_ = local_target_pt; };

    void optimize();

    ControlPoints getControlPoints() { return cps_; };

    AStar::Ptr a_star_;
    std::vector<Eigen::Vector3d> ref_pts_;

    std::vector<ControlPoints> distinctiveTrajs(vector<std::pair<int, int>> segments);
    std::vector<std::pair<int, int>> initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init = true);
    bool BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts); // must be called after initControlPoints()
    bool BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double &final_cost, const ControlPoints &control_points, double ts);
    bool BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points);
     Eigen::MatrixXd BsplineOptimizeTrajRefineRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points);
     Eigen::MatrixXd BsplineOptimizeTrajReboundRebound(Eigen::MatrixXd &optimal_points, double ts);
    inline double getSwarmClearance(void) { return swarm_clearance_; }

  private:
    //GridMap::Ptr grid_map_;
    tgk_planner::PosChecker::Ptr pos_checker_;
    tgk_planner::PosChecker::Ptr occ_map_;
    fast_planner::ObjPredictor::Ptr moving_objs_;
    SwarmTrajData *swarm_trajs_{NULL}; // Can not use shared_ptr and no need to free
    int drone_id_;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    // main input
    // Eigen::MatrixXd control_points_;     // B-spline control points, N x dim
    double bspline_interval_; // B-spline knot span
    Eigen::Vector3d end_pt_;  // end of the trajectory
    // int             dim_;                // dimension of the B-spline
    //
    vector<Eigen::Vector3d> guide_pts_; // geometric guiding path points, N-6
    vector<Eigen::Vector3d> waypoints_; // waypts constraints
    vector<int> waypt_idx_;             // waypts constraints index
                                        //
    int max_num_id_, max_time_id_;      // stopping criteria
    int cost_function_;                 // used to determine objective function
    double start_time_;                 // global time for moving obstacles

    /* optimization parameters */
    int order_;                    // bspline degree
    double lambda1_;               // jerk smoothness weight
    double lambda2_, new_lambda2_; // distance weight
    double lambda3_;               // feasibility weight
    double lambda4_;               // curve fitting

    double lambda1_new;               // jerk smoothness weight
    double new_new_lambda2_; // distance weight
    double lambda2_new; // distance weight
    double lambda3_new;               // feasibility weight
    double lambda4_new;               // curve fitting


   double lambda1_re;               // jerk smoothness weight
    double lambda2_re; // distance weight
    double lambda3_re;               // feasibility weight
    double lambda4_re;               // curve fitting



    int a;
    //
    double dist0_, swarm_clearance_; // safe distance
    double max_vel_, max_acc_;       // dynamic limits

    int variable_num_;              // optimization variables
    int iter_num_;                  // iteration of the solver
    Eigen::VectorXd best_variable_; //
    double min_cost_;               //

    Eigen::Vector3d local_target_pt_; 

#define INIT_min_ellip_dist_ 123456789.0123456789
    double min_ellip_dist_;

    ControlPoints cps_;

    /* cost function */
    /* calculate each part of cost function with control points q as input */

    static double costFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data);
    void combineCost(const std::vector<double> &x, vector<double> &grad, double &cost);

    // q contains all control points
    void calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, bool falg_use_jerk = true);
    void newcalcSmoothnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, bool falg_use_jerk = false);
    void calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    void newvcalcFeasibilityCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    void newacalcFeasibilityCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    void calcTerminalCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
  
    void newyawcalcDistanceCostRebound (const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost);
    void newcalcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost);
    void calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost);
    void calcSwarmCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    void calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    bool check_collision_and_rebound(void);

    static int earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls);
    static double costFunctionRebound(void *func_data, const double *x, double *grad, const int n);
    static double costFunctionReboundRebound(void *func_data, const double *x, double *grad, const int n);
    static double costFunctionReboundReboundRebound(void *func_data, const double *x, double *grad, const int n);
    static double costFunctionRefine(void *func_data, const double *x, double *grad, const int n);

    bool rebound_optimize(double &final_cost);
    bool reboundrebound_optimize(double &final_cost);
    bool reboundreboundrebound_optimize(double &final_cost);
    bool refine_optimize();
    void combineCostRebound(const double *x, double *grad, double &f_combine, const int n);
    void combineCostReboundRebound(const double *x, double *grad, double &f_combine, const int n);
    void old_combineCostReboundRebound(const double *x, double *grad, double &f_combine, const int n);
    void combineCostReboundReboundRebound(const double *x, double *grad, double &f_combine, const int n);
    void combineCostRefine(const double *x, double *grad, double &f_combine, const int n);

    /* for benckmark evaluation only */
  public:
    typedef unique_ptr<BsplineOptimizer> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner
#endif