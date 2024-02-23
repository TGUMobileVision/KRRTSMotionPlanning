#ifndef _UNIFORM_BSPLINE_H_
#define _UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

using namespace std;

namespace ego_planner
{
  // An implementation of non-uniform B-spline with different dimensions
  // It also represents uniform B-spline which is a special case of non-uniform
  // 不同维度的非均匀 B 样条的实现
   // 它也表示均匀 B 样条，这是非均匀的一种特殊情况
  class UniformBspline
  {
  private:
    // control points for B-spline with different dimensions.
    // Each row represents one single control point
    // The dimension is determined by column number
    // e.g. B-spline with N points in 3D space -> Nx3 matrix
// 不同维度的 B 样条的控制点。
// 每行代表一个控制点
// 维度由列号决定
// 例如 在 3D 空间中具有 N 个点的 B 样条 -> Nx3 矩阵
    Eigen::MatrixXd control_points_;
    
  int order_; 
   double max_vel_, max_acc_;   

   
    int p_, n_, m_;     // p degree, n+1 control points, m = n+p+1//p 度，n+1 个控制点，m = n+p+1
    Eigen::VectorXd u_; // knots vector //// 结向量
    double interval_;   // knot span \delta t // 结跨度 \delta t

    Eigen::MatrixXd getDerivativeControlPoints();

    double limit_vel_, limit_acc_, limit_ratio_, feasibility_tolerance_; // physical limits and time adjustment ratio  //// 物理限制和时间调整比例

  public:
    UniformBspline() {}
    UniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);
    ~UniformBspline();

    Eigen::MatrixXd get_control_points(void) { return control_points_; }

    // initialize as an uniform B-spline
    //// 初始化为一个统一的 B 样条
    void setUniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);

    // get / set basic bspline info
    //// 获取/设置基本 bspline 信息

    void setKnot(const Eigen::VectorXd &knot);
    Eigen::VectorXd getKnot();
    Eigen::MatrixXd getControlPoint();
    double getInterval();
    bool getTimeSpan(double &um, double &um_p);

    // compute position / derivative
 ////计算位置/导数
    Eigen::VectorXd evaluateDeBoor(const double &u);                                               // use u \in [up, u_mp]
    inline Eigen::VectorXd evaluateDeBoorT(const double &t) { return evaluateDeBoor(t + u_(p_)); } // use t \in [0, duration]
    UniformBspline getDerivative();

    // 3D B-spline interpolation of points in point_set, with boundary vel&acc
    // constraints
    // input : (K+2) points with boundary vel/acc; ts
    // output: (K+6) control_pts
    // point_set 中点的 3D B 样条插值，带边界 vel&acc
     // 约束
     // 输入 : (K+2) 个边界为 vel/acc 的点； ts
     // 输出：(K+6) control_pts
    static void parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                      const vector<Eigen::Vector3d> &start_end_derivative,
                                      Eigen::MatrixXd &ctrl_pts);

    /* check feasibility, adjust time */
/* 检查可行性，调整时间 */
    void setPhysicalLimits(const double &vel, const double &acc, const double &tolerance);
    bool checkFeasibility(double &ratio, bool show = false);
    void lengthenTime(const double &ratio);

    /* for performance evaluation */
    /* 用于性能评估 */
    double getTimeSum();
    double getLength(const double &res = 0.01);
    double getJerk();
    void getMeanAndMaxVel(double &mean_v, double &max_v);
    void getMeanAndMaxAcc(double &mean_a, double &max_a);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace ego_planner
#endif