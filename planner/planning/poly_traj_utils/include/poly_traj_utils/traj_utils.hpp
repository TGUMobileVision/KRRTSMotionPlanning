#ifndef TRAJ_UTILS_HPP
#define TRAJ_UTILS_HPP

#include "root_finder.hpp"
#include <vector>
#include <list>
#include <Eigen/Eigen>
#include <iostream>

using std::vector;

// Polynomial order and trajectory dimension are fixed here
//多项式阶数和轨迹维数在这里是固定的
constexpr int TrajOrder = 5;
constexpr int TrajDim = 3;

// Type for piece boundary condition and coefficient matrix
//块边界条件和系数矩阵的类型
typedef Eigen::Matrix<double, TrajDim, TrajOrder + 1> BoundaryCond;
typedef Eigen::Matrix<double, TrajDim, TrajOrder + 1> CoefficientMat;
typedef Eigen::Matrix<double, TrajDim, TrajOrder> VelCoefficientMat;
typedef Eigen::Matrix<double, TrajDim, TrajOrder - 1> AccCoefficientMat;

typedef Eigen::Matrix<double, 6, 1> StatePV;
typedef Eigen::Matrix<double, 9, 1> StatePVA;
typedef Eigen::Matrix<double, TrajDim, 1> ControlJrk;
typedef Eigen::Matrix<double, TrajDim, 1> ControlAcc;

// A single piece of a trajectory, which is indeed a polynomial
//一条轨迹，它确实是一个多项式
class Piece
{
private:
    // Piece(t) = c5*t^5 + c4*t^4 + ... + c1*t + c0
    // The natural coefficient matrix = [c5,c4,c3,c2,c1,c0]
    double duration;
    // Any time in [0, T] is normalized into [0.0, 1.0]
    // Therefore, nCoeffMat = [c5*T^5,c4*T^4,c3*T^3,c2*T^2,c1*T,c0*1]
    // is used for better numerical stability
    CoefficientMat nCoeffMat;

public:
    Piece() = default;

    // Constructor from duration and coefficient
    //持续时间和系数的构造函数
    Piece(double dur, CoefficientMat coeffs) : duration(dur)
    {
        double t = 1.0;
        for (int i = TrajOrder; i >= 0; i--)
        {
            nCoeffMat.col(i) = coeffs.col(i) * t;
            t *= dur;
        }
    }

    // Constructor from boundary condition and duration
    //来自边界条件和持续时间的构造函数
    Piece(BoundaryCond boundCond, double dur) : duration(dur)
    {
        // The BoundaryCond matrix boundCond = [p(0),v(0),a(0),p(T),v(T),a(T)]
        // BoundaryCond 矩阵 boundCond = [p(0),v(0),a(0),p(T),v(T),a(T)]
        double t1 = dur;
        double t2 = t1 * t1;

        // Inverse mapping is computed without explicit matrix inverse
        // It maps boundary condition to normalized coefficient matrix
        // 逆映射是在没有显式逆矩阵的情况下计算的
        // 将边界条件映射到归一化系数矩阵
        nCoeffMat.col(0) = 0.5 * (boundCond.col(5) - boundCond.col(2)) * t2 -
                           3.0 * (boundCond.col(1) + boundCond.col(4)) * t1 +
                           6.0 * (boundCond.col(3) - boundCond.col(0));
        nCoeffMat.col(1) = (-boundCond.col(5) + 1.5 * boundCond.col(2)) * t2 +
                           (8.0 * boundCond.col(1) + 7.0 * boundCond.col(4)) * t1 +
                           15.0 * (-boundCond.col(3) + boundCond.col(0));
        nCoeffMat.col(2) = (0.5 * boundCond.col(5) - 1.5 * boundCond.col(2)) * t2 -
                           (6.0 * boundCond.col(1) + 4.0 * boundCond.col(4)) * t1 +
                           10.0 * (boundCond.col(3) - boundCond.col(0));
        nCoeffMat.col(3) = 0.5 * boundCond.col(2) * t2;
        nCoeffMat.col(4) = boundCond.col(1) * t1;
        nCoeffMat.col(5) = boundCond.col(0);
    }

    inline int getDim() const
    {
        return TrajDim;
    }

    inline int getOrder() const
    {
        return TrajOrder;
    }

    inline double getDuration() const
    {
        return duration;
    }

    // Get the position at time t in this piece
    //获取这件作品在时间 t 的位置

    inline Eigen::Vector3d getPos(double t) const
    {
        // Normalize the time
        //时间正常化
       //std:: cout<<"tttt111"<<t<<std::endl;
       //std:: cout<<"tttt111"<<t<<std::endl;
        t /= duration;
        //std::cout<<"tttt222"<<t<<std::endl;
        Eigen::Vector3d pos(0.0, 0.0, 0.0);
        double tn = 1.0;
        for (int i = TrajOrder; i >= 0; i--)
        {
            pos += tn * nCoeffMat.col(i);
            tn *= t;
        }
        // The pos is not affected by normalization
        //pos不受归一化影响
        return pos;
    }

    // Get the velocity at time t in this piece
    //获取这段时间 t 的速度
    inline Eigen::Vector3d getVel(double t) const
    {
        // Normalize the time
        //时间正常化
        t /= duration;
        Eigen::Vector3d vel(0.0, 0.0, 0.0);
        double tn = 1.0;
        int n = 1;
        for (int i = TrajOrder - 1; i >= 0; i--)
        {
            vel += n * tn * nCoeffMat.col(i);
            tn *= t;
            n++;
        }
        // Recover the actual vel
        //恢复真实的vel
        vel /= duration;
        return vel;
    }

    // Get the acceleration at time t in this piece
    //得到这段时间 t 的加速度
    inline Eigen::Vector3d getAcc(double t) const
    {
        // Normalize the time
        //时间正常化
        t /= duration;
        Eigen::Vector3d acc(0.0, 0.0, 0.0);
        double tn = 1.0;
        int m = 1;
        int n = 2;
        for (int i = TrajOrder - 2; i >= 0; i--)
        {
            acc += m * n * tn * nCoeffMat.col(i);
            tn *= t;
            m++;
            n++;
        }
        // Recover the actual acc
        // 恢复实际acc
        acc /= duration * duration;
        return acc;
    }

    // Get the jerk at time t in this piece
    // 获取这段时间 t 的jerk
    inline Eigen::Vector3d getJerk(double t) const
    {
        // Normalize the time
        t /= duration;
        Eigen::Vector3d jerk(0.0, 0.0, 0.0);
        double tn = 1.0;
        int m = 1;
        int n = 2;
        int k = 3;
        for (int i = TrajOrder - 3; i >= 0; i--)
        {
            jerk += k * m * n * tn * nCoeffMat.col(i);
            tn *= t;
            k++;
            m++;
            n++;
        }
        // Recover the actual acc
        // 恢复实际acc
        jerk /= duration * duration * duration;
        return jerk;
    }

    // Get the boundary condition of this piece
    //得到这块的边界条件
    inline BoundaryCond getBoundCond() const
    {
        BoundaryCond boundCond;
        boundCond << getPos(0.0), getVel(0.0), getAcc(0.0),
            getPos(duration), getVel(duration), getAcc(duration);
        return boundCond;
    }

    // Get the coefficient matrix of the piece
    // Default arg chooses the natural coefficients
    // If normalized version is needed, set the arg true
    // 获取棋子的系数矩阵
     // 默认参数选择自然系数
     // 如果需要规范化版本，则将 arg 设置为 true
    inline CoefficientMat getCoeffMat(bool normalized = false) const
    {
        CoefficientMat posCoeffsMat;
        double t = 1;
        for (int i = TrajOrder; i >= 0; i--)
        {
            posCoeffsMat.col(i) = nCoeffMat.col(i) / t;
            t *= normalized ? 1.0 : duration;
        }
        return posCoeffsMat;
    }

    // Get the polynomial coefficients of velocity of this piece
    // Default arg chooses the natural coefficients
    // If normalized version is needed, set the arg true
    // 得到这块速度的多项式系数
     // 默认参数选择自然系数
     // 如果需要规范化版本，则将 arg 设置为 true
    inline VelCoefficientMat getVelCoeffMat(bool normalized = false) const
    {
        VelCoefficientMat velCoeffMat;
        int n = 1;
        double t = 1.0;
        t *= normalized ? 1.0 : duration;
        for (int i = TrajOrder - 1; i >= 0; i--)
        {
            velCoeffMat.col(i) = n * nCoeffMat.col(i) / t;
            n++;
            t *= normalized ? 1.0 : duration;
        }
        return velCoeffMat;
    }

    // Get the polynomial coefficients of acceleration of this piece
    // Default arg chooses the natural coefficients
    // If normalized version is needed, set the arg true
    // 得到这块加速度的多项式系数
     // 默认参数选择自然系数
     // 如果需要规范化版本，则将 arg 设置为 true
    inline AccCoefficientMat getAccCoeffMat(bool normalized = false) const
    {
        AccCoefficientMat accCoeffMat;
        int n = 2;
        int m = 1;
        double t = 1.0;
        t *= normalized ? 1.0 : duration * duration;
        for (int i = TrajOrder - 2; i >= 0; i--)
        {
            accCoeffMat.col(i) = n * m * nCoeffMat.col(i) / t;
            n++;
            m++;
            t *= normalized ? 1.0 : duration;
        }
        return accCoeffMat;
    }

    // Get the max velocity rate of the piece
    // 获取棋子的最大速度
    inline double getMaxVelRate() const
    {
        // Compute normalized squared vel norm polynomial coefficient matrix
        // 计算归一化平方 vel 范数多项式系数矩阵
        Eigen::MatrixXd nVelCoeffMat = getVelCoeffMat(true);
        Eigen::VectorXd coeff = RootFinder::polySqr(nVelCoeffMat.row(0)) +
                                RootFinder::polySqr(nVelCoeffMat.row(1)) +
                                RootFinder::polySqr(nVelCoeffMat.row(2));
        int N = coeff.size();
        int n = N - 1;
        for (int i = 0; i < N; i++)
        {
            coeff(i) *= n;
            n--;
        }
        if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON)
        {
            return 0.0;
        }
        else
        {
            // Search an open interval whose boundaries are not zeros
            // 搜索边界不为零的开区间
            double l = -0.0625;
            double r = 1.0625;
            while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON)
            {
                l = 0.5 * l;
            }
            while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON)
            {
                r = 0.5 * (r + 1.0);
            }
            // Find all stationaries
            // 查找所有固定点
            std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                      FLT_EPSILON / duration);

            // Check boundary points and stationaries within duration
            // 检查持续时间内的边界点和静止点
            candidates.insert(0.0);
            candidates.insert(1.0);
            double maxVelRateSqr = -INFINITY;
            double tempNormSqr;
            for (std::set<double>::const_iterator it = candidates.begin();
                 it != candidates.end();
                 it++)
            {
                if (0.0 <= *it && 1.0 >= *it)
                {
                    // Recover the actual time then get the vel squared norm
                    // 恢复实际时间然后得到vel平方范数
                    tempNormSqr = getVel((*it) * duration).squaredNorm();
                    maxVelRateSqr = maxVelRateSqr < tempNormSqr ? tempNormSqr : maxVelRateSqr;
                }
            }
            return sqrt(maxVelRateSqr);
        }
    }

    // Get the max acceleration rate of the piece
    //获取工件的最大加速度
    inline double getMaxAccRate() const
    {
        // Compute normalized squared acc norm polynomial coefficient matrix
        //计算归一化平方acc范数多项式系数矩阵
        Eigen::MatrixXd nAccCoeffMat = getAccCoeffMat(true);
        Eigen::VectorXd coeff = RootFinder::polySqr(nAccCoeffMat.row(0)) +
                                RootFinder::polySqr(nAccCoeffMat.row(1)) +
                                RootFinder::polySqr(nAccCoeffMat.row(2));
        int N = coeff.size();
        int n = N - 1;
        for (int i = 0; i < N; i++)
        {
            coeff(i) *= n;
            n--;
        }
        if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON)
        {
            return 0.0;
        }
        else
        {
            // Search an open interval whose boundaries are not zeros
            // 搜索边界不为零的开区间
            double l = -0.0625;
            double r = 1.0625;
            while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON)
            {
                l = 0.5 * l;
            }
            while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON)
            {
                r = 0.5 * (r + 1.0);
            }
            // Find all stationaries
            // 查找所有固定点
            std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                      FLT_EPSILON / duration);
            // Check boundary points and stationaries within duration
            // 检查持续时间内的边界点和静止点
            candidates.insert(0.0);
            candidates.insert(1.0);
            double maxAccRateSqr = -INFINITY;
            double tempNormSqr;
            for (std::set<double>::const_iterator it = candidates.begin();
                 it != candidates.end();
                 it++)
            {
                if (0.0 <= *it && 1.0 >= *it)
                {
                    // Recover the actual time then get the acc squared norm
                    //恢复实际时间然后得到acc平方范数
                    tempNormSqr = getAcc((*it) * duration).squaredNorm();
                    maxAccRateSqr = maxAccRateSqr < tempNormSqr ? tempNormSqr : maxAccRateSqr;
                }
            }
            return sqrt(maxAccRateSqr);
        }
    }

    // Check whether velocity rate of the piece is always less than maxVelRate
    // 检查棋子的速度是否总是小于maxVelRate
    inline bool checkMaxVelRate(double maxVelRate) const
    {
        double sqrMaxVelRate = maxVelRate * maxVelRate;
        if (getVel(0.0).squaredNorm() >= sqrMaxVelRate ||
            getVel(duration).squaredNorm() >= sqrMaxVelRate)
        {
            return false;
        }
        else
        {
            Eigen::MatrixXd nVelCoeffMat = getVelCoeffMat(true);
            Eigen::VectorXd coeff = RootFinder::polySqr(nVelCoeffMat.row(0)) +
                                    RootFinder::polySqr(nVelCoeffMat.row(1)) +
                                    RootFinder::polySqr(nVelCoeffMat.row(2));
            // Convert the actual squared maxVelRate to a normalized one
            //将实际平方 maxVelRate 转换为归一化的
            double t2 = duration * duration;
            coeff.tail<1>()(0) -= sqrMaxVelRate * t2;
            // Directly check the root existence in the normalized interval
            //直接检查归一化区间内的根是否存在
            return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
        }
    }

    // Check whether accleration rate of the piece is always less than maxAccRate
    //检查工件的加速度是否总是小于max AccuRate
    inline bool checkMaxAccRate(double maxAccRate) const
    {
        double sqrMaxAccRate = maxAccRate * maxAccRate;
        if (getAcc(0.0).squaredNorm() >= sqrMaxAccRate ||
            getAcc(duration).squaredNorm() >= sqrMaxAccRate)
        {
            return false;
        }
        else
        {
            Eigen::MatrixXd nAccCoeffMat = getAccCoeffMat(true);
            Eigen::VectorXd coeff = RootFinder::polySqr(nAccCoeffMat.row(0)) +
                                    RootFinder::polySqr(nAccCoeffMat.row(1)) +
                                    RootFinder::polySqr(nAccCoeffMat.row(2));
            // Convert the actual squared maxAccRate to a normalized one
            //将实际平方 maxAccRate 转换为归一化的
            double t2 = duration * duration;
            double t4 = t2 * t2;
            coeff.tail<1>()(0) -= sqrMaxAccRate * t4;
            // Directly check the root existence in the normalized interval
            //// 直接检查归一化区间内的根是否存在
            return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
        }
    }

    //Scale the Piece(t) to Piece(k*t)
    inline void scaleTime(double k)
    {
        duration /= k;
        return;
    }

    inline void sampleOneSeg(std::vector< StatePVA >* vis_x) const 
    {
        double dt = 0.005;
        for (double t = 0.0; t < duration; t += dt) 
        {
            Eigen::Vector3d pos, vel, acc;
            pos = getPos(t);
            vel = getVel(t);
            acc = getAcc(t);
            StatePVA x;
            x << pos(0), pos(1), pos(2), vel(0), vel(1), vel(2), acc(0), acc(1), acc(2);
            vis_x->push_back(x);
        }
    }

};

// A whole trajectory which contains multiple pieces
class Trajectory
{
private:
    typedef std::vector<Piece> Pieces;
    Pieces pieces;

 vector<double> times;       // time of each segment
  vector<vector<double>> cxs; // coefficient of x of each segment, from n-1 -> 0
  vector<vector<double>> cys; // coefficient of y of each segment
  vector<vector<double>> czs; // coefficient of z of each segmen

  
public:
    Trajectory() = default;

    // Constructor from durations and coefficient matrices
    //持续时间和系数矩阵的构造函数
    Trajectory(const std::vector<double> &durs,
               const std::vector<CoefficientMat> &coeffMats)
    {
        int N = std::min(durs.size(), coeffMats.size());
        for (int i = 0; i < N; i++)
        {
            pieces.emplace_back(durs[i], coeffMats[i]);
        }
    }

    inline int getPieceNum() const
    {
        return pieces.size();
    }

    // Get durations vector of all pieces
    // 获取所有片段的持续时间向量
    inline std::vector<double> getDurations() const
    {
        std::vector<double> durations;
        durations.reserve(getPieceNum());
        for (int i = 0; i < getPieceNum(); i++)
        {
            durations.push_back(pieces[i].getDuration());
        }
        return durations;
    }

    // Get total duration of the trajectory
    // 获取轨迹的总持续时间
    inline double getTotalDuration() const
    {
        double totalDuration = 0.0;
        for (int i = 0; i < getPieceNum(); i++)
        {
            totalDuration += pieces[i].getDuration();
        }
        return totalDuration;
    }

    // Reload the operator[] to reach the i-th piece
     // 重新加载 operator[] 以到达第 i 块 
    inline const Piece &operator[](int i) const
    {
        return pieces[i];
    }

    inline Piece &operator[](int i)
    {
        return pieces[i];
    }

    inline void clear(void)
    {
        pieces.clear();
    }

    inline Pieces::const_iterator begin() const
    {
        return pieces.begin();
    }

    inline Pieces::const_iterator end() const
    {
        return pieces.end();
    }

    // Put another piece at the tail of this trajectory
    // 在这条轨迹的尾部再放一块
    inline void emplace_back(const Piece &piece)
    {
        pieces.emplace_back(piece);
        return;
    }

    // Two corresponding constructors of Piece both are supported here
    // 这里支持两个对应的 Piece 构造函数
    template <typename ArgTypeL, typename ArgTypeR>
    inline void emplace_back(const ArgTypeL &argL, const ArgTypeR &argR)
    {
        pieces.emplace_back(argL, argR);
        return;
    }

    // Append another Trajectory at the tail of this trajectory
    // 在此轨迹的尾部附加另一个轨迹
    inline void append(const Trajectory &traj)
    {
        pieces.insert(pieces.end(), traj.begin(), traj.end());
        return;
    }

    // Find the piece at which the time t is located
    // The index is returned and the offset in t is removed
    // 找到时间t所在的棋子
     // 返回索引并移除 t 中的偏移量
    inline int locatePieceIdx(double &t) const
    {
        int idx;
        double dur;
        for (idx = 0;
             idx < getPieceNum() &&
             t > (dur = pieces[idx].getDuration());
             idx++)
           
        {
            t -= dur;
        }
          
        if (idx == getPieceNum())
        {
           ///  std::cout << "idxxxxxxxxxxxxxxxxxxxxx" << idx << std::endl;
            idx--;
            t += pieces[idx].getDuration();
        }
       // std::cout << "idxxxxxxxxxxxxxxxxxxxxx" << idx << std::endl;
        return idx;

      
    }

    // Get the position at time t of the trajectory
    //获取轨迹在时间 t 的位置
    inline Eigen::Vector3d getPos(double t) const
    {
         //std:: cout<<"******************BBBBBBBBBBBBBBB"<<t<<std::endl;
          //std:: cout<<"******************AAAAAAAAAAAAAAAAA"<<getTotalDuration()<<std::endl;
        int pieceIdx = locatePieceIdx(t);
        return pieces[pieceIdx].getPos(t);
    }

    // Get the velocity at time t of the trajectory
    //获取轨迹在时间 t 的速度
    inline Eigen::Vector3d getVel(double t) const
    {
        int pieceIdx = locatePieceIdx(t);
        return pieces[pieceIdx].getVel(t);
    }

    // Get the acceleration at time t of the trajectory
    //获取轨迹在时间 t 处的加速度
    inline Eigen::Vector3d getAcc(double t) const
    {
        int pieceIdx = locatePieceIdx(t);
        return pieces[pieceIdx].getAcc(t);
    }

    // Get the position at the juncIdx-th waypoint
    //获取第 juncIdx 个航点的位置
    inline Eigen::Vector3d getJuncPos(int juncIdx) const
    {
        if (juncIdx != getPieceNum())
        {
            return pieces[juncIdx].getPos(0.0);
        }
        else
        {
            return pieces[juncIdx - 1].getPos(pieces[juncIdx - 1].getDuration());
        }
    }

    // Get the velocity at the juncIdx-th waypoint
    //获取第 juncIdx 个航点处的速度
    inline Eigen::Vector3d getJuncVel(int juncIdx) const
    {
        if (juncIdx != getPieceNum())
        {
            return pieces[juncIdx].getVel(0.0);
        }
        else
        {
            return pieces[juncIdx - 1].getVel(pieces[juncIdx - 1].getDuration());
        }
    }

    // Get the acceleration at the juncIdx-th waypoint
    //获取第 juncIdx 个航点处的加速度
    inline Eigen::Vector3d getJuncAcc(int juncIdx) const
    {
        if (juncIdx != getPieceNum())
        {
            return pieces[juncIdx].getAcc(0.0);
        }
        else
        {
            return pieces[juncIdx - 1].getAcc(pieces[juncIdx - 1].getDuration());
        }
    }

    // Get the max velocity rate of the trajectory
    //获取轨迹的最大速度
    inline double getMaxVelRate() const
    {
        double maxVelRate = -INFINITY;
        double tempNorm;
        for (int i = 0; i < getPieceNum(); i++)
        {
            tempNorm = pieces[i].getMaxVelRate();
            maxVelRate = maxVelRate < tempNorm ? tempNorm : maxVelRate;
        }
        return maxVelRate;
    }

    // Get the max acceleration rate of the trajectory
    //获取轨迹的最大加速度
    inline double getMaxAccRate() const
    {
        double maxAccRate = -INFINITY;
        double tempNorm;
        for (int i = 0; i < getPieceNum(); i++)
        {
            tempNorm = pieces[i].getMaxAccRate();
            maxAccRate = maxAccRate < tempNorm ? tempNorm : maxAccRate;
        }
        return maxAccRate;
    }

    // Check whether the velocity rate of this trajectory exceeds the threshold
    //检查该轨迹的速度是否超过阈值
    inline bool checkMaxVelRate(double maxVelRate) const
    {
        bool feasible = true;
        for (int i = 0; i < getPieceNum() && feasible; i++)
        {
            feasible = feasible && pieces[i].checkMaxVelRate(maxVelRate);
        }
        return feasible;
    }

    // Check whether the acceleration rate of this trajectory exceeds the threshold
    //检查该轨迹的加速度是否超过阈值
    inline bool checkMaxAccRate(double maxAccRate) const
    {
        bool feasible = true;
        for (int i = 0; i < getPieceNum() && feasible; i++)
        {
            feasible = feasible && pieces[i].checkMaxAccRate(maxAccRate);
        }
        return feasible;
    }

    // Scale the Trajectory(t) to Trajectory(k*t)
    inline void scaleTime(double k)
    {
        for (int i = 0; i < getPieceNum(); i++)
        {
            pieces[i].scaleTime(k);
        }
    }

    inline void sampleWholeTrajectory(std::vector< StatePVA >* vis_x) const 
    {
        int n = getPieceNum();
        for (int i = 0; i < n; ++i)
        {
            pieces[i].sampleOneSeg(vis_x);
        }
    }





 







    inline double evaluateTrajJerk() const
    {
        double objective = 0.0;
        int M = getPieceNum();
        CoefficientMat cMat;
        double t1, t2, t3, t4, t5;
        for (int i = 0; i < M; i++)
        {
            cMat = operator[](i).getCoeffMat();
            t1 = operator[](i).getDuration();
            t2 = t1 * t1;
            t3 = t2 * t1;
            t4 = t2 * t2;
            t5 = t2 * t3;
            objective += 36.0 * cMat.col(2).squaredNorm() * t1 +
                        144.0 * cMat.col(1).dot(cMat.col(2)) * t2 +
                        192.0 * cMat.col(1).squaredNorm() * t3 +
                        240.0 * cMat.col(0).dot(cMat.col(2)) * t3 +
                        720.0 * cMat.col(0).dot(cMat.col(1)) * t4 +
                        720.0 * cMat.col(0).squaredNorm() * t5;
        }
        return objective;
    }
};

#endif
