#include "point_to_point.h"
// JacobiSVD 属 Eigen/SVD 模块（Eigen/Core 不含），原工程经 <ceres/ceres.h> 间接引入，此处显式包含
#include <eigen3/Eigen/LU>   // 2x2 determinant 特化在此（原由 <ceres/ceres.h> 间接引入）
#include <eigen3/Eigen/SVD>   // JacobiSVD（原由 <ceres/ceres.h> 间接引入）

namespace slam
{

    PointToPoint::PointToPoint(int max_iter_times)
        : max_iter_times_(max_iter_times)
    {
    }
    /**
     * @brief 把目标点和源点存储在pairs_中
     *
     * @param source
     * @param target
     */
    void PointToPoint::AddPair(const unity::PointType &source, const unity::PointType &target)
    {
        pairs_.push_back({source, target});
    }

    /**
     * @brief 使用最小二乘法对点云数据进行配准
     *
     * @param result 位姿
     * @return true
     * @return false
     */
    bool PointToPoint::Solve(unity::Rigid2f &result)
    {
        // 检查点对数量，至少需要3个点才能确定2D刚体变换（旋转+平移）
        // 因为2D刚体变换有3个自由度（旋转角θ，平移tx，平移ty），需要至少2个点（提供4个方程）
        // 但实际中为了数值稳定性，我们要求至少3个点对
        if (pairs_.size() < 3)
            return false;

        // 步骤1：准备数据矩阵
        // 创建2行N列的矩阵，每列存储一个点的(x,y)坐标
        // source_points存储源点云坐标，target_points存储目标点云坐标
        Eigen::MatrixXf source_points(2, pairs_.size());
        Eigen::MatrixXf target_points(2, pairs_.size());

        // 步骤2：计算质心（几何中心）
        // 质心计算公式：μ = (1/n) * Σ point_i
        Eigen::Vector2f source_mean(0, 0), target_mean(0, 0);
        for (size_t i = 0; i < pairs_.size(); ++i)
        {
            // 将点对分别存入矩阵列中
            source_points.col(i) = pairs_[i].first;
            target_points.col(i) = pairs_[i].second;

            // 累加计算质心坐标
            source_mean += pairs_[i].first;  // 源点云质心x,y分量累加
            target_mean += pairs_[i].second; // 目标点云质心x,y分量累加
        }
        // 完成质心计算（除以点对数量）
        source_mean /= pairs_.size(); // μ_s = (μ_sx, μ_sy)
        target_mean /= pairs_.size(); // μ_t = (μ_tx, μ_ty)

        // 步骤3：数据去中心化（减去质心）
        // 目的：将问题转化为纯旋转问题，平移分量可以通过质心差异计算
        // 数学原理：平移t = μ_t - R*μ_s （最后计算）
        source_points.colwise() -= source_mean; // 每个源点：p'_i = p_i - μ_s
        target_points.colwise() -= target_mean; // 每个目标点：q'_i = q_i - μ_t

        // 步骤4：构建协方差矩阵H
        // H = Σ (p'_i * q'_i^T) = source_points * target_points^T
        // 这个矩阵包含了源点和目标点之间的相关性信息
        Eigen::Matrix2f H = source_points * target_points.transpose();

        // 步骤5：奇异值分解（SVD）
        // 分解 H = U * Σ * V^T，其中U和V是正交矩阵，Σ是对角矩阵
        // 这是求解旋转矩阵的关键步骤
        Eigen::JacobiSVD<Eigen::Matrix2f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

        // 步骤6：计算最优旋转矩阵
        // 根据正交Procrustes问题的解，最优旋转 R = V * U^T
        Eigen::Matrix2f R = svd.matrixV() * svd.matrixU().transpose();

        // 步骤7：处理反射情况（确保是纯旋转没有镜像）
        // 旋转矩阵的行列式应该为1，如果为-1说明包含反射
        if (R.determinant() < 0)
        {
            // 修正反射：将V矩阵的第二列取反
            Eigen::Matrix2f V = svd.matrixV();
            V.col(1) *= -1;
            R = V * svd.matrixU().transpose();
        }

        // 步骤8：计算平移向量
        // 根据公式 t = μ_t - R * μ_s
        Eigen::Vector2f t = target_mean - R * source_mean;

        // 步骤9：将旋转矩阵转换为旋转角度
        // 旋转矩阵形式：
        // [ cosθ  -sinθ ]
        // [ sinθ   cosθ ]
        // 通过矩阵元素计算旋转角θ（单位：弧度）
        float theta = std::atan2(R(1, 0), R(0, 0)); // θ = arctan(sinθ/cosθ)

        // 步骤10：封装结果
        // Rigid2f构造函数参数顺序为：旋转角θ，x平移量，y平移量
        result = unity::Rigid2f(theta, t(0), t(1));
        return true;
    }
}