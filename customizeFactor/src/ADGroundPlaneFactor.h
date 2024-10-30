#ifndef ADGroundPlaneFactor_H
#define ADGroundPlaneFactor_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/AdaptAutoDiff.h>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

// 定義殘差函數 Functor，用於計算誤差
struct ResidualFunctor
{
    gtsam::Vector3 measuredNormal_;
    double measuredDistance_;
    gtsam::Vector3 G_k;

    ResidualFunctor(const gtsam::Vector3 &measuredNormal, double measuredDistance, const gtsam::Vector3 &G_k)
        : measuredNormal_(measuredNormal), measuredDistance_(measuredDistance), G_k(G_k) {}

    // 泛型版本，接受普通的陣列作為輸入
    template <typename T>
    bool operator()(const T *const pose_array, T *residual) const
    {
        // 假設 pose_array 包含 6 個元素，前 3 個是平移，後 3 個是旋轉（李代數形式）
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> translation(pose_array);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> rotation_vec(pose_array + 3);

        // 使用 GTSAM 提供的 Expmap 方法將旋轉向量轉換為旋轉矩陣
        gtsam::Rot3 rotation = gtsam::Rot3::Expmap(rotation_vec);

        // 計算測量的法向量在世界坐標系下的表示
        Eigen::Matrix<T, 3, 1> measuredNormal_W = rotation.matrix() * G_k.cast<T>();

        // 計算參數化後的殘差
        T theta = atan2(measuredNormal_W.y(), measuredNormal_W.x());
        T phi = acos(measuredNormal_W.z() / measuredNormal_W.norm());
        T d_k_prime = measuredDistance_ + measuredNormal_W.dot(translation);

        // 設定殘差
        residual[0] = theta - atan2(T(0.0), T(1.0)); // 假設初始法向量的theta
        residual[1] = phi - acos(T(1.0));            // 假設初始法向量的phi
        residual[2] = d_k_prime - T(0.12);           // 假設的初始距離

        return true;
    }
};

// 定義 ADGroundPlaneFactor
class ADGroundPlaneFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
private:
    gtsam::Vector3 measuredNormal_;
    double measuredDistance_;
    const gtsam::Vector3 G_k;
    gtsam::SharedNoiseModel noiseModel_;

public:
    using NoiseModelFactor1<gtsam::Pose3>::evaluateError;

    ADGroundPlaneFactor(gtsam::Key key, const gtsam::Point3 &normal, const double &distance,
                        const gtsam::SharedNoiseModel &noiseModel, rclcpp::Node::SharedPtr node)
        : gtsam::NoiseModelFactor1<gtsam::Pose3>(noiseModel, key),
          measuredNormal_(normal),
          measuredDistance_(distance),
          G_k(normal.normalized()), // 初始化法向量
          noiseModel_(noiseModel),
          node_(node)
    {
    }

    gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                                boost::optional<gtsam::Matrix &> H = boost::none) const override
    {
        // 使用 AdaptAutoDiff 計算自動微分
        ResidualFunctor functor(measuredNormal_, measuredDistance_, G_k);
        // 定義 Adaptor，並傳入 functor
        // 假設 residual 是殘差的容器，並且 pose 是 GTSAM 的 Pose3 物件
        typedef gtsam::AdaptAutoDiff<ResidualFunctor, 3, 6> Adaptor;

        gtsam::Vector3 residual;
        if (H)
        {
            // 如果需要計算雅可比矩陣
            residual = Adaptor::Evaluate(functor, pose, *H);
        }
        else
        {
            // 僅計算殘差
            residual = Adaptor::Evaluate(functor, pose);
        }

        return residual;
    }

    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new ADGroundPlaneFactor(*this)));
    }

private:
    rclcpp::Node::SharedPtr node_;
};

#endif // ADGroundPlaneFactor_H
