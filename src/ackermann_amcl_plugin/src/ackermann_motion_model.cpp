// #include "nav2_amcl/motion_model/ackermann_motion_model.hpp"
// #include <cmath>
// #include <algorithm>

// namespace nav2_amcl
// {

// void
// AckermannMotionModel::initialize(
//   double alpha1, double alpha2, double alpha3, double alpha4,
//   double alpha5)
// {
//   alpha1_ = alpha1;
//   alpha2_ = alpha2;
//   alpha3_ = alpha3;
//   alpha4_ = alpha4;
//   alpha5_ = alpha5;
// }

// void
// AckermannMotionModel::odometryUpdate(
//   pf_t * pf, const pf_vector_t & pose,
//   const pf_vector_t & delta)
// {
//   pf_sample_set_t * set = pf->sets + pf->current_set;

//   // 基于 odom 增量计算运动分解
//   double delta_x = delta.v[0];
//   double delta_y = delta.v[1];
//   double delta_theta = delta.v[2];

//   double delta_trans = sqrt(delta_x * delta_x + delta_y * delta_y);

//   // 近似反推出方向角，适用于阿克曼车辆的前轮转向轨迹
//   double steering_angle;
//   if (fabs(delta_theta) > 1e-5 && delta_trans > 1e-5) {
//     // 转弯半径
//     double turning_radius = delta_trans / delta_theta;
//     // 假设车轴距（wheelbase）L = 0.35 米
//     double L = 0.35;
//     steering_angle = atan(L / turning_radius);
//   } else {
//     steering_angle = 0.0;
//   }

//   // 给运动加入噪声（高斯采样）
//   for (int i = 0; i < set->sample_count; ++i) {
//     pf_sample_t * sample = set->samples + i;

//     // 给定 steering_angle 和 delta_trans，我们可以从车辆当前角度进行增量更新
//     double noisy_trans = delta_trans + pf_ran_gaussian(sqrt(alpha3_ * delta_trans * delta_trans));
//     double noisy_theta = delta_theta + pf_ran_gaussian(sqrt(alpha1_ * delta_theta * delta_theta));

//     // 使用阿克曼运动模型推算下一位置
//     double theta = sample->pose.v[2];

//     sample->pose.v[0] += noisy_trans * cos(theta);
//     sample->pose.v[1] += noisy_trans * sin(theta);
//     sample->pose.v[2] += noisy_theta;
//   }
// }

// }  // namespace nav2_amcl

// #include <pluginlib/class_list_macros.hpp>
// PLUGINLIB_EXPORT_CLASS(nav2_amcl::AckermannMotionModel, nav2_amcl::MotionModel)

#include "ackermann_amcl_plugin/ackermann_motion_model.hpp"
#include <cmath>
#include <algorithm>
namespace nav2_amcl
{

void
AckermannMotionModel::initialize(
  double alpha1, double alpha2, double alpha3, double alpha4,
  double alpha5)
{
  alpha1_ = alpha1;
  alpha2_ = alpha2;
  alpha3_ = alpha3;
  alpha4_ = alpha4;
  alpha5_ = alpha5;
}

void
AckermannMotionModel::odometryUpdate(
  pf_t * pf, const pf_vector_t & /*pose*/,
  const pf_vector_t & delta)
{
  pf_sample_set_t * set = pf->sets + pf->current_set;

  // 提取 odom 增量
  const double delta_x = delta.v[0];
  const double delta_y = delta.v[1];
  const double delta_theta = delta.v[2];

  const double delta_trans = std::sqrt(delta_x * delta_x + delta_y * delta_y);

  // 近似推测转向角（steering angle），如需更精确可通过 IMU 或转角传感器输入
  // 如果你没有 steering_angle 输入，可不计算这个量
  // double L = 0.35;  // 轴距，可作为参数
  // double steering_angle = (fabs(delta_theta) > 1e-5 && delta_trans > 1e-5) ?
  //                         std::atan(L * delta_theta / delta_trans) : 0.0;

  for (int i = 0; i < set->sample_count; ++i) {
    pf_sample_t * sample = set->samples + i;

    // 对 trans 和旋转角加入噪声
    double noisy_trans = delta_trans + pf_ran_gaussian(std::sqrt(alpha3_ * delta_trans * delta_trans));
    double noisy_theta = delta_theta + pf_ran_gaussian(std::sqrt(alpha1_ * delta_theta * delta_theta));

    double theta = sample->pose.v[2];

    // 阿克曼车近似位移：车辆沿当前方向前进 noisy_trans，方向变化 noisy_theta
    sample->pose.v[0] += noisy_trans * std::cos(theta);
    sample->pose.v[1] += noisy_trans * std::sin(theta);
    sample->pose.v[2] += noisy_theta;
  }
}

}  // namespace nav2_amcl

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(nav2_amcl::AckermannMotionModel, nav2_amcl::MotionModel)
