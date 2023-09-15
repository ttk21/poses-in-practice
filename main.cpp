#include "Eigen/Eigen"
#include "sophus/so3.hpp"
#include "sophus/se3.hpp"
#include <iostream>

constexpr double pi = 3.14159265358979323846;

int main()
{
  const auto R_wb = Sophus::SO3d::rotZ(pi) * Sophus::SO3d::rotY(0.1) * Sophus::SO3d::rotX(-0.1);
  const Eigen::Vector3d t_wb(100.0, -50.0, 0.0);
  const Sophus::SE3d T_wb(R_wb, t_wb);

  Eigen::Matrix4d mat4_cb{
    {0, 1, 0, 0},
    {0, 0, 1, 2},
    {1, 0, 0, 0},
    {0, 0, 0, 1}};
  const auto T_cb = Sophus::SE3d::fitToSE3(mat4_cb);
  const auto T_wc = T_wb * T_cb.inverse();

  const Eigen::Vector3d x_c(0.0, 0.0, 100.0);
  const Eigen::Vector3d x_w = T_wc * x_c;

  std::cout << "T_wb = " << std::endl << T_wb.matrix() << std::endl << std::endl;
  std::cout << "mat4_cb = " << std::endl << mat4_cb << std::endl << std::endl;
  std::cout << "T_cb_inv = " << std::endl << T_cb.inverse().matrix() << std::endl << std::endl;
  std::cout << "T_wc = " << std::endl << T_wc.matrix() << std::endl << std::endl;
  std::cout << "x_w = " << std::endl << x_w << std::endl << std::endl;
}
