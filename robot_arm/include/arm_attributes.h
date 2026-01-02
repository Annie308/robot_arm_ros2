#pragma once

#include <Eigen/Dense>

//Make these parameters!!!!
//----------------------
inline extern const float base = 0.5f;
inline extern const float l1 = 0.5f; 
inline extern const float l2 = 0.5f;
inline extern const float l3 = 0.5f;
inline extern const float l4 = 0.5f;
inline extern const float l5 = 0.5f;
inline extern const float l6 = 0.5f;
//---------------------

inline extern const float PI = 3.14159265358979323846f;

inline extern const Eigen::Vector3d base_link(0, 0,base);
inline extern const Eigen::Vector3d link1(0, 0, l1);
inline extern const Eigen::Vector3d link2(l2, 0, 0);
inline extern const Eigen::Vector3d link3(l3, 0, 0);

inline extern const Eigen::Vector3d link4(l4, 0,0);
inline extern const Eigen::Vector3d link5(0, 0, l5);
inline extern const Eigen::Vector3d link5_connector(l5, 0, 0);
inline extern const Eigen::Vector3d link6(l6, 0, 0);
