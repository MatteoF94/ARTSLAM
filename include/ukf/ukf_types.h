//
// Created by matteo on 10/02/21.
//

#ifndef ARTSLAM_UKF_TYPES_H
#define ARTSLAM_UKF_TYPES_H


#include <Eigen/Dense>

typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 12, 12> Matrix12d;

typedef Eigen::Matrix<double, 10, 1> Vector10d;
typedef Eigen::Matrix<double, 10, 10> Matrix10d;

typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 9, 9> Matrix9d;

typedef Eigen::Matrix<double, 8, 1> Vector8d;
typedef Eigen::Matrix<double, 8, 8> Matrix8d;

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 7, 7> Matrix7d;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

typedef Eigen::Matrix<double, 5, 1> Vector5d;
typedef Eigen::Matrix<double, 5, 5> Matrix5d;

typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;

#endif //ARTSLAM_UKF_TYPES_H
