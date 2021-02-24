/** @file robot_state.cpp
 * @brief Definition of multiple classes, representing the state of a moving robot: RobotState3D, RobotState2HALFD and
 * RobotState2D
 * @author Matteo Frosi
*/

#include <robot_state.h>

/* Class constructor. */
RobotState3D::RobotState3D() {
    this->state_vector = Vector10d::Zero();
    this->covariance_matrix = Matrix10d::Identity();
}

/* Class constructor. */
RobotState2HALFD::RobotState2HALFD() {
    this->state_vector = Vector7d::Zero();
    this->covariance_matrix = Matrix7d::Identity();
}

/* Class constructor. */
RobotState2D::RobotState2D() {
    this->state_vector = Vector5d::Zero();
    this->covariance_matrix = Matrix5d::Identity();
}