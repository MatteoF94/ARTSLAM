/** @file robot_measurement.cpp
 * @brief Definition of multiple classes, representing the measurement taken by one or multiple sensors:
 * RobotMeasurement3D, RobotMeasurement2HALFD and RobotMeasurement2D
 * @author Matteo Frosi
*/

#include <robot_measurement.h>

/* Class constructor. */
RobotMeasurement3D::RobotMeasurement3D() {
    this->measurement_vector = Vector10d::Zero();
    this->timestamp = 0;
}

/* Class constructor. */
RobotMeasurement2HALFD::RobotMeasurement2HALFD() {
    this->measurement_vector = Vector7d::Zero();
    this->timestamp = 0;
}

/* Class constructor. */
RobotMeasurement2D::RobotMeasurement2D() {
    this->measurement_vector = Vector5d::Zero();
    this->timestamp = 0;
}