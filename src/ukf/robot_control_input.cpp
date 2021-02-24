/** @file robot_control_input.cpp
 * @brief Definition of class IMUControlInput
 * @author Matteo Frosi
*/

#include <robot_control_input.h>

/* Class constructor. */
IMUControlInput::IMUControlInput() {
    this->gyro_x = 0.0;
    this->gyro_y = 0.0;
    this->gyro_z = 0.0;
    this->accel_x = 0.0;
    this->accel_y = 0.0;
    this->accel_z = 0.0;
    this->timestamp = 0;
}