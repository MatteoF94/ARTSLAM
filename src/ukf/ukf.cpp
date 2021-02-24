//
// Created by matteo on 10/02/21.
//

#include <ukf.h>
#include <chrono>
#include <fstream>
#include <GeographicLib/UTMUPS.hpp>

UKF2D::UKF2D() {
    UKFConfig default_config;
    configure_ukf(&default_config);
}

UKF2D::UKF2D(const UKFConfig *const config_ptr) {
    configure_ukf(config_ptr);
}

void UKF2D::configure_ukf(const UKFConfig *const config_ptr) {
    // configure the initial state
    double heading = std::atan2(2*(config_ptr->init_quat_w*config_ptr->init_quat_z + config_ptr->init_quat_x*config_ptr->init_quat_y),
                                1 - 2*(std::pow(config_ptr->init_quat_y,2) + std::pow(config_ptr->init_quat_z,2)));
    initialize_state(config_ptr->init_easting, config_ptr->init_northing,
                     heading, config_ptr->init_forward_vel, config_ptr->init_leftward_vel,
                     config_ptr->init_var_pos, config_ptr->init_var_quat, config_ptr->init_var_vel);

    // configure the process noise matrix
    this->process_noise_matrix = Matrix5d::Identity();
    this->process_noise_matrix.diagonal() << config_ptr->noise_var_pos, config_ptr->noise_var_pos,
                                             config_ptr->noise_var_quat,
                                             config_ptr->noise_var_vel, config_ptr->noise_var_vel;

    // configure the measurement noise matrix
    this->measurement_noise_matrix = Matrix5d::Identity();
    this->measurement_noise_matrix.diagonal() << config_ptr->noise_var_pos_meas, config_ptr->noise_var_pos_meas,
                                                 config_ptr->noise_var_quat_meas,
                                                 config_ptr->noise_var_vel_meas, config_ptr->noise_var_vel_meas;

    // configure the control input covariance matrix
    this->control_covariance_matrix = Matrix3d::Identity();
    this->control_covariance_matrix.diagonal() << config_ptr->control_var_ang_vel,
                                                  config_ptr->control_var_accel, config_ptr->control_var_accel;

    // configure the remaining parameters
    this->is_first_imu = true;
    this->is_first_gnss = true;
    this->is_gnss_available = false;
    this->prev_timestamp = 0;

    this->init_utm = std::make_pair(0.0, 0.0);
    this->pos_at_first_gnss = std::make_pair(0.0, 0.0);

    this->propagated_sigma_points = Eigen::Matrix<double, 5, 17>::Zero();
    this->partial_iterations = config_ptr->num_iterations;
    this->alpha = config_ptr->alpha;
    this->beta = config_ptr->beta;
}

/* Initializes the first state vector and covariance matrix of the UKF. */
void UKF2D::initialize_state(const double easting, const double northing, const double heading, const double vel_x,
                             const double vel_y, const double var_pos, const double var_heading,
                             const double var_vel) {
    this->curr_state.state_vector(0) = easting;
    this->curr_state.state_vector(1) = northing;
    this->curr_state.state_vector(2) = heading;
    this->curr_state.state_vector(3) = vel_x;
    this->curr_state.state_vector(4) = vel_y;
    this->curr_state.covariance_matrix.diagonal() << var_pos, var_pos, var_heading, var_vel, var_vel;
}

/* ----------------------------------------------------- */
/* Registers an object waiting for predicted odometries. */
/* ----------------------------------------------------- */
void UKF2D::register_prior_odom_observer(OdomPriorObserver *odom_prior_observer_ptr_tmp) {
    this->prior_odom_observers.emplace_back(odom_prior_observer_ptr_tmp);
}

/* --------------------------------------------------- */
/* Removes an object waiting for predicted odometries. */
/* --------------------------------------------------- */
void UKF2D::remove_prior_odom_observer(OdomPriorObserver *odom_prior_observer_ptr_tmp) {
    auto iterator = std::find(this->prior_odom_observers.begin(), this->prior_odom_observers.end(), odom_prior_observer_ptr_tmp);

    if(iterator != this->prior_odom_observers.end()) {
        this->prior_odom_observers.erase(iterator);
    }
}

/* ------------------------------------------------------ */
/* Notifies all the observers with the computed keyframe. */
/* ------------------------------------------------------ */
void UKF2D::notify_prior_odom_observers(const Eigen::Matrix4f& odom) {
    // TODO not correct, should use also predicted time
    Header header;
    header.timestamp = this->measurement.timestamp;

    for(OdomPriorObserver *observer : this->prior_odom_observers) {
        observer->update(header, odom);
    }
}

void UKF2D::insert_imu(const ImuMSG::ConstPtr& imu_msg) {
    if(this->is_first_imu) {
        this->prev_timestamp = imu_msg->header.timestamp;
        this->is_first_imu = false;
    } else {
        this->curr_control_input.timestamp = imu_msg->header.timestamp;
        this->curr_control_input.gyro_z = imu_msg->angular_velocity(2);
        this->curr_control_input.accel_x = imu_msg->linear_acceleration(0);
        this->curr_control_input.accel_y = imu_msg->linear_acceleration(1);

        estimate_state();

        this->prev_timestamp = imu_msg->header.timestamp;
    }
}

void UKF2D::insert_gnss(const GeoPointStampedMSG::ConstPtr& gnss_msg) {
    std::lock_guard<std::mutex> lock(this->gnss_insertion_mutex);

    int zone;
    bool nortph;
    double easting, northing;
    GeographicLib::UTMUPS::Forward(gnss_msg->lat, gnss_msg->lon, zone, nortph, easting, northing);

    // TODO also check for timestamp around current IMU measurement
    if(this->is_first_gnss) {
        this->init_utm.first = easting;
        this->init_utm.second = northing;

        this->pos_at_first_gnss.first = this->curr_state.state_vector(0);
        this->pos_at_first_gnss.second = this->curr_state.state_vector(1);

        this->is_first_gnss = false;
    } else {
        this->is_gnss_available = true;
        this->measurement.measurement_vector(0) = (easting - this->init_utm.first) + this->pos_at_first_gnss.first;
        this->measurement.measurement_vector(1) = (northing - this->init_utm.second) + this->pos_at_first_gnss.second;
    }
}

/* --------------------------------------------------------------------------------------------------- */
/* Performs the UKF prediction and correction steps with full available control input and measurement. */
/* --------------------------------------------------------------------------------------------------- */
void UKF2D::perform_batch_step(IMUControlInput control_input, RobotMeasurement2D tmp_measurement) {
    if(this->is_first_imu) {
        this->prev_timestamp = control_input.timestamp;
        this->is_first_imu = false;
    } else {
        this->curr_control_input = control_input;
    }

    int zone;
    bool nortph;
    double easting, northing;
    GeographicLib::UTMUPS::Forward(tmp_measurement.measurement_vector(0), tmp_measurement.measurement_vector(1), zone, nortph, easting, northing);
    tmp_measurement.measurement_vector(0) = easting;
    tmp_measurement.measurement_vector(1) = northing;

    if(this->is_first_gnss) {
        this->init_utm.first = tmp_measurement.measurement_vector(0);
        this->init_utm.second = tmp_measurement.measurement_vector(1);
        this->init_orientation = tmp_measurement.measurement_vector(2);

        this->pos_at_first_gnss.first = this->curr_state.state_vector(0);
        this->pos_at_first_gnss.second = this->curr_state.state_vector(1);

        this->pos_east.emplace_back(tmp_measurement.measurement_vector(0) - this->init_utm.first + this->pos_at_first_gnss.first);
        this->pos_north.emplace_back(tmp_measurement.measurement_vector(1) - this->init_utm.second + this->pos_at_first_gnss.second);
        this->rot_angle.emplace_back(tmp_measurement.measurement_vector(2));

        this->is_first_gnss = false;
        return;
    } else {
        this->is_gnss_available = true;
        this->measurement = tmp_measurement;
        this->measurement.measurement_vector(0) = (tmp_measurement.measurement_vector(0) - this->init_utm.first) + this->pos_at_first_gnss.first;
        this->measurement.measurement_vector(1) = (tmp_measurement.measurement_vector(1) - this->init_utm.second) + this->pos_at_first_gnss.second;
    }

    if(!this->is_first_gnss && !this->is_first_imu) {
        estimate_state();
        this->prev_timestamp = control_input.timestamp;
    }
}

/* ------------------------ */
/* Runs the UKF estimation. */
/* ------------------------ */
void UKF2D::estimate_state() {
    // predict the state of the robot with the previous state and current control input
    predict_state();
    Eigen::Matrix4f odom = Eigen::Matrix4f::Identity();
    Eigen::Rotation2D<float> inv_init(this->init_orientation);
    Eigen::Rotation2D<float> curr_or(this->predicted_state.state_vector(2));
    odom.block<2,2>(0,0) = inv_init.toRotationMatrix().inverse() * curr_or.toRotationMatrix();
    Eigen::Vector2f transl = this->predicted_state.state_vector.segment(0,2).cast<float>();
    odom.block<2,1>(0,3) = inv_init.toRotationMatrix().inverse() * transl;
    notify_prior_odom_observers(odom);
    //this->pos_east.emplace_back(this->predicted_state.state_vector(0));
    //this->pos_north.emplace_back(this->predicted_state.state_vector(1));
    //this->pos_east.emplace_back(this->curr_state.state_vector(0));
    //this->pos_north.emplace_back(this->curr_state.state_vector(1));
    this->pos_east.emplace_back(this->measurement.measurement_vector(0));
    this->pos_north.emplace_back(this->measurement.measurement_vector(1));
    this->rot_angle.emplace_back(this->measurement.measurement_vector(2));

    // correct the state of the robot with the current sensor measurements, if available
    std::lock_guard<std::mutex> lock(this->gnss_insertion_mutex);
    if(this->is_gnss_available) {
        correct_state();
        this->is_gnss_available = false;
    } else {
        this->curr_state = this->predicted_state;
    }
}

/* -----------------------------------------*/
/* Computes the prediction step of the UKF. */
/* ---------------------------------------- */
void UKF2D::predict_state() {
    // create the augmented state
    Vector8d augmented_state = Vector8d::Zero();
    augmented_state.segment(0,5) = this->curr_state.state_vector;
    augmented_state(5) = 0;     // measurement error mean for angular velocity
    augmented_state(6) = 0;     // measurement error mean for forward acceleration (x axis)
    augmented_state(7) = 0;     // measurement error mean for leftward acceleration (y axis)

    // create the augmented covariance matrix
    Matrix8d augmented_covariance = Matrix8d::Identity();
    augmented_covariance.block<5,5>(0,0) = this->curr_state.covariance_matrix;
    augmented_covariance.block<3,3>(5,5) = this->control_covariance_matrix;

    // cholesky decomposition for factor L of the augmented state
    Matrix8d L = augmented_covariance.llt().matrixL();

    // calculate prediction time step
    // TODO not use measurement for time but control input
    std::chrono::nanoseconds dt_ns(this->measurement.timestamp - this->prev_timestamp);
    std::chrono::seconds dt = std::chrono::duration_cast<std::chrono::seconds>(dt_ns);
    double delta_t = dt_ns.count() / 1e9;

    // propagate the augmented sigma points
    Vector8d tmp_sigma_mean = augmented_state;
    this->propagated_sigma_points.col(0) = apply_motion_model(tmp_sigma_mean, delta_t);
    for(int i = 1; i <= 8; i++) {
        Vector8d tmp_sigma_plus = augmented_state + std::sqrt(3.0)*L.col(i-1);
        this->propagated_sigma_points.col(i) = apply_motion_model(tmp_sigma_plus, delta_t);
        Vector8d tmp_sigma_minus = augmented_state - std::sqrt(3.0)*L.col(i-1);
        this->propagated_sigma_points.col(i+8) = apply_motion_model(tmp_sigma_minus, delta_t);
    }

    // compute the predicted state using the augmented sigma points
    this->predicted_state.state_vector = (-5.0/3.0) * this->propagated_sigma_points.col(0);
    for(int i = 1; i < 17; i++) {
        this->predicted_state.state_vector += (1.0/6.0) * this->propagated_sigma_points.col(i);
    }

    // compute the predicted covariance using the augmented sigma points
    Eigen::Matrix<double, 5, 17> delta_state = this->propagated_sigma_points.colwise() - this->predicted_state.state_vector;
    this->predicted_state.covariance_matrix = (-5.0/3.0 + (1-std::pow(this->alpha,2)+this->beta)) * (delta_state.col(0)) * (delta_state.col(0).transpose());
    for(int i = 1; i < 17; i++) {
        this->predicted_state.covariance_matrix += (1.0/6.0) * (delta_state.col(i)) * (delta_state.col(i).transpose());
    }
    this->predicted_state.covariance_matrix += this->process_noise_matrix;
}

/* --------------------------------------------------------------------------- */
/* Propagates the sigma points with a motion model derived from 2D kinematics. */
/* --------------------------------------------------------------------------- */
Vector5d UKF2D::apply_motion_model(Vector8d &state_vector, double delta_t) {
    Vector5d propagated_state_vector;
    double step_t = delta_t / this->partial_iterations;

    for(int i = 0; i < this->partial_iterations; i++) {
        // smaller time steps are more precise, since they take into account the acceleration
        apply_partial_motion_model(state_vector, step_t);
    }

    propagated_state_vector = state_vector.segment(0,5);
    return propagated_state_vector;
}

/* ------------------------------------------------------------------------------------------------------------ */
/* Propagates the sigma points with a partial motion model, using a time step smaller than the transition time. */
/* ------------------------------------------------------------------------------------------------------------ */
void UKF2D::apply_partial_motion_model(Vector8d &state_vector, double delta_t) {
    // apply the control noise to the input
    double omega = this->curr_control_input.gyro_z + state_vector(5);
    double accel_x = this->curr_control_input.accel_x + state_vector(6);
    double accel_y = this->curr_control_input.accel_y + state_vector(7);

    // compute the true velocity and its angle w.r.t. EAST
    double true_v = std::sqrt(std::pow(state_vector(3),2) + std::pow(state_vector(4),2));
    double delta_heading = std::atan(state_vector(4) / state_vector(3)); // values between 0 and PI/2 radians
    double heading = state_vector(2) + delta_heading;

    // kinematics of the movement
    state_vector(0) = state_vector(0) + (true_v/omega) * (std::sin(omega*delta_t + heading) - std::sin(heading));
    state_vector(1) = state_vector(1) + (true_v/omega) * (-std::cos(omega*delta_t + heading) + std::cos(heading));
    state_vector(2) = std::fmod((state_vector(2) + omega*delta_t + M_PI), (2.0 * M_PI)) - M_PI;
    state_vector(3) = state_vector(3) + accel_x*delta_t;
    state_vector(4) = state_vector(4) + accel_y*delta_t;
}

/* -----------------------------------------*/
/* Computes the correction step of the UKF. */
/* ---------------------------------------- */
void UKF2D::correct_state() {
    // compute the predicted measurement points
    Eigen::Matrix<double, 5, 11> measurement_sigma_points;
    Matrix5d L = this->predicted_state.covariance_matrix.llt().matrixL();
    measurement_sigma_points.col(0) = this->predicted_state.state_vector;
    for(int i = 1; i <=5; i++) {
        measurement_sigma_points.col(i) = this->predicted_state.state_vector + std::sqrt(3.0)*L.col(i-1);
        measurement_sigma_points.col(i+5) = this->predicted_state.state_vector - std::sqrt(3.0)*L.col(i-1);
    }
    Vector5d predicted_measurement = (-2.0/3.0) * measurement_sigma_points.col(0);
    for(int i = 1; i < 11; i++) {
        predicted_measurement += (1.0/6.0) * measurement_sigma_points.col(i);
    }

    // compute the innovation covariance
    Eigen::Matrix<double, 5, 11> delta_measurement_matrix = measurement_sigma_points.colwise() - predicted_measurement;
    Matrix5d innovation_matrix = (-2.0/3.0 + (1-std::pow(this->alpha,2)+this->beta)) * (delta_measurement_matrix.col(0)) * (delta_measurement_matrix.col(0).transpose());
    for(int i = 1; i < 11; i++) {
        innovation_matrix += (1.0/6.0) * (delta_measurement_matrix.col(i)) * (delta_measurement_matrix.col(i).transpose());
    }
    innovation_matrix += this->measurement_noise_matrix;

    // compute the cross covariance
    Eigen::Matrix<double, 5, 11> predicted_meas_sigma_points = measurement_sigma_points;
    Eigen::Matrix<double, 5, 11> delta_state = predicted_meas_sigma_points.colwise() - this->predicted_state.state_vector;
    Matrix5d cross_matrix = (-2.0/3.0 + (1-std::pow(this->alpha,2)+this->beta)) * (delta_state.col(0)) * (delta_measurement_matrix.col(0).transpose());
    for(int i = 1; i < 11; i++) {
        cross_matrix += (1.0/6.0) * (delta_state.col(i)) * (delta_measurement_matrix.col(i).transpose());
    }

    // compute the Kalman gain
    Matrix5d kalman_gain = cross_matrix*innovation_matrix.inverse();

    // compute the a posteriori state estimate
    this->curr_state.state_vector = this->predicted_state.state_vector + kalman_gain*(this->measurement.measurement_vector - predicted_measurement);

    // compute the a posteriori covariance estimate
    this->curr_state.covariance_matrix = this->predicted_state.covariance_matrix - kalman_gain*innovation_matrix*(kalman_gain.transpose());
}

void UKF2D::save_trajectory(std::string &filename) {
    std::ofstream output_file(filename);

    for(int i = 0; i < this->pos_east.size(); i++) {
        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
        Eigen::AngleAxisd yaw_angle(-this->rot_angle[0] + M_PI/2, Eigen::Vector3d::UnitZ());
        mat(0,3) = this->pos_east[i];
        mat(1, 3) = this->pos_north[i];
        Eigen::AngleAxisd roto(this->rot_angle[i] - this->rot_angle[0], Eigen::Vector3d::UnitZ());
        mat.block<3,3>(0,0) = roto.toRotationMatrix();
        mat.block<3,1>(0,3) = yaw_angle.toRotationMatrix() * mat.block<3,1>(0,3);
        mat(2,3) = mat(1,3);
        //mat(0, 3) = -mat(1,3);

        Eigen::Matrix3d rot_mat = Eigen::Matrix3d::Identity();
        rot_mat(0,1) = -1.0;
        rot_mat(1,2) = -1.0;
        rot_mat(2,0) = 1.0;
        mat.block<3,3>(0,0) = rot_mat * mat.block<3,3>(0,0);

        output_file << mat(0,0) << " " << mat(0,1) << " " << mat(0,2) << " " << mat(0,3) << " "
                    << mat(1,0) << " " << mat(1,1) << " " << mat(1,2) << " " << mat(1,3) << " "
                    << mat(2,0) << " " << mat(2,1) << " " << mat(2,2) << " " << mat(2,3) << "\n";
    }
}