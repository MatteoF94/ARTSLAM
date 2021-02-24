//
// Created by matteo on 10/02/21.
//

#ifndef ARTSLAM_UKF_H
#define ARTSLAM_UKF_H


#include <ukf_types.h>
#include <robot_state.h>
#include <robot_measurement.h>
#include <robot_control_input.h>
#include <ukf_config.h>
#include <dispatch_queue.h>
#include <slam_types.h>
#include <odom_prior_observer.h>

/**
 * @class UKF2D
 * @brief This class implements a semi-augmented Unscented Kalman Filter (UKF).
 */
class UKF2D {
public:
    /**
     * @brief Class constructor.
     */
    UKF2D();

    /**
     * @brief Class constructor, with parameters.
     * @param config_ptr Pointer to the configuration object for this class.
     */
    explicit UKF2D(const UKFConfig *config_ptr);

    /**
     * @brief Initializes the first state vector of the UKF.
     * @param easting Robot position w.r.t. East (UTM coordinates) [m].
     * @param northing Robot position w.r.t. North (UTM coordinates) [m].
     * @param heading Robot heading angle w.r.t. East [rad].
     * @param vel_x Robot forward speed [m/s].
     * @param vel_y Robot leftward speed [m/s].
     */
    void initialize_state(double easting = 0.0, double northing = 0.0,
                          double heading = 0.0,
                          double vel_x = 0.0, double vel_y = 0.0,
                          double var_pos = 1e-4, double var_heading = 1e-4, double var_vel = 1e-4);

    void insert_imu(const ImuMSG::ConstPtr& imu_msg);

    void insert_gnss(const GeoPointStampedMSG::ConstPtr& gnss_msg);

    /**
     * @brief Performs the UKF prediction and correction steps with full available control input and measurement.
     * @param control_input The control input used in the prediction step of the UKF.
     * @param tmp_measurement The measurement used in the correction step of the UKF.
     */
    void perform_batch_step(IMUControlInput control_input, RobotMeasurement2D tmp_measurement);

    /**
     * @brief Runs the UKF estimation.
     */
    void estimate_state();

    /**
     * @brief Registers an object waiting for predicted odometries.
     * @param odom_prior_observer_ptr_tmp Pointer to the observer object.
     */
    void register_prior_odom_observer(OdomPriorObserver *odom_prior_observer_ptr_tmp);

    /**
     * @brief Removes an object waiting for predicted odometries.
     * @param odom_prior_observer_ptr_tmp Pointer to the observer object.
     */
    void remove_prior_odom_observer(OdomPriorObserver *odom_prior_observer_ptr_tmp);

    /**
     * @brief Notifies all the observers with the predicted odometries.
     * @param header Contains general information.
     * @param odom The predicted odometry to send to the observers.
     */
    void notify_prior_odom_observers(const Eigen::Matrix4f& odom);

    void save_trajectory(std::string& filename);

private:
    // ---------------------------------------------------------------
    // -------------------- CONFIGURATION METHODS --------------------
    //----------------------------------------------------------------
    /**
     * @brief Configures the UKF object using a configuration object.
     * @param config_ptr Pointer to the configuration object.
     */
    void configure_ukf(const UKFConfig *config_ptr);

    // ---------------------------------------------------------------
    // ------------------------- UKF METHODS -------------------------
    //----------------------------------------------------------------
    /**
     * @brief Computes the prediction step of the UKF.
     */
    void predict_state();

    /**
     * @brief Computes the correction step of the UKF.
     */
    void correct_state();

    /**
     * @brief Propagates the sigma points with a motion model derived from 2D kinematics.
     * @param state_vector The state vector to propagate.
     * @param delta_t The interval of time used to propagate the state vector.
     * @return
     */
    Vector5d apply_motion_model(Vector8d& state_vector, double delta_t);

    /**
     * @brief Propagates the sigma points with a partial motion model, using a time step smaller than the transition time.
     * @param state_vector The state vector to propagate.
     * @param delta_t The time step used to propagate the state vector.
     */
    void apply_partial_motion_model(Vector8d& state_vector, double delta_t);

    // ---------------------------------------------------------------
    // ------------------- PARAMETERS AND VARIABLES ------------------
    //----------------------------------------------------------------
    RobotState2D curr_state;                /**< Current state of the UKF */
    RobotState2D predicted_state;           /**< Predicted state of the UKF */
    IMUControlInput curr_control_input;     /**< Control inputs given by the IMU */
    uint64_t prev_timestamp;                /**< Timestamp (in nanoseconds) of the previous control input (IMU) */
    RobotMeasurement2D measurement;         /**< Sensor measurements, used in the correction step of the UKF */

    bool is_first_imu;
    bool is_first_gnss;
    bool is_gnss_available;
    double init_orientation;                        /**< Initial orientation of the robot */
    std::pair<double, double> init_utm;             /**< Initial UTM reading (easting and northing) */
    std::pair<double, double> pos_at_first_gnss;    /**< Vehicle position when the first GNSS measurements is received */

    Eigen::Matrix<double, 5, 17> propagated_sigma_points;
    Matrix5d process_noise_matrix;
    Matrix5d measurement_noise_matrix;
    Matrix3d control_covariance_matrix;

    double alpha;
    double beta;
    uint8_t k;

    uint16_t partial_iterations;    /**< Number of partial iterations of the motion model */

    std::vector<double> pos_east;
    std::vector<double> pos_north;
    std::vector<double> rot_angle;

    DispatchQueue* gnss_insertion_dqueue;
    std::mutex gnss_insertion_mutex;
    DispatchQueue* imu_insertion_dqueue;

    std::vector<OdomPriorObserver*> prior_odom_observers;    /**< Vector of objects waiting for predicted odometries */
};


#endif //ARTSLAM_UKF_H
