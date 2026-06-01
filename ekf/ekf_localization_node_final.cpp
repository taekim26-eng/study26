#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gkusv_driver_interface/msg/usv_compass.hpp>
#include <gkusv_driver_interface/msg/usv_gps.hpp>

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>

namespace utm_conv {

struct UTMCoord {
    double easting;
    double northing;
    int zone;
    char band;
};

UTMCoord latlon_to_utm(double lat_deg, double lon_deg)
{
    constexpr double a = 6378137.0;
    constexpr double f = 1.0 / 298.257223563;
    constexpr double e2 = 2.0 * f - f * f;
    constexpr double e_prime2 = e2 / (1.0 - e2);
    constexpr double k0 = 0.9996;
    constexpr double pi = 3.14159265358979323846;

    const double lat = lat_deg * pi / 180.0;
    const double lon = lon_deg * pi / 180.0;

    const int zone = static_cast<int>((lon_deg + 180.0) / 6.0) + 1;
    const double lon0 = ((zone - 1) * 6.0 - 180.0 + 3.0) * pi / 180.0;

    const double sin_lat = std::sin(lat);
    const double cos_lat = std::cos(lat);
    const double tan_lat = std::tan(lat);
    const double N = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
    const double T = tan_lat * tan_lat;
    const double C = e_prime2 * cos_lat * cos_lat;
    const double A = cos_lat * (lon - lon0);

    const double M = a * ((1.0 - e2 / 4.0 - 3.0 * e2 * e2 / 64.0 - 5.0 * e2 * e2 * e2 / 256.0) * lat
        - (3.0 * e2 / 8.0 + 3.0 * e2 * e2 / 32.0 + 45.0 * e2 * e2 * e2 / 1024.0) * std::sin(2.0 * lat)
        + (15.0 * e2 * e2 / 256.0 + 45.0 * e2 * e2 * e2 / 1024.0) * std::sin(4.0 * lat)
        - (35.0 * e2 * e2 * e2 / 3072.0) * std::sin(6.0 * lat));

    const double easting = k0 * N * (A + (1.0 - T + C) * A * A * A / 6.0
        + (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * e_prime2) * A * A * A * A * A / 120.0)
        + 500000.0;

    double northing = k0 * (M + N * tan_lat * (A * A / 2.0
        + (5.0 - T + 9.0 * C + 4.0 * C * C) * A * A * A * A / 24.0
        + (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * e_prime2) * A * A * A * A * A * A / 720.0));

    if (lat_deg < 0.0) {
        northing += 10000000.0;
    }

    return {easting, northing, zone, lat_deg >= 0.0 ? 'N' : 'S'};
}

}  // namespace utm_conv

class EkfLocalizationNode : public rclcpp::Node
{
public:
    EkfLocalizationNode()
    : Node("node")
    {
        origin_lat_ = this->declare_parameter<double>("origin_lat", 36.690691050000005);
        origin_lon_ = this->declare_parameter<double>("origin_lon", 128.80968808076932);

        process_accel_std_ = this->declare_parameter<double>("process_accel_std", 0.8);
        process_yaw_rate_std_ = this->declare_parameter<double>("process_yaw_rate_std", 0.5);

        gps_position_std_floor_ = this->declare_parameter<double>("gps_position_std_floor", 0.05);
        gps_velocity_std_rtk_ = this->declare_parameter<double>("gps_velocity_std_rtk", 0.10);
        gps_velocity_std_non_rtk_ = this->declare_parameter<double>("gps_velocity_std_non_rtk", 0.30);
        compass_yaw_std_ = this->declare_parameter<double>("compass_yaw_std", 0.22);

        gps_position_std_rtk_fixed_ = this->declare_parameter<double>("gps_position_std_rtk_fixed", gps_position_std_floor_);
        gps_position_std_rtk_float_ = this->declare_parameter<double>("gps_position_std_rtk_float", 0.20);
        gps_position_std_dgps_ = this->declare_parameter<double>("gps_position_std_dgps", 0.50);
        gps_position_std_single_ = this->declare_parameter<double>("gps_position_std_single", 1.50);

        use_sensor_stamp_ = this->declare_parameter<bool>("use_sensor_stamp", true);
        measurement_buffer_delay_sec_ = this->declare_parameter<double>("measurement_buffer_delay_sec", 0.15);
        out_of_sequence_tolerance_sec_ = this->declare_parameter<double>("out_of_sequence_tolerance_sec", 0.02);
        future_stamp_tolerance_sec_ = this->declare_parameter<double>("future_stamp_tolerance_sec", 0.10);
        max_sensor_stamp_age_sec_ = this->declare_parameter<double>("max_sensor_stamp_age_sec", 5.0);
        gps_stamp_offset_sec_ = this->declare_parameter<double>("gps_stamp_offset_sec", 0.0);
        compass_stamp_offset_sec_ = this->declare_parameter<double>("compass_stamp_offset_sec", 0.0);

        compass_yaw_offset_deg_ = this->declare_parameter<double>("compass_yaw_offset_deg", 0.0);
        gps_position_gate_sigma_ = this->declare_parameter<double>("gps_position_gate_sigma", 5.0);
        gps_velocity_gate_sigma_ = this->declare_parameter<double>("gps_velocity_gate_sigma", 5.0);
        compass_gate_sigma_ = this->declare_parameter<double>("compass_gate_sigma", 5.0);

        R_gps_position_floor_ = square(gps_position_std_floor_);
        R_gps_pos_rtk_fixed_ = square(gps_position_std_rtk_fixed_);
        R_gps_pos_rtk_float_ = square(gps_position_std_rtk_float_);
        R_gps_pos_dgps_ = square(gps_position_std_dgps_);
        R_gps_pos_single_ = square(gps_position_std_single_);
        R_gps_vel_rtk_ = square(gps_velocity_std_rtk_);
        R_gps_vel_non_rtk_ = square(gps_velocity_std_non_rtk_);
        R_compass_ = square(compass_yaw_std_);

        const auto origin = utm_conv::latlon_to_utm(origin_lat_, origin_lon_);
        origin_easting_ = origin.easting;
        origin_northing_ = origin.northing;
        utm_zone_ = origin.zone;

        x_ = Eigen::VectorXd::Zero(STATE_DIM);
        P_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
        P_(IDX_X, IDX_X) = 10.0;
        P_(IDX_Y, IDX_Y) = 10.0;
        P_(IDX_YAW, IDX_YAW) = square(PI);
        P_(IDX_VX, IDX_VX) = 1.0;
        P_(IDX_VY, IDX_VY) = 1.0;

        H_gps_pos_ = Eigen::MatrixXd::Zero(2, STATE_DIM);
        H_gps_pos_(0, IDX_X) = 1.0;
        H_gps_pos_(1, IDX_Y) = 1.0;

        H_gps_vel_ = Eigen::MatrixXd::Zero(2, STATE_DIM);
        H_gps_vel_(0, IDX_VX) = 1.0;  // velocity_east -> vx in ENU/map frame
        H_gps_vel_(1, IDX_VY) = 1.0;  // velocity_north -> vy in ENU/map frame

        H_compass_ = Eigen::MatrixXd::Zero(1, STATE_DIM);
        H_compass_(0, IDX_YAW) = 1.0;

        I_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);

        gps_sub_ = this->create_subscription<gkusv_driver_interface::msg::UsvGps>(
            "/gps_data", 10, std::bind(&EkfLocalizationNode::gpsCallback, this, std::placeholders::_1));

        compass_sub_ = this->create_subscription<gkusv_driver_interface::msg::UsvCompass>(
            "/compass_data", 10, std::bind(&EkfLocalizationNode::compassCallback, this, std::placeholders::_1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ekf/pose", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/ekf/odom", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&EkfLocalizationNode::timerCallback, this));

        RCLCPP_INFO(
            this->get_logger(),
            "EKF node started. origin zone=%d E=%.2f N=%.2f, gps_pos_floor=%.3f m, buffer_delay=%.3f s",
            utm_zone_, origin_easting_, origin_northing_, gps_position_std_floor_, measurement_buffer_delay_sec_);
    }

private:
    enum class MeasurementType {
        GPS,
        Compass
    };

    struct Measurement {
        MeasurementType type = MeasurementType::GPS;
        rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
        double gps_x = 0.0;
        double gps_y = 0.0;
        double gps_var = 0.0;
        double velocity_east = 0.0;
        double velocity_north = 0.0;
        bool has_velocity = false;
        int fix_quality = 0;
        double yaw = 0.0;
    };

    static constexpr int STATE_DIM = 5;
    static constexpr int IDX_X = 0;
    static constexpr int IDX_Y = 1;
    static constexpr int IDX_YAW = 2;
    static constexpr int IDX_VX = 3;
    static constexpr int IDX_VY = 4;

    static constexpr double PI = 3.14159265358979323846;
    static constexpr double LAT_MIN = -90.0;
    static constexpr double LAT_MAX = 90.0;
    static constexpr double LON_MIN = -180.0;
    static constexpr double LON_MAX = 180.0;
    static constexpr double MAX_PREDICT_DT = 1.0;

    static double square(double value)
    {
        return value * value;
    }

    static rclcpp::Duration durationFromSeconds(double seconds)
    {
        const auto nanoseconds = static_cast<int64_t>(seconds * 1.0e9);
        return rclcpp::Duration::from_nanoseconds(nanoseconds);
    }

    static double normalizeAngle(double angle)
    {
        if (!std::isfinite(angle)) {
            return 0.0;
        }

        angle = std::fmod(angle + PI, 2.0 * PI);
        if (angle < 0.0) {
            angle += 2.0 * PI;
        }
        return angle - PI;
    }

    rclcpp::Time measurementStamp(
        const builtin_interfaces::msg::Time & header_stamp,
        const rclcpp::Time & arrival_stamp,
        double offset_sec,
        const char * sensor_name)
    {
        if (!use_sensor_stamp_) {
            return arrival_stamp;
        }

        rclcpp::Time stamp(header_stamp, this->get_clock()->get_clock_type());
        if (stamp.nanoseconds() <= 0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "%s header.stamp is zero. Falling back to arrival time.", sensor_name);
            return arrival_stamp;
        }

        if (offset_sec != 0.0) {
            stamp = stamp + durationFromSeconds(offset_sec);
        }

        const double age = (arrival_stamp - stamp).seconds();
        if (age < -future_stamp_tolerance_sec_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "%s stamp is %.3f s in the future. Falling back to arrival time.",
                sensor_name, -age);
            return arrival_stamp;
        }
        if (max_sensor_stamp_age_sec_ > 0.0 && age > max_sensor_stamp_age_sec_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "%s stamp is %.3f s old. Falling back to arrival time.",
                sensor_name, age);
            return arrival_stamp;
        }

        return stamp;
    }

    double headingToYaw(double heading_deg) const
    {
        return normalizeAngle((90.0 - heading_deg + compass_yaw_offset_deg_) * PI / 180.0);
    }

    void enqueueMeasurement(const Measurement & measurement)
    {
        measurement_queue_.push_back(measurement);
        if (measurement_queue_.size() > MAX_QUEUE_SIZE) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Measurement queue overflow. Dropping oldest measurement.");
            measurement_queue_.pop_front();
        }
    }

    void tryInitialize()
    {
        if (initialized_ || !gps_received_ || !compass_received_) {
            return;
        }

        x_.setZero();
        x_(IDX_X) = pending_gps_x_;
        x_(IDX_Y) = pending_gps_y_;
        x_(IDX_YAW) = normalizeAngle(pending_yaw_);

        P_ = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
        const double init_gps_var = pending_gps_var_ > 0.0 ? pending_gps_var_ : R_gps_pos_single_;
        P_(IDX_X, IDX_X) = 4.0 * init_gps_var;
        P_(IDX_Y, IDX_Y) = 4.0 * init_gps_var;
        P_(IDX_YAW, IDX_YAW) = 4.0 * R_compass_;
        P_(IDX_VX, IDX_VX) = 1.0;
        P_(IDX_VY, IDX_VY) = 1.0;

        initialized_ = true;
        last_predict_time_ = pending_gps_stamp_.nanoseconds() >= pending_compass_stamp_.nanoseconds()
            ? pending_gps_stamp_
            : pending_compass_stamp_;

        RCLCPP_INFO(
            this->get_logger(), "EKF initialized: x=%.3f y=%.3f yaw=%.2f deg stamp=%.3f",
            x_(IDX_X), x_(IDX_Y), x_(IDX_YAW) * 180.0 / PI,
            static_cast<double>(last_predict_time_.nanoseconds()) * 1.0e-9);
    }

    void predictTo(const rclcpp::Time & stamp)
    {
        if (!initialized_) {
            return;
        }

        double dt = (stamp - last_predict_time_).seconds();
        if (dt <= 0.0) {
            return;
        }

        predictState(x_, P_, dt);
        last_predict_time_ = stamp;
    }

    void predictState(Eigen::VectorXd & state, Eigen::MatrixXd & covariance, double dt) const
    {
        while (dt > 0.0) {
            const double step = std::min(dt, MAX_PREDICT_DT);
            predictStep(state, covariance, step);
            dt -= step;
        }
    }

    void predictStep(Eigen::VectorXd & state, Eigen::MatrixXd & covariance, double dt) const
    {
        state(IDX_X) += state(IDX_VX) * dt;
        state(IDX_Y) += state(IDX_VY) * dt;
        state(IDX_YAW) = normalizeAngle(state(IDX_YAW));

        Eigen::MatrixXd F = I_;
        F(IDX_X, IDX_VX) = dt;
        F(IDX_Y, IDX_VY) = dt;

        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
        const double q_acc = square(process_accel_std_);
        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;

        Q(IDX_X, IDX_X) = q_acc * dt3 / 3.0;
        Q(IDX_X, IDX_VX) = q_acc * dt2 / 2.0;
        Q(IDX_VX, IDX_X) = q_acc * dt2 / 2.0;
        Q(IDX_VX, IDX_VX) = q_acc * dt;

        Q(IDX_Y, IDX_Y) = q_acc * dt3 / 3.0;
        Q(IDX_Y, IDX_VY) = q_acc * dt2 / 2.0;
        Q(IDX_VY, IDX_Y) = q_acc * dt2 / 2.0;
        Q(IDX_VY, IDX_VY) = q_acc * dt;

        Q(IDX_YAW, IDX_YAW) = square(process_yaw_rate_std_) * dt;

        covariance = F * covariance * F.transpose() + Q;
        covariance = 0.5 * (covariance + covariance.transpose());
    }

    template<typename MatR>
    void josephUpdate(const Eigen::MatrixXd & H, const Eigen::MatrixXd & K, const MatR & R)
    {
        const Eigen::MatrixXd IKH = I_ - K * H;
        P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
        P_ = 0.5 * (P_ + P_.transpose());
    }

    double gpsPositionVarianceFloor(int fix_quality) const
    {
        if (fix_quality == 4) {
            return std::max(R_gps_pos_rtk_fixed_, R_gps_position_floor_);
        }
        if (fix_quality == 5) {
            return std::max(R_gps_pos_rtk_float_, R_gps_position_floor_);
        }
        if (fix_quality == 2) {
            return std::max(R_gps_pos_dgps_, R_gps_position_floor_);
        }
        return std::max(R_gps_pos_single_, R_gps_position_floor_);
    }

    double gpsPositionVariance(double accuracy, int fix_quality) const
    {
        const double variance_floor = gpsPositionVarianceFloor(fix_quality);
        if (!std::isfinite(accuracy) || accuracy <= 0.0) {
            return variance_floor;
        }

        return std::max(square(accuracy), variance_floor);
    }

    bool updateGpsPosition(double local_x, double local_y, double gps_var)
    {
        const Eigen::Vector2d z(local_x, local_y);
        const Eigen::Vector2d y_innov = z - H_gps_pos_ * x_;
        const Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * gps_var;
        const Eigen::Matrix2d S = H_gps_pos_ * P_ * H_gps_pos_.transpose() + R;

        if (gps_position_gate_sigma_ > 0.0) {
            const Eigen::Vector2d normalized_innov = S.ldlt().solve(y_innov);
            const double maha2 = y_innov.dot(normalized_innov);
            if (!std::isfinite(maha2) || maha2 > square(gps_position_gate_sigma_)) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Rejecting GPS position outlier. mahalanobis=%.2f threshold=%.2f",
                    std::sqrt(std::max(0.0, maha2)), gps_position_gate_sigma_);
                return false;
            }
        }

        const Eigen::MatrixXd K = P_ * H_gps_pos_.transpose() * S.inverse();

        x_ = x_ + K * y_innov;
        x_(IDX_YAW) = normalizeAngle(x_(IDX_YAW));
        josephUpdate(H_gps_pos_, K, R);
        return true;
    }

    bool updateGpsVelocity(double velocity_east, double velocity_north, int fix_quality)
    {
        const double vel_var = (fix_quality >= 4) ? R_gps_vel_rtk_ : R_gps_vel_non_rtk_;
        const Eigen::Vector2d z(velocity_east, velocity_north);
        const Eigen::Vector2d y_innov = z - H_gps_vel_ * x_;
        const Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * vel_var;
        const Eigen::Matrix2d S = H_gps_vel_ * P_ * H_gps_vel_.transpose() + R;

        if (gps_velocity_gate_sigma_ > 0.0) {
            const Eigen::Vector2d normalized_innov = S.ldlt().solve(y_innov);
            const double maha2 = y_innov.dot(normalized_innov);
            if (!std::isfinite(maha2) || maha2 > square(gps_velocity_gate_sigma_)) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Rejecting GPS velocity outlier. mahalanobis=%.2f threshold=%.2f",
                    std::sqrt(std::max(0.0, maha2)), gps_velocity_gate_sigma_);
                return false;
            }
        }

        const Eigen::MatrixXd K = P_ * H_gps_vel_.transpose() * S.inverse();

        x_ = x_ + K * y_innov;
        x_(IDX_YAW) = normalizeAngle(x_(IDX_YAW));
        josephUpdate(H_gps_vel_, K, R);
        return true;
    }

    bool updateCompass(double yaw_measured)
    {
        const double y_innov = normalizeAngle(yaw_measured - x_(IDX_YAW));
        const double S = P_(IDX_YAW, IDX_YAW) + R_compass_;

        if (compass_gate_sigma_ > 0.0) {
            const double maha2 = y_innov * y_innov / S;
            if (!std::isfinite(maha2) || maha2 > square(compass_gate_sigma_)) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Rejecting compass yaw outlier. mahalanobis=%.2f threshold=%.2f",
                    std::sqrt(std::max(0.0, maha2)), compass_gate_sigma_);
                return false;
            }
        }

        const Eigen::VectorXd K = P_.col(IDX_YAW) / S;

        x_ = x_ + K * y_innov;
        x_(IDX_YAW) = normalizeAngle(x_(IDX_YAW));

        Eigen::Matrix<double, 1, 1> R;
        R(0, 0) = R_compass_;
        josephUpdate(H_compass_, Eigen::MatrixXd(K), R);
        return true;
    }

    bool prepareMeasurementTime(Measurement & measurement)
    {
        if (!initialized_) {
            return true;
        }

        const double dt = (measurement.stamp - last_predict_time_).seconds();
        if (dt >= 0.0) {
            return true;
        }

        const double late_sec = -dt;
        if (late_sec <= out_of_sequence_tolerance_sec_) {
            measurement.stamp = last_predict_time_;
            return true;
        }

        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Dropping out-of-sequence %s measurement. late=%.3f s",
            measurement.type == MeasurementType::GPS ? "GPS" : "Compass", late_sec);
        return false;
    }

    void processMeasurement(Measurement measurement)
    {
        if (measurement.type == MeasurementType::GPS) {
            pending_gps_x_ = measurement.gps_x;
            pending_gps_y_ = measurement.gps_y;
            pending_gps_var_ = measurement.gps_var;
            pending_gps_stamp_ = measurement.stamp;
            gps_received_ = true;

            if (!initialized_) {
                tryInitialize();
                return;
            }

            if (!prepareMeasurementTime(measurement)) {
                return;
            }

            predictTo(measurement.stamp);
            const bool position_used = updateGpsPosition(measurement.gps_x, measurement.gps_y, measurement.gps_var);
            if (position_used && measurement.has_velocity) {
                updateGpsVelocity(
                    measurement.velocity_east, measurement.velocity_north, measurement.fix_quality);
            }
            return;
        }

        pending_yaw_ = measurement.yaw;
        pending_compass_stamp_ = measurement.stamp;
        compass_received_ = true;

        if (!initialized_) {
            tryInitialize();
            return;
        }

        if (!prepareMeasurementTime(measurement)) {
            return;
        }

        predictTo(measurement.stamp);
        updateCompass(measurement.yaw);
    }

    void processQueuedMeasurements(const rclcpp::Time & now)
    {
        if (measurement_queue_.empty()) {
            return;
        }

        std::stable_sort(
            measurement_queue_.begin(), measurement_queue_.end(),
            [](const Measurement & lhs, const Measurement & rhs) {
                return lhs.stamp.nanoseconds() < rhs.stamp.nanoseconds();
            });

        const int64_t buffer_delay_ns =
            static_cast<int64_t>(std::max(0.0, measurement_buffer_delay_sec_) * 1.0e9);
        if (buffer_delay_ns > 0 && now.nanoseconds() <= buffer_delay_ns) {
            return;
        }

        const rclcpp::Time process_until = now - rclcpp::Duration::from_nanoseconds(buffer_delay_ns);
        while (!measurement_queue_.empty() &&
               measurement_queue_.front().stamp.nanoseconds() <= process_until.nanoseconds()) {
            const Measurement measurement = measurement_queue_.front();
            measurement_queue_.pop_front();
            processMeasurement(measurement);
        }
    }

    void gpsCallback(const gkusv_driver_interface::msg::UsvGps::SharedPtr msg)
    {
        const rclcpp::Time arrival_stamp = this->now();
        const rclcpp::Time stamp = measurementStamp(
            msg->header.stamp, arrival_stamp, gps_stamp_offset_sec_, "GPS");

        if (!std::isfinite(msg->latitude) || !std::isfinite(msg->longitude)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "GPS lat/lon contains NaN/Inf, skipping update");
            return;
        }

        if (msg->latitude < LAT_MIN || msg->latitude > LAT_MAX ||
            msg->longitude < LON_MIN || msg->longitude > LON_MAX) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "GPS lat/lon out of range, skipping update: lat=%.6f lon=%.6f",
                msg->latitude, msg->longitude);
            return;
        }

        if (msg->fix_quality < 1) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "GPS fix_quality=%d, skipping update", static_cast<int>(msg->fix_quality));
            return;
        }

        const auto utm = utm_conv::latlon_to_utm(msg->latitude, msg->longitude);
        const double local_x = utm.easting - origin_easting_;
        const double local_y = utm.northing - origin_northing_;

        if (!std::isfinite(local_x) || !std::isfinite(local_y)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "UTM conversion produced NaN/Inf, skipping update");
            return;
        }

        const int fix_quality = static_cast<int>(msg->fix_quality);
        const double gps_var = gpsPositionVariance(msg->accuracy, fix_quality);

        std::lock_guard<std::mutex> lock(mtx_);

        Measurement measurement;
        measurement.type = MeasurementType::GPS;
        measurement.stamp = stamp;
        measurement.gps_x = local_x;
        measurement.gps_y = local_y;
        measurement.gps_var = gps_var;
        measurement.fix_quality = fix_quality;

        const double velocity_east = msg->velocity_east;
        const double velocity_north = msg->velocity_north;
        if (!std::isfinite(velocity_east) || !std::isfinite(velocity_north)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "GPS velocity contains NaN/Inf. Position update will still be used.");
        } else {
            measurement.velocity_east = velocity_east;
            measurement.velocity_north = velocity_north;
            measurement.has_velocity = true;
        }

        enqueueMeasurement(measurement);
    }

    void compassCallback(const gkusv_driver_interface::msg::UsvCompass::SharedPtr msg)
    {
        const rclcpp::Time arrival_stamp = this->now();
        const rclcpp::Time stamp = measurementStamp(
            msg->header.stamp, arrival_stamp, compass_stamp_offset_sec_, "Compass");
        const double heading_deg = static_cast<double>(msg->heading);
        if (!std::isfinite(heading_deg)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Compass heading contains NaN/Inf, skipping update");
            return;
        }

        const double yaw_measured = headingToYaw(heading_deg);

        std::lock_guard<std::mutex> lock(mtx_);

        Measurement measurement;
        measurement.type = MeasurementType::Compass;
        measurement.stamp = stamp;
        measurement.yaw = yaw_measured;
        enqueueMeasurement(measurement);
    }

    void timerCallback()
    {
        const rclcpp::Time stamp = this->now();
        std::lock_guard<std::mutex> lock(mtx_);

        processQueuedMeasurements(stamp);

        if (!initialized_) {
            return;
        }

        Eigen::VectorXd publish_state = x_;
        Eigen::MatrixXd publish_covariance = P_;
        const double dt_to_now = (stamp - last_predict_time_).seconds();
        if (dt_to_now > 0.0) {
            predictState(publish_state, publish_covariance, dt_to_now);
        }

        publishPose(stamp, publish_state);
        publishOdom(stamp, publish_state, publish_covariance);
    }

    void publishPose(const rclcpp::Time & stamp, const Eigen::VectorXd & state)
    {
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = stamp;
        msg.header.frame_id = "map";

        msg.pose.position.x = state(IDX_X);
        msg.pose.position.y = state(IDX_Y);
        msg.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, state(IDX_YAW));
        msg.pose.orientation = tf2::toMsg(q);

        pose_pub_->publish(msg);
    }

    void publishOdom(
        const rclcpp::Time & stamp,
        const Eigen::VectorXd & state,
        const Eigen::MatrixXd & covariance)
    {
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = stamp;
        msg.header.frame_id = "map";
        msg.child_frame_id = "base_link";

        msg.pose.pose.position.x = state(IDX_X);
        msg.pose.pose.position.y = state(IDX_Y);
        msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, state(IDX_YAW));
        msg.pose.pose.orientation = tf2::toMsg(q);

        msg.pose.covariance.fill(0.0);
        msg.pose.covariance[0] = covariance(IDX_X, IDX_X);
        msg.pose.covariance[1] = covariance(IDX_X, IDX_Y);
        msg.pose.covariance[6] = covariance(IDX_Y, IDX_X);
        msg.pose.covariance[7] = covariance(IDX_Y, IDX_Y);
        msg.pose.covariance[14] = 1e6;
        msg.pose.covariance[21] = 1e6;
        msg.pose.covariance[28] = 1e6;
        msg.pose.covariance[35] = covariance(IDX_YAW, IDX_YAW);

        const double yaw = state(IDX_YAW);
        const double vx_w = state(IDX_VX);
        const double vy_w = state(IDX_VY);
        const double cos_yaw = std::cos(yaw);
        const double sin_yaw = std::sin(yaw);

        msg.twist.twist.linear.x = cos_yaw * vx_w + sin_yaw * vy_w;
        msg.twist.twist.linear.y = -sin_yaw * vx_w + cos_yaw * vy_w;
        msg.twist.twist.angular.z = 0.0;

        Eigen::Matrix2d R_world_to_body;
        R_world_to_body << cos_yaw, sin_yaw,
                          -sin_yaw, cos_yaw;
        Eigen::Matrix2d P_vel_world;
        P_vel_world << covariance(IDX_VX, IDX_VX), covariance(IDX_VX, IDX_VY),
                       covariance(IDX_VY, IDX_VX), covariance(IDX_VY, IDX_VY);
        const Eigen::Matrix2d P_vel_body = R_world_to_body * P_vel_world * R_world_to_body.transpose();

        msg.twist.covariance.fill(0.0);
        msg.twist.covariance[0] = P_vel_body(0, 0);
        msg.twist.covariance[1] = P_vel_body(0, 1);
        msg.twist.covariance[6] = P_vel_body(1, 0);
        msg.twist.covariance[7] = P_vel_body(1, 1);
        msg.twist.covariance[14] = 1e6;
        msg.twist.covariance[21] = 1e6;
        msg.twist.covariance[28] = 1e6;
        msg.twist.covariance[35] = 1e6;

        odom_pub_->publish(msg);
    }

    static constexpr std::size_t MAX_QUEUE_SIZE = 200;

    double origin_lat_ = 0.0;
    double origin_lon_ = 0.0;
    double origin_easting_ = 0.0;
    double origin_northing_ = 0.0;
    int utm_zone_ = 0;

    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd I_;
    Eigen::MatrixXd H_gps_pos_;
    Eigen::MatrixXd H_gps_vel_;
    Eigen::MatrixXd H_compass_;

    double process_accel_std_ = 0.4;
    double process_yaw_rate_std_ = 0.5;
    double gps_position_std_floor_ = 0.30;
    double gps_velocity_std_rtk_ = 0.05;
    double gps_velocity_std_non_rtk_ = 0.30;
    double compass_yaw_std_ = 0.22;
    double gps_position_std_rtk_fixed_ = 0.05;
    double gps_position_std_rtk_float_ = 0.20;
    double gps_position_std_dgps_ = 0.50;
    double gps_position_std_single_ = 1.50;

    bool use_sensor_stamp_ = true;
    double measurement_buffer_delay_sec_ = 0.15;
    double out_of_sequence_tolerance_sec_ = 0.02;
    double future_stamp_tolerance_sec_ = 0.10;
    double max_sensor_stamp_age_sec_ = 5.0;
    double gps_stamp_offset_sec_ = 0.0;
    double compass_stamp_offset_sec_ = 0.0;
    double compass_yaw_offset_deg_ = 0.0;
    double gps_position_gate_sigma_ = 5.0;
    double gps_velocity_gate_sigma_ = 5.0;
    double compass_gate_sigma_ = 5.0;

    double R_gps_position_floor_ = 0.0;
    double R_gps_pos_rtk_fixed_ = 0.0;
    double R_gps_pos_rtk_float_ = 0.0;
    double R_gps_pos_dgps_ = 0.0;
    double R_gps_pos_single_ = 0.0;
    double R_gps_vel_rtk_ = 0.0;
    double R_gps_vel_non_rtk_ = 0.0;
    double R_compass_ = 0.0;

    bool gps_received_ = false;
    bool compass_received_ = false;
    bool initialized_ = false;

    double pending_gps_x_ = 0.0;
    double pending_gps_y_ = 0.0;
    double pending_gps_var_ = 0.0;
    double pending_yaw_ = 0.0;
    rclcpp::Time pending_gps_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time pending_compass_stamp_{0, 0, RCL_ROS_TIME};

    rclcpp::Time last_predict_time_{0, 0, RCL_ROS_TIME};
    std::deque<Measurement> measurement_queue_;
    std::mutex mtx_;

    rclcpp::Subscription<gkusv_driver_interface::msg::UsvGps>::SharedPtr gps_sub_;
    rclcpp::Subscription<gkusv_driver_interface::msg::UsvCompass>::SharedPtr compass_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EkfLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
