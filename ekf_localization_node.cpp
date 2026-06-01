/**
 * EKF Localization Node for Autonomous Surface Vehicle (USV) — 3차 수정 버전
 *
 * 입력 토픽:
 *   /gps_data      (gkusv_driver_interface/msg/UsvGps)
 *   /compass_data  (gkusv_driver_interface/msg/UsvCompass)
 *
 * 출력 토픽:
 *   /ekf/pose      (geometry_msgs/msg/PoseStamped)       — x, y, yaw 추정값
 *   /ekf/odom      (nav_msgs/msg/Odometry)               — 전체 오도메트리
 *
 * 좌표계: UTM → 정박지1 원점 기준 로컬 좌표 (ENU: x=East, y=North)
 *   정박지1: lat=36.690691050000005, lon=128.80968808076932
 *
 * 2차 → 3차 변경사항:
 *   [1] GPS latitude/longitude/accuracy 의 NaN/Inf 입력 검사 추가
 *       → 비정상값 한 번이라도 들어오면 EKF 상태 전체가 NaN 으로 오염되는
 *          치명적 상황을 방지
 *   [2] Compass heading 의 NaN/Inf 입력 검사 추가
 *   [3] (1번 피드백)은 driver 가 status=3, accuracy=0.014 로 안정 동작 중임을
 *       실제 데이터로 확인하여 현 시점에서는 반영하지 않음.
 *       driver 동작이 변경되거나 RTK 끊김이 발생하면 재검토 예정.
 *
 * (1차 → 2차 핵심 변경사항 — 참고)
 *   - 속도 상태를 world frame 으로 변경: [x, y, yaw, vx_w, vy_w]
 *   - GPS + Compass 모두 수신된 뒤 초기화
 *   - 프로세스 노이즈를 σ × √dt 형태로 물리 기반 재정의
 *   - Joseph form 공분산 업데이트
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// 커스텀 메시지
#include <gkusv_driver_interface/msg/usv_gps.hpp>
#include <gkusv_driver_interface/msg/usv_compass.hpp>

#include <Eigen/Dense>
#include <cmath>
#include <mutex>

// ─── UTM 변환 (WGS84 → UTM) ───────────────────────────────────────────────
namespace utm_conv {

struct UTMCoord {
    double easting;
    double northing;
    int zone;
    char band;
};

UTMCoord latlon_to_utm(double lat_deg, double lon_deg)
{
    // WGS84 파라미터
    const double a  = 6378137.0;
    const double f  = 1.0 / 298.257223563;
    const double e2 = 2.0 * f - f * f;
    const double e_prime2 = e2 / (1.0 - e2);
    const double k0 = 0.9996;

    double lat = lat_deg * M_PI / 180.0;
    double lon = lon_deg * M_PI / 180.0;

    int zone = static_cast<int>((lon_deg + 180.0) / 6.0) + 1;
    double lon0 = ((zone - 1) * 6.0 - 180.0 + 3.0) * M_PI / 180.0;

    double N = a / std::sqrt(1.0 - e2 * std::sin(lat) * std::sin(lat));
    double T = std::tan(lat) * std::tan(lat);
    double C = e_prime2 * std::cos(lat) * std::cos(lat);
    double A = std::cos(lat) * (lon - lon0);

    // M: 적도에서 위도까지의 자오선 호 길이
    double M = a * ((1.0 - e2/4.0 - 3.0*e2*e2/64.0 - 5.0*e2*e2*e2/256.0) * lat
              - (3.0*e2/8.0 + 3.0*e2*e2/32.0 + 45.0*e2*e2*e2/1024.0) * std::sin(2.0*lat)
              + (15.0*e2*e2/256.0 + 45.0*e2*e2*e2/1024.0) * std::sin(4.0*lat)
              - (35.0*e2*e2*e2/3072.0) * std::sin(6.0*lat));

    double easting = k0 * N * (A + (1.0-T+C)*A*A*A/6.0
                   + (5.0-18.0*T+T*T+72.0*C-58.0*e_prime2)*A*A*A*A*A/120.0)
                   + 500000.0;

    double northing = k0 * (M + N * std::tan(lat) *
                    (A*A/2.0 + (5.0-T+9.0*C+4.0*C*C)*A*A*A*A/24.0
                   + (61.0-58.0*T+T*T+600.0*C-330.0*e_prime2)*A*A*A*A*A*A/720.0));

    if (lat_deg < 0.0)
        northing += 10000000.0;

    char band = (lat_deg >= 0.0) ? 'N' : 'S';

    return {easting, northing, zone, band};
}

} // namespace utm_conv


// ─── EKF 로컬라이제이션 노드 ──────────────────────────────────────────────
class EkfLocalizationNode : public rclcpp::Node
{
public:
    EkfLocalizationNode()
    : Node("ekf_localization_node")
    {
        // 정박지1 원점의 UTM 좌표 계산
        auto origin = utm_conv::latlon_to_utm(ORIGIN_LAT, ORIGIN_LON);
        origin_easting_  = origin.easting;
        origin_northing_ = origin.northing;
        utm_zone_        = origin.zone;
        RCLCPP_INFO(this->get_logger(),
            "Origin (정박지1) UTM: zone=%d, E=%.2f, N=%.2f",
            utm_zone_, origin_easting_, origin_northing_);

        // ─── EKF 상태 초기화 ───
        // 상태 벡터: [x, y, yaw, vx_w, vy_w]  (5×1, world frame 속도)
        x_ = Eigen::VectorXd::Zero(STATE_DIM);

        // 상태 공분산 (초기 불확실성)
        P_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
        P_(IDX_X,   IDX_X)   = 10.0;
        P_(IDX_Y,   IDX_Y)   = 10.0;
        P_(IDX_YAW, IDX_YAW) = (M_PI);
        P_(IDX_VX,  IDX_VX)  = 1.0;
        P_(IDX_VY,  IDX_VY)  = 1.0;

        // ─── 프로세스 노이즈 파라미터 (물리적 근거 기반) ───
        sigma_pos_  = 0.05;      // [m/√s]
        sigma_vel_  = 0.5;       // [m/s²] (가속도 표준편차)
        sigma_yaw_  = 0.5;       // [rad/s] (각속도 표준편차)

        // GPS 측정 노이즈 (RTK Fixed 기본값)
        R_gps_default_ = 0.014 * 0.014;  // RTK Fixed σ=1.4 cm

        // Compass yaw 측정 노이즈
        R_compass_ = 0.05;

        // ─── 측정 행렬 H 캐싱 ───
        H_gps_ = Eigen::MatrixXd::Zero(2, STATE_DIM);
        H_gps_(0, IDX_X) = 1.0;
        H_gps_(1, IDX_Y) = 1.0;

        H_compass_ = Eigen::MatrixXd::Zero(1, STATE_DIM);
        H_compass_(0, IDX_YAW) = 1.0;

        I_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);

        // ─── 구독자 ───
        gps_sub_ = this->create_subscription<gkusv_driver_interface::msg::UsvGps>(
            "/gps_data", 10,
            std::bind(&EkfLocalizationNode::gpsCallback, this, std::placeholders::_1));

        compass_sub_ = this->create_subscription<gkusv_driver_interface::msg::UsvCompass>(
            "/compass_data", 10,
            std::bind(&EkfLocalizationNode::compassCallback, this, std::placeholders::_1));

        // ─── 발행자 ───
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ekf/pose", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/ekf/odom", 10);

        // ─── 주기적 예측 + 발행 (50Hz) ───
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&EkfLocalizationNode::timerCallback, this));

        last_predict_time_ = this->now();

        RCLCPP_INFO(this->get_logger(),
            "EKF Localization Node started. (GPS + Compass, world-frame velocity model)");
    }

private:
    // ─── 상수 ───
    static constexpr int STATE_DIM = 5;
    static constexpr double ORIGIN_LAT = 36.690691050000005;
    static constexpr double ORIGIN_LON = 128.80968808076932;

    // 상태 인덱스
    static constexpr int IDX_X   = 0;
    static constexpr int IDX_Y   = 1;
    static constexpr int IDX_YAW = 2;
    static constexpr int IDX_VX  = 3;
    static constexpr int IDX_VY  = 4;

    // GPS 입력 검증용 상수
    //   위/경도가 정상 범위를 벗어나면 driver 가 비정상값을 보낸 것으로 간주
    static constexpr double LAT_MIN = -90.0;
    static constexpr double LAT_MAX =  90.0;
    static constexpr double LON_MIN = -180.0;
    static constexpr double LON_MAX =  180.0;

    // UTM 원점
    double origin_easting_  = 0.0;
    double origin_northing_ = 0.0;
    int    utm_zone_        = 0;

    // EKF 변수
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd I_;

    // 캐시된 측정 행렬
    Eigen::MatrixXd H_gps_;
    Eigen::MatrixXd H_compass_;

    // 노이즈 파라미터
    double sigma_pos_;
    double sigma_vel_;
    double sigma_yaw_;
    double R_gps_default_;
    double R_compass_;

    // 초기화 플래그
    bool gps_received_     = false;
    bool compass_received_ = false;
    bool initialized_      = false;

    // 최신 GPS, Compass 캐시 (초기화 전 동기화용)
    double pending_gps_x_ = 0.0;
    double pending_gps_y_ = 0.0;
    double pending_yaw_   = 0.0;

    rclcpp::Time last_predict_time_;
    std::mutex mtx_;

    // ROS 인터페이스
    rclcpp::Subscription<gkusv_driver_interface::msg::UsvGps>::SharedPtr     gps_sub_;
    rclcpp::Subscription<gkusv_driver_interface::msg::UsvCompass>::SharedPtr compass_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr          odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ─── 각도 정규화 (−π ~ π) ─────────────────────────────────────────────
    static double normalizeAngle(double angle)
    {
        while (angle >  M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // ─── EKF 초기화 시도 ──────────────────────────────────────────────────
    void tryInitialize()
    {
        if (initialized_) return;
        if (!gps_received_ || !compass_received_) return;

        x_.setZero();
        x_(IDX_X)   = pending_gps_x_;
        x_(IDX_Y)   = pending_gps_y_;
        x_(IDX_YAW) = normalizeAngle(pending_yaw_);

        P_ = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
        P_(IDX_X,   IDX_X)   = R_gps_default_ * 4.0;
        P_(IDX_Y,   IDX_Y)   = R_gps_default_ * 4.0;
        P_(IDX_YAW, IDX_YAW) = R_compass_ * 4.0;
        P_(IDX_VX,  IDX_VX)  = 1.0;
        P_(IDX_VY,  IDX_VY)  = 1.0;

        initialized_ = true;
        last_predict_time_ = this->now();

        RCLCPP_INFO(this->get_logger(),
            "EKF initialized: pos=(%.2f, %.2f), yaw=%.2f deg",
            x_(IDX_X), x_(IDX_Y), x_(IDX_YAW) * 180.0 / M_PI);
    }

    // ─── 예측 단계 ────────────────────────────────────────────────────────
    void predict(double dt)
    {
        if (dt <= 0.0 || dt > 1.0) return;

        // 상태 천이 (world frame 등속)
        x_(IDX_X) += x_(IDX_VX) * dt;
        x_(IDX_Y) += x_(IDX_VY) * dt;

        // 야코비안 F (5×5)
        Eigen::MatrixXd F = I_;
        F(IDX_X, IDX_VX) = dt;
        F(IDX_Y, IDX_VY) = dt;

        // 프로세스 노이즈 Q
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
        Q(IDX_X,   IDX_X)   = sigma_pos_ * sigma_pos_ * dt;
        Q(IDX_Y,   IDX_Y)   = sigma_pos_ * sigma_pos_ * dt;
        Q(IDX_YAW, IDX_YAW) = sigma_yaw_ * sigma_yaw_ * dt;
        Q(IDX_VX,  IDX_VX)  = sigma_vel_ * sigma_vel_ * dt;
        Q(IDX_VY,  IDX_VY)  = sigma_vel_ * sigma_vel_ * dt;

        P_ = F * P_ * F.transpose() + Q;
    }

    // ─── Joseph form 공분산 업데이트 ──────────────────────────────────────
    template<typename MatR>
    void josephUpdate(const Eigen::MatrixXd& H,
                      const Eigen::MatrixXd& K,
                      const MatR&            R)
    {
        Eigen::MatrixXd IKH = I_ - K * H;
        P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
        P_ = 0.5 * (P_ + P_.transpose());
    }

    // ─── GPS 콜백 (측정 업데이트: x, y) ──────────────────────────────────
    void gpsCallback(const gkusv_driver_interface::msg::UsvGps::SharedPtr msg)
    {
        // ─── [3차 추가] 입력 검증: NaN/Inf, 위/경도 범위 ───
        //   비정상값이 한 번이라도 EKF 에 들어가면 P, x_ 가 영구적으로
        //   NaN 으로 오염되어 노드 재시작 외에는 복구 불가. 사전 차단 필수.
        if (!std::isfinite(msg->latitude) || !std::isfinite(msg->longitude)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "GPS lat/lon contains NaN/Inf, skipping update "
                "(lat=%.6f, lon=%.6f)", msg->latitude, msg->longitude);
            return;
        }
        if (msg->latitude  < LAT_MIN || msg->latitude  > LAT_MAX ||
            msg->longitude < LON_MIN || msg->longitude > LON_MAX) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "GPS lat/lon out of valid range, skipping update "
                "(lat=%.6f, lon=%.6f)", msg->latitude, msg->longitude);
            return;
        }

        // status 확인
        if (msg->status < 1) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "GPS status=%d, skipping update", msg->status);
            return;
        }

        // WGS84 → UTM → 로컬 좌표
        auto utm = utm_conv::latlon_to_utm(msg->latitude, msg->longitude);
        double local_x = utm.easting  - origin_easting_;
        double local_y = utm.northing - origin_northing_;

        // ─── [3차 추가] UTM 변환 결과 NaN/Inf 검증 (이중 안전장치) ───
        if (!std::isfinite(local_x) || !std::isfinite(local_y)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "UTM conversion produced non-finite values, skipping update");
            return;
        }

        // GPS accuracy 처리 (NaN/0/음수 안전장치 — 2차에서 이미 반영)
        double acc = msg->accuracy;
        double gps_var;
        if (!std::isfinite(acc) || acc <= 0.0) {
            gps_var = R_gps_default_;
        } else {
            gps_var = std::max(acc * acc, R_gps_default_);
        }

        std::lock_guard<std::mutex> lock(mtx_);

        // 초기화 이전: 캐시만 갱신
        pending_gps_x_ = local_x;
        pending_gps_y_ = local_y;
        gps_received_  = true;
        if (!initialized_) {
            tryInitialize();
            return;
        }

        // ─── EKF 측정 업데이트 (H: 2×5) ───
        Eigen::Vector2d z(local_x, local_y);
        Eigen::Vector2d z_pred = H_gps_ * x_;
        Eigen::Vector2d y_innov = z - z_pred;

        Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * gps_var;

        Eigen::Matrix2d S = H_gps_ * P_ * H_gps_.transpose() + R;
        Eigen::MatrixXd K = P_ * H_gps_.transpose() * S.inverse();

        x_ = x_ + K * y_innov;
        x_(IDX_YAW) = normalizeAngle(x_(IDX_YAW));
        josephUpdate(H_gps_, K, R);
    }

    // ─── Compass 콜백 (측정 업데이트: yaw) ────────────────────────────────
    void compassCallback(const gkusv_driver_interface::msg::UsvCompass::SharedPtr msg)
    {
        // ─── [3차 추가] 입력 검증: heading NaN/Inf ───
        //   heading 이 NaN 으로 한 번이라도 들어오면 yaw_measured = NaN 이 되고,
        //   normalizeAngle 의 while 루프가 NaN 비교로 인해 무한루프 가능
        double heading_deg_raw = static_cast<double>(msg->heading);
        if (!std::isfinite(heading_deg_raw)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Compass heading is NaN/Inf, skipping update");
            return;
        }

        // compass heading: 0~360° (북쪽 기준 시계방향)
        // EKF yaw: rad (동쪽 기준 반시계방향, ENU)
        double yaw_measured = (90.0 - heading_deg_raw) * M_PI / 180.0;
        yaw_measured = normalizeAngle(yaw_measured);

        std::lock_guard<std::mutex> lock(mtx_);

        // 초기화 이전: 캐시만 갱신
        pending_yaw_       = yaw_measured;
        compass_received_  = true;
        if (!initialized_) {
            tryInitialize();
            return;
        }

        // ─── EKF 측정 업데이트 (H: 1×5) ───
        double z_pred = x_(IDX_YAW);
        double y_innov = normalizeAngle(yaw_measured - z_pred);

        double S = P_(IDX_YAW, IDX_YAW) + R_compass_;
        Eigen::VectorXd K = P_.col(IDX_YAW) / S;

        x_ = x_ + K * y_innov;
        x_(IDX_YAW) = normalizeAngle(x_(IDX_YAW));

        Eigen::Matrix<double,1,1> R_mat;
        R_mat(0,0) = R_compass_;
        Eigen::MatrixXd K_mat = K;
        josephUpdate(H_compass_, K_mat, R_mat);
    }

    // ─── 타이머 콜백 (예측 + 발행) ────────────────────────────────────────
    void timerCallback()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!initialized_) return;

        rclcpp::Time now = this->now();
        double dt = (now - last_predict_time_).seconds();
        last_predict_time_ = now;

        predict(dt);
        publishPose(now);
        publishOdom(now);
    }

    // ─── PoseStamped 발행 ─────────────────────────────────────────────────
    void publishPose(const rclcpp::Time & stamp)
    {
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = stamp;
        msg.header.frame_id = "map";

        msg.pose.position.x = x_(IDX_X);
        msg.pose.position.y = x_(IDX_Y);
        msg.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, x_(IDX_YAW));
        msg.pose.orientation = tf2::toMsg(q);

        pose_pub_->publish(msg);
    }

    // ─── Odometry 발행 ───────────────────────────────────────────────────
    void publishOdom(const rclcpp::Time & stamp)
    {
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = stamp;
        msg.header.frame_id = "map";
        msg.child_frame_id  = "base_link";

        msg.pose.pose.position.x = x_(IDX_X);
        msg.pose.pose.position.y = x_(IDX_Y);
        msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, x_(IDX_YAW));
        msg.pose.pose.orientation = tf2::toMsg(q);

        msg.pose.covariance[0]  = P_(IDX_X,   IDX_X);
        msg.pose.covariance[1]  = P_(IDX_X,   IDX_Y);
        msg.pose.covariance[6]  = P_(IDX_Y,   IDX_X);
        msg.pose.covariance[7]  = P_(IDX_Y,   IDX_Y);
        msg.pose.covariance[35] = P_(IDX_YAW, IDX_YAW);

        // world-frame → body-frame twist 변환
        double yaw = x_(IDX_YAW);
        double vx_w = x_(IDX_VX);
        double vy_w = x_(IDX_VY);
        double vx_b =  std::cos(yaw) * vx_w + std::sin(yaw) * vy_w;
        double vy_b = -std::sin(yaw) * vx_w + std::cos(yaw) * vy_w;

        msg.twist.twist.linear.x  = vx_b;
        msg.twist.twist.linear.y  = vy_b;
        msg.twist.twist.angular.z = 0.0;

        msg.twist.covariance[0]  = P_(IDX_VX, IDX_VX);
        msg.twist.covariance[7]  = P_(IDX_VY, IDX_VY);
        msg.twist.covariance[35] = 1e6;

        odom_pub_->publish(msg);
    }
};


// ─── main ──────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EkfLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
