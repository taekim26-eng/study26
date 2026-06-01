/**
 * EKF Visualizer Node — 수정 버전
 *
 * 목적: EKF 추정 결과의 시각적 검증을 위한 비교용 토픽 발행
 *   - /ekf/path             : EKF 추정 위치 누적 경로
 *   - /gps/path             : GPS 측정 위치 누적 경로
 *   - /compass/heading_marker : Compass 방향 화살표 (위치는 EKF 기준)
 *
 * 시각적 검증 기준:
 *   1) /ekf/path  ≈ /gps/path        → EKF 위치 추정 정확
 *   2) Compass 화살표 방향 ≈ EKF yaw → EKF 방향 추정 정확
 *
 * 수정사항:
 *   [1] Path 누적 점 개수 제한 (메모리/성능)
 *       - 무한 누적 시 RAM 증가, 매 발행마다 전체 path 복사로 부하
 *       - 최근 MAX_PATH_POINTS 개만 유지
 *   [2] GPS lat/lon NaN/Inf, 위경도 범위 검증 추가
 *       - 비정상값 발생 시 path 에 NaN 포인트가 누적되면 시각화가 깨짐
 *   [3] Compass heading NaN/Inf 검증 추가
 *   [4] 화살표 길이 주석을 실제 코드와 일치 (3m → 0.5m)
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <gkusv_driver_interface/msg/usv_gps.hpp>
#include <gkusv_driver_interface/msg/usv_compass.hpp>
#include <cmath>

namespace utm_conv {
struct UTMCoord { double easting, northing; int zone; char band; };
UTMCoord latlon_to_utm(double lat_deg, double lon_deg)
{
    const double a=6378137.0, f=1.0/298.257223563;
    const double e2=2.0*f-f*f, e_prime2=e2/(1.0-e2), k0=0.9996;
    double lat=lat_deg*M_PI/180.0, lon=lon_deg*M_PI/180.0;
    int zone=static_cast<int>((lon_deg+180.0)/6.0)+1;
    double lon0=((zone-1)*6.0-180.0+3.0)*M_PI/180.0;
    double N=a/std::sqrt(1.0-e2*std::sin(lat)*std::sin(lat));
    double T=std::tan(lat)*std::tan(lat);
    double C=e_prime2*std::cos(lat)*std::cos(lat);
    double A=std::cos(lat)*(lon-lon0);
    double M_val=a*((1.0-e2/4.0-3.0*e2*e2/64.0-5.0*e2*e2*e2/256.0)*lat
              -(3.0*e2/8.0+3.0*e2*e2/32.0+45.0*e2*e2*e2/1024.0)*std::sin(2.0*lat)
              +(15.0*e2*e2/256.0+45.0*e2*e2*e2/1024.0)*std::sin(4.0*lat)
              -(35.0*e2*e2*e2/3072.0)*std::sin(6.0*lat));
    double easting=k0*N*(A+(1.0-T+C)*A*A*A/6.0
                   +(5.0-18.0*T+T*T+72.0*C-58.0*e_prime2)*A*A*A*A*A/120.0)+500000.0;
    double northing=k0*(M_val+N*std::tan(lat)*(A*A/2.0
                   +(5.0-T+9.0*C+4.0*C*C)*A*A*A*A/24.0
                   +(61.0-58.0*T+T*T+600.0*C-330.0*e_prime2)*A*A*A*A*A*A/720.0));
    if(lat_deg<0.0) northing+=10000000.0;
    return {easting, northing, zone, (lat_deg>=0.0)?'N':'S'};
}
}

class EkfVisualizerNode : public rclcpp::Node
{
public:
    EkfVisualizerNode() : Node("ekf_visualizer_node")
    {
        auto origin = utm_conv::latlon_to_utm(ORIGIN_LAT, ORIGIN_LON);
        origin_easting_  = origin.easting;
        origin_northing_ = origin.northing;

        // 구독
        ekf_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/ekf/pose", 10,
            std::bind(&EkfVisualizerNode::ekfCallback, this, std::placeholders::_1));

        gps_sub_ = this->create_subscription<gkusv_driver_interface::msg::UsvGps>(
            "/gps_data", 10,
            std::bind(&EkfVisualizerNode::gpsCallback, this, std::placeholders::_1));

        compass_sub_ = this->create_subscription<gkusv_driver_interface::msg::UsvCompass>(
            "/compass_data", 10,
            std::bind(&EkfVisualizerNode::compassCallback, this, std::placeholders::_1));

        // 발행 (Path)
        ekf_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/ekf/path", 10);
        gps_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/gps/path", 10);

        // 발행 (Marker - compass 방향 화살표)
        compass_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/compass/heading_marker", 10);

        ekf_path_.header.frame_id = "map";
        gps_path_.header.frame_id = "map";

        // 메모리 절약을 위해 미리 reserve
        ekf_path_.poses.reserve(MAX_PATH_POINTS);
        gps_path_.poses.reserve(MAX_PATH_POINTS);

        RCLCPP_INFO(this->get_logger(),
            "EKF Visualizer Node started. (max path points: %zu)",
            MAX_PATH_POINTS);
    }

private:
    // ─── 상수 ───
    static constexpr double ORIGIN_LAT = 36.690691050000005;
    static constexpr double ORIGIN_LON = 128.80968808076932;

    // Path 누적 최대 점 개수
    //   EKF 50Hz × 5000점 = 100초 분량 (1분 40초)
    //   GPS 10Hz × 5000점 = 500초 분량 (8분 20초)
    //   필요시 조정. 너무 크면 메모리/성능 부담, 너무 작으면 경로가 짧게 보임
    static constexpr size_t MAX_PATH_POINTS = 5000;

    // GPS 입력 검증용 위/경도 범위
    static constexpr double LAT_MIN = -90.0;
    static constexpr double LAT_MAX =  90.0;
    static constexpr double LON_MIN = -180.0;
    static constexpr double LON_MAX =  180.0;

    double origin_easting_ = 0.0, origin_northing_ = 0.0;

    // 최신 EKF 위치 (compass 화살표 시작점용)
    double latest_x_ = 0.0, latest_y_ = 0.0;
    bool has_position_ = false;

    nav_msgs::msg::Path ekf_path_, gps_path_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ekf_sub_;
    rclcpp::Subscription<gkusv_driver_interface::msg::UsvGps>::SharedPtr gps_sub_;
    rclcpp::Subscription<gkusv_driver_interface::msg::UsvCompass>::SharedPtr compass_sub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ekf_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr gps_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr compass_marker_pub_;

    // ─── Path 누적 점 개수 제한 헬퍼 ───
    //   FIFO 방식으로 오래된 점부터 제거
    static void trimPath(nav_msgs::msg::Path & path, size_t max_points)
    {
        if (path.poses.size() > max_points) {
            // 초과분만큼 앞쪽 제거
            size_t excess = path.poses.size() - max_points;
            path.poses.erase(path.poses.begin(),
                             path.poses.begin() + excess);
        }
    }

    // ─── EKF 콜백: EKF path 누적 + 최신 위치 캐시 ─────────────────────────
    void ekfCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // EKF 출력에 NaN 이 있으면 path 가 깨지므로 검증
        if (!std::isfinite(msg->pose.position.x) ||
            !std::isfinite(msg->pose.position.y)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "EKF pose contains NaN/Inf, skipping");
            return;
        }

        ekf_path_.header.stamp = msg->header.stamp;
        ekf_path_.poses.push_back(*msg);
        trimPath(ekf_path_, MAX_PATH_POINTS);
        ekf_path_pub_->publish(ekf_path_);

        latest_x_ = msg->pose.position.x;
        latest_y_ = msg->pose.position.y;
        has_position_ = true;
    }

    // ─── GPS 콜백: GPS path 누적 ──────────────────────────────────────────
    void gpsCallback(const gkusv_driver_interface::msg::UsvGps::SharedPtr msg)
    {
        // status 확인
        if (msg->status < 1) return;

        // ─── 입력 검증: NaN/Inf, 위/경도 범위 ───
        if (!std::isfinite(msg->latitude) || !std::isfinite(msg->longitude)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "GPS lat/lon contains NaN/Inf, skipping "
                "(lat=%.6f, lon=%.6f)", msg->latitude, msg->longitude);
            return;
        }
        if (msg->latitude  < LAT_MIN || msg->latitude  > LAT_MAX ||
            msg->longitude < LON_MIN || msg->longitude > LON_MAX) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "GPS lat/lon out of valid range, skipping "
                "(lat=%.6f, lon=%.6f)", msg->latitude, msg->longitude);
            return;
        }

        // WGS84 → UTM → 로컬 좌표
        auto utm = utm_conv::latlon_to_utm(msg->latitude, msg->longitude);
        double lx = utm.easting  - origin_easting_;
        double ly = utm.northing - origin_northing_;

        // UTM 변환 결과 이중 검증
        if (!std::isfinite(lx) || !std::isfinite(ly)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "UTM conversion produced non-finite values, skipping");
            return;
        }

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = msg->header.stamp;
        pose.header.frame_id = "map";
        pose.pose.position.x = lx;
        pose.pose.position.y = ly;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;

        gps_path_.header.stamp = msg->header.stamp;
        gps_path_.poses.push_back(pose);
        trimPath(gps_path_, MAX_PATH_POINTS);
        gps_path_pub_->publish(gps_path_);
    }

    // ─── Compass 콜백: heading 화살표 발행 ────────────────────────────────
    //   화살표 시작점: EKF 추정 위치 (latest_x_, latest_y_)
    //   화살표 방향:   Compass heading 변환 yaw
    //   → 시각적으로 "EKF 위치에서 Compass 가 가리키는 방향" 을 보여줌
    //     EKF yaw 화살표(별도 발행)와 일치하는지 비교 가능
    void compassCallback(const gkusv_driver_interface::msg::UsvCompass::SharedPtr msg)
    {
        if (!has_position_) return;

        // ─── 입력 검증: heading NaN/Inf ───
        double heading_deg = static_cast<double>(msg->heading);
        if (!std::isfinite(heading_deg)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Compass heading is NaN/Inf, skipping");
            return;
        }

        // compass heading (북쪽 기준 시계방향, deg) → ENU yaw (rad)
        double yaw = (90.0 - heading_deg) * M_PI / 180.0;

        auto marker = visualization_msgs::msg::Marker();
        marker.header.stamp = msg->header.stamp;
        marker.header.frame_id = "map";
        marker.ns = "compass";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // 화살표 시작점: EKF 추정 현재 위치
        geometry_msgs::msg::Point start, end;
        start.x = latest_x_;
        start.y = latest_y_;
        start.z = 0.0;

        // 화살표 끝점: heading 방향으로 0.5m
        end.x = latest_x_ + 0.5 * std::cos(yaw);
        end.y = latest_y_ + 0.5 * std::sin(yaw);
        end.z = 0.0;

        marker.points.push_back(start);
        marker.points.push_back(end);

        marker.scale.x = 0.02;  // 화살표 축 두께
        marker.scale.y = 0.04;  // 화살표 머리 두께
        marker.scale.z = 0.02;

        // 노란색
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        compass_marker_pub_->publish(marker);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EkfVisualizerNode>());
    rclcpp::shutdown();
    return 0;
}
