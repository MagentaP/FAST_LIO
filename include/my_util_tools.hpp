#ifndef UTIL_TOOLS_UTIL_HPP
#define UTIL_TOOLS_UTIL_HPP
// C++
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>
#include <array>
#include <unordered_map>
#include <algorithm>
#include <queue>
#include <chrono>
#include <iostream>
#include <random>
#include <unordered_set>
#include <set>
#include <thread>
#include <atomic>
#include <functional>

// // for responding to ctrl+c
// #include <sys/wait.h>
// #include <sys/types.h>

// ros basic
#include <rclcpp/rclcpp.hpp>


// messages
#include <std_msgs/msg/int16_multi_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Eigen>
#include <Eigen/Geometry>

// tf2
// #include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/convert.h>
#include <tf2_ros/transform_broadcaster.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


// using namespace visualization_msgs;
// using namespace geometry_msgs;
// using namespace nav_msgs;
// using namespace sensor_msgs;

using vec2 = Eigen::Vector2d;
using vec3 = Eigen::Vector3d;
using tr3d = Eigen::Transform<double, 3, Eigen::Isometry>;
using tr3f = Eigen::Transform<float, 3, Eigen::Isometry>;
using tr2d = Eigen::Transform<double, 2, Eigen::Isometry>;
using tr2f = Eigen::Transform<float, 2, Eigen::Isometry>;
using rot3d = Eigen::Matrix3d;
using rot3f = Eigen::Matrix3f;

extern const double INF;
extern const int dx[8];
extern const int dy[8];


inline rclcpp::Time TimePCL2ROS(const uint64_t x) { return rclcpp::Time(x/1e6); }
inline uint64_t TimeROS2PCL(const rclcpp::Time &x) { return x.nanoseconds()/1e3; }

enum class TeamColor : uint8_t
{
  UNKNOWN = 0x00,
  RED = 0x1F,
  BLUE = 0xF1
};


struct Point2D
{
  float x, y;
  Point2D();
  Point2D(const float X, const float Y);
}__attribute__((packed));

struct Point3D
{
  float x, y, z;
  Point3D();
  Point3D(const float X, const float Y, const float Z);
}__attribute__((packed));

template<typename T>
class UpdatedValue
{
public:
  T value;
  rclcpp::Time lastUpdate;
  UpdatedValue(const T v): value(v), lastUpdate(rclcpp::Time(0)) { }
  void update(const T v, const rclcpp::Time t)
  {
    value = v;
    lastUpdate = t;
  }
  operator T() const
  {
    return value;
  }
};

//! turn simple things into byte_array
template<typename T>
void VEC_CAST(const T x, std::vector<uint8_t> &result)
{
  result.resize(sizeof(T));
  uint8_t *it = (uint8_t*)&x;
  for(size_t i=0; i<sizeof(T); i++)
    result[i] = it[i];
}
//! turn byte_array into simple things
template<typename T>
void VEC_CAST(const std::vector<uint8_t> &x, T& result)
{
  const int xsize = x.size();
  assert(xsize == sizeof(T));
  uint8_t *it = (uint8_t*)&result;
  for(size_t i=0; i<sizeof(T); i++)
    it[i] = x[i];
}

template<class T> class CAST { };

template<> class CAST<vec2>
{
public:
  CAST(const vec2 &TMP);
  operator vec2() const;
  CAST& operator = (const vec2 &TMP);

  CAST(const Point2D &TMP);
  operator Point2D() const;
  CAST& operator = (const Point2D &TMP);
private:
  vec2 tmp;
};

template<> class CAST<vec3>
{
public:
  CAST(const vec3 &TMP);
  operator vec3() const;
  CAST& operator = (const vec3 &TMP);

  // template<typename XprType, bool InnerPanel>
  // CAST(const Eigen::Block<XprType, 3, 1, InnerPanel> &TMP);
  // template<class XprType, bool InnerPanel>
  // operator Eigen::Block<XprType, 3, 1, InnerPanel>() const;
  // template<class XprType, bool InnerPanel>
  // CAST& operator = (const Eigen::Block<XprType, 3, 1, InnerPanel> &TMP);

  CAST(const geometry_msgs::msg::Point &TMP);
  operator geometry_msgs::msg::Point() const;
  CAST& operator = (const geometry_msgs::msg::Point &TMP);

  CAST(const geometry_msgs::msg::Vector3 &TMP);
  operator geometry_msgs::msg::Vector3() const;
  CAST& operator = (const geometry_msgs::msg::Vector3 &TMP);

  CAST(const Point3D &TMP);
  operator Point3D() const;
  CAST& operator = (const Point3D &TMP);
private:
  vec3 tmp;
};

template<> class CAST<rot3d>
{
public:
  CAST(const rot3d &TMP);
  operator rot3d() const;
  CAST& operator = (const rot3d &TMP);

  // template<typename XprType, bool InnerPanel>
  // CAST(const Eigen::Block<XprType, 3, 3, InnerPanel> &TMP);
  // template<typename XprType, bool InnerPanel>
  // operator Eigen::Block<XprType, 3, 3, InnerPanel>() const;
  // template<typename XprType, bool InnerPanel>
  // CAST& operator = (const Eigen::Block<XprType, 3, 3, InnerPanel> &TMP);

  CAST(const Eigen::Quaterniond &TMP);
  operator Eigen::Quaterniond() const;
  CAST& operator = (const Eigen::Quaterniond &TMP);

  CAST(const geometry_msgs::msg::Quaternion &TMP);
  operator geometry_msgs::msg::Quaternion() const;
  CAST& operator = (const geometry_msgs::msg::Quaternion &TMP);
private:
  rot3d tmp;
};


template<> class CAST<tr3d>
{
public:
  // template<class Scalar>
  // CAST(const Eigen::Transform<Scalar, 3, Eigen::Isometry> &TMP);
  // template<class Scalar>
  // operator Eigen::Transform<Scalar, 3, Eigen::Isometry>() const;
  // template<class Scalar>
  // CAST& operator = (const Eigen::Transform<Scalar, 3, Eigen::Isometry> &TMP);

  CAST(const tr3d &TMP);
  operator tr3d() const;
  CAST& operator = (const tr3d &TMP);

  CAST(const geometry_msgs::msg::Pose &TMP);
  operator geometry_msgs::msg::Pose() const;
  CAST& operator = (const geometry_msgs::msg::Pose &TMP);

  CAST(const geometry_msgs::msg::Transform &TMP);
  operator geometry_msgs::msg::Transform() const;
  CAST& operator = (const geometry_msgs::msg::Transform &TMP);
private:
  tr3d tmp;
};


class TF2Pack
{
public:
  TF2Pack(rclcpp::Node *nh);
  // this would throw tf2::TransformException if target-transform not found
  CAST<tr3d> listenTransform(const std::string &frame_id, const std::string &child_frame);
  void publishTransform
  (
    const std::string &frame_id, 
    const std::string &child_frame, 
    const rclcpp::Time &stamp, 
    const CAST<tr3d> &transform
  );
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;
private:
  
};

/*
 * turn vector in to 3d-tranform
 * vec in form [x, y, z, yaw, pitch, roll] (rad) or [x, y, z, qw, qx, qy, qz]
 */
CAST<tr3d> Vec2Tr3d(const std::vector<double> &vec);
/*
 * turn 3d-tranform in to vector
 * return in form [x, y, z, yaw, pitch, roll] (degree)
 */
std::vector<double> Tr3d2Vec(const CAST<tr3d> &pos);

#endif