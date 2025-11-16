#include "my_util_tools.hpp"

const double INF = 1.145e14;
const int dx[8] = {0, 0, 1,-1, 1, 1,-1,-1};
const int dy[8] = {1,-1, 0, 0, 1,-1, 1,-1};

Point2D::Point2D() {}
Point2D::Point2D(const float X, const float Y) : x(X), y(Y) {}
Point3D::Point3D() {}
Point3D::Point3D(const float X, const float Y, const float Z) : x(X), y(Y), z(Z) {}




//!======================== vec2 - vec2 =========================
CAST<vec2>::CAST(const vec2 &TMP) : tmp(TMP) { }
CAST<vec2>::operator vec2() const { return tmp; }
CAST<vec2>& CAST<vec2>::operator = (const vec2 &TMP) { this->tmp = TMP; return *this; }
//!==============================================================


//!======================== vec2 - Point2D =========================
CAST<vec2>::CAST(const Point2D &TMP) : tmp(TMP.x, TMP.y) { }
CAST<vec2>::operator Point2D() const { return Point2D(tmp.x(), tmp.y()); }
CAST<vec2>& CAST<vec2>::operator = (const Point2D &TMP)
{
  this->tmp = vec2(TMP.x, TMP.y);
  return *this;
}
//!==============================================================


//!======================= rot3d - rot3d =========================
CAST<rot3d>::CAST(const rot3d &TMP) : tmp(TMP) { }
CAST<rot3d>::operator rot3d() const { return tmp; }
CAST<rot3d>& CAST<rot3d>::operator = (const rot3d &TMP) { this->tmp = TMP; return *this; }
//!==============================================================

// //!======================= rot3d - Eigen::Block =================
// template<typename XprType, bool InnerPanel>
// CAST<rot3d>::CAST(const Eigen::Block<XprType, 3, 3, InnerPanel> &TMP) : tmp(TMP) { }
// template<typename XprType, bool InnerPanel>
// CAST<rot3d>::operator Eigen::Block<XprType, 3, 3, InnerPanel>() const { return ans; }
// template<typename XprType, bool InnerPanel>
// CAST<rot3d>& CAST<rot3d>::operator = (const Eigen::Block<XprType, 3, 3, InnerPanel> &TMP) { this->tmp = TMP; return *this; }
// //!==============================================================


//!======================= rot3d - Eigen::Quaternion ============
CAST<rot3d>::CAST(const Eigen::Quaterniond &TMP) : tmp(TMP) { }
CAST<rot3d>::operator Eigen::Quaterniond() const { return Eigen::Quaterniond(tmp); }
CAST<rot3d>& CAST<rot3d>::operator = (const Eigen::Quaterniond &TMP) { tmp = rot3d(TMP); return *this; }
//!==============================================================

//!======================= rot3d -   geometry_msgs::msg::Quaternion ====
CAST<rot3d>::CAST(const   geometry_msgs::msg::Quaternion &TMP)
  : tmp(Eigen::Quaterniond(TMP.w, TMP.x, TMP.y, TMP.z)) { }
CAST<rot3d>::operator   geometry_msgs::msg::Quaternion() const
{
  const Eigen::Quaterniond q(tmp);
    geometry_msgs::msg::Quaternion ans;
  ans.w = q.w(), ans.x = q.x(),
  ans.y = q.y(), ans.z = q.z();
  return ans;
}
CAST<rot3d>& CAST<rot3d>::operator = (const   geometry_msgs::msg::Quaternion &TMP)
{
  this->tmp = rot3d(Eigen::Quaterniond(TMP.w, TMP.x, TMP.y, TMP.z));
  return *this;
}
//!==============================================================



//!======================= vec3 - vec3 ==========================
CAST<vec3>::CAST(const vec3 &TMP) : tmp(TMP.cast<double>()) { }
CAST<vec3>::operator vec3() const { return tmp; }
CAST<vec3>& CAST<vec3>::operator = (const vec3 &TMP) { this->tmp = TMP; return *this; }
//!==============================================================


//!======================= vec3 - Eigen::Block ==================
// template<typename XprType, bool InnerPanel>
// CAST<vec3>::CAST(const Eigen::Block<XprType, 3, 1, InnerPanel> &TMP)
//   : tmp(TMP)
// {
// }
// template<typename XprType, bool InnerPanel>
// CAST<vec3>::operator Eigen::Block<XprType, 3, 1, InnerPanel>() const
// {
//   Eigen::Block<XprType, 3, 1, InnerPanel> ans(tmp);
//   return ans;
// }
// template<typename XprType, bool InnerPanel>
// CAST<vec3>& CAST<vec3>::operator = (const Eigen::Block<XprType, 3, 1, InnerPanel> &TMP)
// {
//   this->tmp = TMP;
// }
//!==============================================================



//!======================= vec3 -   geometry_msgs::msg::Point ==========
CAST<vec3>::CAST(const   geometry_msgs::msg::Point &TMP)
  : tmp(TMP.x, TMP.y, TMP.z) { }
CAST<vec3>::operator   geometry_msgs::msg::Point() const
{
    geometry_msgs::msg::Point ans;
  ans.x = tmp.x(), ans.y = tmp.y(), ans.z = tmp.z();
  return ans;
}
CAST<vec3>& CAST<vec3>::operator = (const   geometry_msgs::msg::Point &TMP)
{
  this->tmp = vec3(TMP.x, TMP.y, TMP.z);
  return *this;
}
//!==============================================================

//!======================= vec3 -   geometry_msgs::msg::Vector3 ========
CAST<vec3>::CAST(const   geometry_msgs::msg::Vector3 &TMP)
  : tmp(TMP.x, TMP.y, TMP.z) { }
CAST<vec3>::operator   geometry_msgs::msg::Vector3() const
{
    geometry_msgs::msg::Vector3 ans;
  ans.x = tmp.x(), ans.y = tmp.y(), ans.z = tmp.z();
  return ans;
}
CAST<vec3>& CAST<vec3>::operator = (const   geometry_msgs::msg::Vector3 &TMP)
{
  this->tmp = vec3(TMP.x, TMP.y, TMP.z);
  return *this;
}
//!==============================================================


//!======================= vec3 - Point3D =======================
CAST<vec3>::CAST(const Point3D &TMP)
  : tmp(TMP.x, TMP.y, TMP.z) { }
CAST<vec3>::operator Point3D() const
{
  Point3D ans;
  ans.x = tmp.x(), ans.y = tmp.y(), ans.z = tmp.z();
  return ans;
}
CAST<vec3>& CAST<vec3>::operator = (const Point3D &TMP)
{
  this->tmp = vec3(TMP.x, TMP.y, TMP.z);
  return *this;
}
//!==============================================================


//!======================= tr3d - tr3d ==========================
CAST<tr3d>::CAST(const tr3d &TMP) : tmp(TMP.cast<double>()) { }
CAST<tr3d>::operator tr3d() const { return tmp; }
CAST<tr3d>& CAST<tr3d>::operator = (const tr3d &TMP) { this->tmp = tr3d(TMP.cast<double>()); return *this; }
//!==============================================================



//!======================= tr3d -   geometry_msgs::msg::Pose ===========
CAST<tr3d>::CAST(const   geometry_msgs::msg::Pose &TMP)
{
  tmp.linear() = (rot3d)CAST<rot3d>(TMP.orientation);
  tmp.translation() = (vec3)CAST<vec3>(TMP.position);
}
CAST<tr3d>::operator   geometry_msgs::msg::Pose() const
{
    geometry_msgs::msg::Pose ans;
  ans.orientation = CAST<rot3d>(tmp.linear());
  ans.position = CAST<vec3>(tmp.translation());
  return ans;
}
CAST<tr3d>& CAST<tr3d>::operator = (const   geometry_msgs::msg::Pose &TMP)
{
  this->tmp.linear() = (rot3d)CAST<rot3d>(TMP.orientation);
  this->tmp.translation() = (vec3)CAST<vec3>(TMP.position);
  return *this;
}
//!==============================================================


//!======================= tr3d -   geometry_msgs::msg::Transform ======
CAST<tr3d>::CAST(const   geometry_msgs::msg::Transform &TMP)
{
  tmp.linear() = (rot3d)CAST<rot3d>(TMP.rotation);
  tmp.translation() = (vec3)CAST<vec3>(TMP.translation);
}
CAST<tr3d>::operator   geometry_msgs::msg::Transform() const
{
    geometry_msgs::msg::Transform ans;
  ans.rotation = CAST<rot3d>(tmp.linear());
  ans.translation = CAST<vec3>(tmp.translation());
  return ans;
}
CAST<tr3d>& CAST<tr3d>::operator = (const   geometry_msgs::msg::Transform &TMP)
{
  tmp.linear() = (rot3d)CAST<rot3d>(TMP.rotation);
  tmp.translation() = (vec3)CAST<vec3>(TMP.translation);
  return *this;
}
//!==============================================================

TF2Pack::TF2Pack(rclcpp::Node *nh)
{
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(nh->get_clock());
  // tf2_buffer_->setUsingDedicatedThread(true); // no need to do this
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  tf2_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*nh);
}
CAST<tr3d> TF2Pack::listenTransform(
  const std::string &frame_id, 
  const std::string &child_frame)
{
  geometry_msgs::msg::TransformStamped tmp;
  tmp = tf2_buffer_->lookupTransform(frame_id, child_frame, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.4));
  return tmp.transform;
}
void TF2Pack::publishTransform(
  const std::string &frame_id, 
  const std::string &child_frame, 
  const rclcpp::Time &stamp, 
  const CAST<tr3d> &transform)
{
    geometry_msgs::msg::TransformStamped msg;
  msg.header.frame_id = frame_id;
  msg.child_frame_id = child_frame;
  msg.header.stamp = stamp;
  msg.transform = CAST<tr3d>(transform);
  tf2_broadcaster_->sendTransform(msg);
}


CAST<tr3d> Vec2Tr3d(const std::vector<double> &vec)
{
  const int vecSize = vec.size();
  const auto flag = vecSize==6||vecSize==7;
  if(flag == false)
  {
    throw std::runtime_error("Vec2Tr3d: vecSize invalid, targetSize 6 or 7, inputSize " + vecSize);
  }
  tr3d res;
  res.translation() = vec3(vec[0], vec[1], vec[2]);
  if(vecSize == 6)
  {
    res.linear() = Eigen::Matrix3d(
        Eigen::AngleAxisd(vec[3], Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(vec[4], Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(vec[5], Eigen::Vector3d::UnitX()));
  }
  else 
  {
    const Eigen::Quaterniond q(vec[3], vec[4], vec[5], vec[6]);
    res.linear() = q.matrix();
  }
  return res;
}

std::vector<double> Tr3d2Vec(const CAST<tr3d> &pos)
{
  static const double rad2deg = 180.0/M_PI;
  const tr3d tmp = pos;
  const rot3d r = tmp.linear();
  const vec3 t = tmp.translation();
  const vec3 euler = r.eulerAngles(2, 1, 0);
  return {t.x(), t.y(), t.z(), 
    euler.x()*rad2deg, euler.y()*rad2deg, euler.z()*rad2deg};
}
