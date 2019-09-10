#ifndef ROBOMAKER_RACECAR_PLUGIN_HH
#define ROBOMAKER_RACECAR_PLUGIN_HH

#include <functional>
#include <cmath>
#include <iostream>
#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

#include "rclcpp/rclcpp.hpp"
#include "deepracer_msgs/msg/progress.hpp"

#if GAZEBO_MAJOR_VERSION >= 9
#include <ignition/math/Pose3.hh>
#else
#include <gazebo/math/Vector3.hh>
#endif

namespace gazebo
{
class RacecarPlugin : public ModelPlugin
{
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

  void sendProgress(const int off_track, const double yaw, const double x, const double y, const double z,
                    const double progrezz, const double distanceFromCenter, const double distanceFromBorder1,
                    const double distanceFromBorder2);

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3<double> getCarPosition(const ignition::math::Pose3<double> pose);
#else
  gazebo::math::Vector3 getCarPosition(const gazebo::math::Pose pose);
#endif

  void OnUpdate();

  void setUpWorld(const std::string worldName);

  double minimumDistanceFromEdges(const double x, const double y);

  // Finds the distance of a point from a edge.
  // We compute the distance of the point (x, y) from the infinite line formed by the
  // 2 points (lx1, ly1) and (lx2, ly2).
  //
  // We also check that the point is inside a bounding box formed by
  // (lx1 Â± margin, ly1 Â± margin) and (lx2 Â± margin, ly2 Â± margin). The distance
  // that we compute is from the infinite line formed by the 2 points. The bounding
  // box check helps ensure that we consider the line only in below (lx1 Â± margin, ly1 Â± margin)
  // and (lx2 Â± margin, ly2 Â± margin).
  double minimumDistanceFromEdge(const double x, const double y, const double lx1, const double ly1, const double lx2,
                                 const double ly2);

  // We fire a axis parallel ray from the point and see how many edges it intersects. Given
  // the polygon is convex, if the count of intersection is equal to 1, the point is inside
  // the polygon else its outside the polygon.
  bool insidePolygon(const double x, const double y);

  // Using lines equation, it computes whether the ray intersects the
  // the edge. If it intersects, it returns true else return false.
  bool doesRayIntersectLine(const double x, const double y, const double lx1, const double ly1, const double lx2,
                            const double ly2);

private:
  int MAX_POLYGON_VERTEX = 100;
  double MAX_VALUE = 9999999;
  double EPSILON = 1e-6;

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3<double> RELATIVE_POSITION_OF_FRONT_OF_CAR = ignition::math::Vector3<double>(0.25, 0, 0);
#else
  gazebo::math::Vector3 RELATIVE_POSITION_OF_FRONT_OF_CAR = gazebo::math::Vector3(0.25, 0, 0);
#endif

  double vertices[100][2];
  double numberOfVertices = 0;
  double margin = 0;
  double prog = 0;
  double lastX = -MAX_VALUE, lastY = -MAX_VALUE;
  std::string worldName;
  rclcpp::Publisher<deepracer_msgs::msg::Progress>::SharedPtr chatter_pub;

  // Pointer to the model
  physics::ModelPtr model;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(RacecarPlugin)

}  // namespace gazebo

#endif  // ROBOMAKER_RACECAR_PLUGIN_HH
