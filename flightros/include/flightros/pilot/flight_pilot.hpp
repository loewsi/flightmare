
#pragma once

#include <memory>

// ros
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CompressedImage.h>

// rpg quadrotor
#include <autopilot/autopilot_helper.h>
#include <autopilot/autopilot_states.h>
#include <quadrotor_common/parameter_helper.h>
#include <quadrotor_msgs/AutopilotFeedback.h>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/rgb_camera.hpp"
#include "flightlib/objects/static_gate.hpp"

using namespace flightlib;

namespace flightros {

class FlightPilot {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FlightPilot(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~FlightPilot();

  // callbacks
  void mainLoopCallback(const ros::TimerEvent& event);
  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);

  bool setUnity(const bool render);
  bool connectUnity(void);
  bool loadParams(void);

 private:
  // ros nodes
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;

  // publisher
  image_transport::Publisher depth_pub;

  // subscriber
  ros::Subscriber sub_state_est_;

  // main loop timer
  ros::Timer timer_main_loop_;

  // unity quadrotor
  std::shared_ptr<Quadrotor> quad_ptr_;
  std::shared_ptr<RGBCamera> rgb_camera_;
  QuadState quad_state_;

  std::string object_id; // Unique name
  std::string object_id2; // Unique name
  std::string object_id3; // Unique name
  std::string object_id4; // Unique name
  std::string object_id5; // Unique name
  std::string object_id6; // Unique name
  std::string object_id7; // Unique name
  std::string object_id8; // Unique name

  std::string prefab_id; // Name of the prefab in the Assets/Resources folder
  std::shared_ptr<StaticGate> gate;
  std::shared_ptr<StaticGate> gate2;
  std::shared_ptr<StaticGate> gate3;
  std::shared_ptr<StaticGate> gate4;
  std::shared_ptr<StaticGate> gate5;
  std::shared_ptr<StaticGate> gate6;
  std::shared_ptr<StaticGate> gate7;
  std::shared_ptr<StaticGate> gate8;

  // Flightmare(Unity3D)
  std::shared_ptr<UnityBridge> unity_bridge_ptr_;
  SceneID scene_id_{UnityScene::WAREHOUSE};
  bool unity_ready_{false};
  bool unity_render_{false};
  RenderMessage_t unity_output_;
  uint16_t receive_id_{0};
  FrameID frame_id;

  // auxiliary variables
  Scalar main_loop_freq_{50.0};
};
}  // namespace flightros