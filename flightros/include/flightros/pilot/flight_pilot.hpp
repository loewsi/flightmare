
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
  std::string object_id9; // Unique name
  std::string object_id10; // Unique name
  std::string object_id11; // Unique name
  std::string object_id12; // Unique name

  std::string object_id13; // Unique name
  std::string object_id14; // Unique name
  std::string object_id15; // Unique name
  std::string object_id16; // Unique name
  std::string object_id17; // Unique name
  std::string object_id18; // Unique name

  std::string object_id19; // Unique name
  std::string object_id20; // Unique name
  std::string object_id21; // Unique name
  std::string object_id22; // Unique name
  std::string object_id23; // Unique name
  std::string object_id24; // Unique name



  std::string prefab_id; // Name of the prefab in the Assets/Resources folder
  std::shared_ptr<StaticGate> gate;
  std::shared_ptr<StaticGate> gate2;
  std::shared_ptr<StaticGate> gate3;
  std::shared_ptr<StaticGate> gate4;
  std::shared_ptr<StaticGate> gate5;
  std::shared_ptr<StaticGate> gate6;
  std::shared_ptr<StaticGate> gate7;
  std::shared_ptr<StaticGate> gate8;
  std::shared_ptr<StaticGate> gate9;
  std::shared_ptr<StaticGate> gate10;
  std::shared_ptr<StaticGate> gate11;
  std::shared_ptr<StaticGate> gate12;

  std::shared_ptr<StaticGate> gate13;
  std::shared_ptr<StaticGate> gate14;
  std::shared_ptr<StaticGate> gate15;
  std::shared_ptr<StaticGate> gate16;
  std::shared_ptr<StaticGate> gate17;
  std::shared_ptr<StaticGate> gate18;

  std::shared_ptr<StaticGate> gate19;
  std::shared_ptr<StaticGate> gate20;
  std::shared_ptr<StaticGate> gate21;
  std::shared_ptr<StaticGate> gate22;
  std::shared_ptr<StaticGate> gate23;
  std::shared_ptr<StaticGate> gate24;


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