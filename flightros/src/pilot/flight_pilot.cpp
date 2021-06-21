#include "flightros/pilot/flight_pilot.hpp"

namespace flightros {

FlightPilot::FlightPilot(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : nh_(nh),
    pnh_(pnh),
    it_(pnh),
    scene_id_(UnityScene::WAREHOUSE),
    unity_ready_(false),
    unity_render_(false),
    receive_id_(0),
    main_loop_freq_(50.0) {
  // load parameters
  if (!loadParams()) {
    ROS_WARN("[%s] Could not load all parameters.",
             pnh_.getNamespace().c_str());
  } else {
    ROS_INFO("[%s] Loaded all parameters.", pnh_.getNamespace().c_str());
  }

  // quad initialization
  quad_ptr_ = std::make_shared<Quadrotor>();

  // gate initialization
  object_id = "unity_gate"; // Unique name
  prefab_id = "rpg_gate"; // Name of the prefab in the Assets/Resources folder
  object_id2 = "unity_gate2"; // Unique name
  object_id3 = "unity_gate3"; // Unique name
  object_id4 = "unity_gate4"; // Unique name
  object_id5 = "unity_gate5"; // Unique name
  object_id6 = "unity_gate6"; // Unique name
  object_id7 = "unity_gate7"; // Unique name
  object_id8 = "unity_gate8"; // Unique name

  gate = std::make_shared<StaticGate>(object_id, prefab_id);
  gate->setPosition(Eigen::Vector3f(0.3, 1.5, 8.5));
  gate->setRotation(
    Quaternion(0, 0.3250816, -0.3250816, 0.8880562));

  gate2 = std::make_shared<StaticGate>(object_id2, prefab_id);
  gate2->setPosition(Eigen::Vector3f(-0.3, 5.5, 8.5));
  gate2->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate3 = std::make_shared<StaticGate>(object_id3, prefab_id);
  gate3->setPosition(Eigen::Vector3f(0.3, 9.0, 8.5));
  gate3->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate4 = std::make_shared<StaticGate>(object_id4, prefab_id);
  gate4->setPosition(Eigen::Vector3f(1.3, 1.0, 6.5));
  gate4->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate5 = std::make_shared<StaticGate>(object_id5, prefab_id);
  gate5->setPosition(Eigen::Vector3f(-0.7, 7.0, 6.1));
  gate5->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate6 = std::make_shared<StaticGate>(object_id6, prefab_id);
  gate6->setPosition(Eigen::Vector3f(0.4, 9.0, 7.7));
  gate6->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate7 = std::make_shared<StaticGate>(object_id7, prefab_id);
  gate7->setPosition(Eigen::Vector3f(-0.6, 3.0, 5.8));
  gate7->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate8 = std::make_shared<StaticGate>(object_id8, prefab_id);
  gate8->setPosition(Eigen::Vector3f(-0.8, 2.0, 7.5));
  gate8->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));
  


  // add mono camera
  rgb_camera_ = std::make_shared<RGBCamera>();
  Vector<3> B_r_BC(0.0, 0.0, 0.3);
  Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  std::cout << R_BC << std::endl;
  rgb_camera_->setFOV(90);
  rgb_camera_->setWidth(720);
  rgb_camera_->setHeight(480);
  rgb_camera_->setRelPose(B_r_BC, R_BC);
  rgb_camera_->setPostProcesscing(
    std::vector<bool>{true, false, false});
  quad_ptr_->addRGBCamera(rgb_camera_);

  // initialization
  quad_state_.setZero();
  quad_ptr_->reset(quad_state_);


  // initialize subscriber call backs
  sub_state_est_ = nh_.subscribe("ground_truth/odometry", 1,
                                 &FlightPilot::poseCallback, this);

  timer_main_loop_ = nh_.createTimer(ros::Rate(main_loop_freq_),
                                     &FlightPilot::mainLoopCallback, this);


  depth_pub = it_.advertise("depth", 1);

  // wait until the gazebo and unity are loaded
  ros::Duration(5.0).sleep();

  // connect unity
  setUnity(unity_render_);
  connectUnity();
  frame_id = 0;
}

FlightPilot::~FlightPilot() {}

void FlightPilot::poseCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  quad_state_.x[QS::POSX] = (Scalar)msg->pose.pose.position.x;
  quad_state_.x[QS::POSY] = (Scalar)msg->pose.pose.position.y;
  quad_state_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
  quad_state_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  quad_state_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.x;
  quad_state_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.y;
  quad_state_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;
  //
  quad_ptr_->setState(quad_state_);

  if (unity_render_ && unity_ready_) {
    unity_bridge_ptr_->getRender(0);
    unity_bridge_ptr_->handleOutput();
  }
}

void FlightPilot::mainLoopCallback(const ros::TimerEvent &event) {
  /*
  unity_bridge_ptr_->getRender(frame_id);
  unity_bridge_ptr_->handleOutput();

  cv::Mat img;

  ros::Time timestamp = ros::Time::now();

  rgb_camera_->getDepthMap(img);
  sensor_msgs::ImagePtr depth_msg = 
    cv_bridge::CvImage(std_msgs::Header(), "32FC1", img).toImageMsg();
  depth_msg->header.stamp = timestamp;
  depth_msg->header.frame_id = "camera";
  depth_pub.publish(depth_msg);

  frame_id += 1;
  */
}

bool FlightPilot::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    unity_bridge_ptr_->addStaticObject(gate);
    unity_bridge_ptr_->addStaticObject(gate2);
    unity_bridge_ptr_->addStaticObject(gate3);
    unity_bridge_ptr_->addStaticObject(gate4);
    unity_bridge_ptr_->addStaticObject(gate5);
    unity_bridge_ptr_->addStaticObject(gate6);
    unity_bridge_ptr_->addStaticObject(gate7);
    unity_bridge_ptr_->addStaticObject(gate8);
    ROS_INFO("[%s] Unity Bridge is created.", pnh_.getNamespace().c_str());
  }
  return true;
}

bool FlightPilot::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

bool FlightPilot::loadParams(void) {
  // load parameters
  quadrotor_common::getParam("main_loop_freq", main_loop_freq_, pnh_);
  quadrotor_common::getParam("unity_render", unity_render_, pnh_);

  return true;
}

}  // namespace flightros