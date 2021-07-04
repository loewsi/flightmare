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
  object_id9 = "unity_gate9"; // Unique name
  object_id10 = "unity_gate10"; // Unique name
  object_id11 = "unity_gate11"; // Unique name
  object_id12 = "unity_gate12"; // Unique name

  object_id13 = "unity_gate13"; // Unique name
  object_id14 = "unity_gate14"; // Unique name
  object_id15 = "unity_gate15"; // Unique name
  object_id16 = "unity_gate16"; // Unique name
  object_id17 = "unity_gate17"; // Unique name
  object_id18 = "unity_gate18"; // Unique name

  object_id19 = "unity_gate19"; // Unique name
  object_id20 = "unity_gate20"; // Unique name
  object_id21 = "unity_gate21"; // Unique name
  object_id22 = "unity_gate22"; // Unique name
  object_id23 = "unity_gate23"; // Unique name
  object_id24 = "unity_gate24"; // Unique name

  gate = std::make_shared<StaticGate>(object_id, prefab_id);
  gate->setPosition(Eigen::Vector3f(0.5, 1.5, 7.3));
  gate->setRotation(
    Quaternion(0, 0.3250816, -0.3250816, 0.8880562));

  gate2 = std::make_shared<StaticGate>(object_id2, prefab_id);
  gate2->setPosition(Eigen::Vector3f(-0.3, 7.5, 6.8));
  gate2->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate3 = std::make_shared<StaticGate>(object_id3, prefab_id);
  gate3->setPosition(Eigen::Vector3f(-1.0, 9.0, 6.2));
  gate3->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate4 = std::make_shared<StaticGate>(object_id4, prefab_id);
  gate4->setPosition(Eigen::Vector3f(1.3, 1.0, 6.5));
  gate4->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate5 = std::make_shared<StaticGate>(object_id5, prefab_id);
  gate5->setPosition(Eigen::Vector3f(1.3, 9.5, 6.2));
  gate5->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate6 = std::make_shared<StaticGate>(object_id6, prefab_id);
  gate6->setPosition(Eigen::Vector3f(0.4, 5.0, 7.7));
  gate6->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate7 = std::make_shared<StaticGate>(object_id7, prefab_id);
  gate7->setPosition(Eigen::Vector3f(1.0, 5.0, 5.8));
  gate7->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate8 = std::make_shared<StaticGate>(object_id8, prefab_id);
  gate8->setPosition(Eigen::Vector3f(-0.8, 2.0, 6.5));
  gate8->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate9 = std::make_shared<StaticGate>(object_id9, prefab_id);
  gate9->setPosition(Eigen::Vector3f(-0.8, 12.4, 6.5));
  gate9->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate10 = std::make_shared<StaticGate>(object_id10, prefab_id);
  gate10->setPosition(Eigen::Vector3f(-0.4, 14.0, 5.3));
  gate10->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate11 = std::make_shared<StaticGate>(object_id11, prefab_id);
  gate11->setPosition(Eigen::Vector3f(0.7, 13.4, 5.7));
  gate11->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate12 = std::make_shared<StaticGate>(object_id12, prefab_id);
  gate12->setPosition(Eigen::Vector3f(1.6, 10.5, 7.8));
  gate12->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate13 = std::make_shared<StaticGate>(object_id13, prefab_id);
  gate13->setPosition(Eigen::Vector3f(0.5, 2.5, 5.3));
  gate13->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate14 = std::make_shared<StaticGate>(object_id14, prefab_id);
  gate14->setPosition(Eigen::Vector3f(-0.3, 3.7, 5.2));
  gate14->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate15 = std::make_shared<StaticGate>(object_id15, prefab_id);
  gate15->setPosition(Eigen::Vector3f(-1.7, 12.7, 7.4));
  gate15->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate16 = std::make_shared<StaticGate>(object_id16, prefab_id);
  gate16->setPosition(Eigen::Vector3f(1.6, 13.5, 7.6));
  gate16->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate17 = std::make_shared<StaticGate>(object_id17, prefab_id);
  gate17->setPosition(Eigen::Vector3f(0.2, 5.2, 5.3));
  gate17->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate18 = std::make_shared<StaticGate>(object_id18, prefab_id);
  gate18->setPosition(Eigen::Vector3f(-1.1, 8.1, 5.3));
  gate18->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate19 = std::make_shared<StaticGate>(object_id19, prefab_id);
  gate19->setPosition(Eigen::Vector3f(-1.3, 3.4, 6.3));
  gate19->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate20 = std::make_shared<StaticGate>(object_id20, prefab_id);
  gate20->setPosition(Eigen::Vector3f(-0.9, 14.7, 5.5));
  gate20->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate21 = std::make_shared<StaticGate>(object_id21, prefab_id);
  gate21->setPosition(Eigen::Vector3f(1.7, 11.7, 7.4));
  gate21->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate22 = std::make_shared<StaticGate>(object_id22, prefab_id);
  gate22->setPosition(Eigen::Vector3f(2.0, 3.5, 7.6));
  gate22->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate23 = std::make_shared<StaticGate>(object_id23, prefab_id);
  gate23->setPosition(Eigen::Vector3f(0.6, 8.2, 5.1));
  gate23->setRotation(
    Quaternion(0.4254518, 0.4254518, 0.237339, 0.762661));

  gate24 = std::make_shared<StaticGate>(object_id24, prefab_id);
  gate24->setPosition(Eigen::Vector3f(-1.8, 5.1, 6.3));
  gate24->setRotation(
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


  depth_pub = it_.advertise("depth_img", 1);

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
  
  unity_bridge_ptr_->getRender(frame_id);
  unity_bridge_ptr_->handleOutput();

  cv::Mat img;

  ros::Time timestamp = ros::Time::now();

  rgb_camera_->getDepthMap(img);
  sensor_msgs::ImagePtr depth_msg = 
    cv_bridge::CvImage(std_msgs::Header(), "16FC1", img).toImageMsg();
  depth_msg->header.stamp = timestamp;
  depth_msg->header.frame_id = "camera";
  depth_pub.publish(depth_msg);

  frame_id += 1;
  
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
    unity_bridge_ptr_->addStaticObject(gate9);
    unity_bridge_ptr_->addStaticObject(gate10);
    unity_bridge_ptr_->addStaticObject(gate11);
    unity_bridge_ptr_->addStaticObject(gate12);

    unity_bridge_ptr_->addStaticObject(gate13);
    unity_bridge_ptr_->addStaticObject(gate14);
    unity_bridge_ptr_->addStaticObject(gate15);
    unity_bridge_ptr_->addStaticObject(gate16);
    unity_bridge_ptr_->addStaticObject(gate17);
    unity_bridge_ptr_->addStaticObject(gate18);
/*
    unity_bridge_ptr_->addStaticObject(gate19);
    unity_bridge_ptr_->addStaticObject(gate20);
    unity_bridge_ptr_->addStaticObject(gate21);
    unity_bridge_ptr_->addStaticObject(gate22);
    unity_bridge_ptr_->addStaticObject(gate23);
    unity_bridge_ptr_->addStaticObject(gate24);*/
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