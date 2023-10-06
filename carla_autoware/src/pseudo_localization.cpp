#include "rclcpp/rclcpp.hpp"

#include "carla_msgs/msg/carla_ego_vehicle_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "tier4_debug_msgs/msg/float32_stamped.hpp"
#include "tier4_system_msgs/msg/mode_change_available.hpp"
#include "autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp"

#include <memory>

using std::placeholders::_1;

class PseudoLocalization: public rclcpp::Node
{
public:
    explicit PseudoLocalization();
private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_kinematic_state;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_with_covariance;
    rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr pub_accel_with_covariance;
    rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr pub_voxel_transformation_likelihood;
    rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr pub_transform_probability;
    rclcpp::Publisher<tier4_system_msgs::msg::ModeChangeAvailable>::SharedPtr pub_componet_state_monitor_launch;
    rclcpp::Publisher<tier4_system_msgs::msg::ModeChangeAvailable>::SharedPtr pub_componet_state_monitor_auto;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_stamped;
    rclcpp::Publisher<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>::SharedPtr pub_initialization_state;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleStatus>::SharedPtr sub_status;

    void egoVehicleOdomCb(const nav_msgs::msg::Odometry &in_odom);
    void egoVehicleStatusCb(const carla_msgs::msg::CarlaEgoVehicleStatus &in_status);
};

PseudoLocalization::PseudoLocalization(): Node("pseudo_localization_node")
{
    pub_kinematic_state = this->create_publisher<nav_msgs::msg::Odometry>("/localization/kinematic_state", 10);
    pub_pose_with_covariance = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/localization/pose_estimator/pose_with_covariance", 10);
    pub_accel_with_covariance = this->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>("/localization/acceleration", 10);
    pub_voxel_transformation_likelihood = this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>("/localization/pose_estimator/nearest_voxel_transformation_likelihood", 10);
    pub_componet_state_monitor_launch = this->create_publisher<tier4_system_msgs::msg::ModeChangeAvailable>("/system/component_state_monitor/component/launch/localization", 10);
    pub_componet_state_monitor_auto = this->create_publisher<tier4_system_msgs::msg::ModeChangeAvailable>("/system/component_state_monitor/component/autonomous/localization", 10);
    pub_transform_probability = this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>("/localization/pose_estimator/transform_probability", 10);
    pub_pose_stamped = this->create_publisher<geometry_msgs::msg::PoseStamped>("/localization/pose_twist_fusion_filter/pose", 10);
    pub_initialization_state = this->create_publisher<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>("/localization/initialization_state", 10);

    sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("/carla/ego_vehicle/odometry", 10, std::bind(&PseudoLocalization::egoVehicleOdomCb, this, _1));
    sub_status = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleStatus>("/carla/ego_vehicle/vehicle_status", 10, std::bind(&PseudoLocalization::egoVehicleStatusCb, this, _1));
}

void PseudoLocalization::egoVehicleOdomCb(const nav_msgs::msg::Odometry &in_odom) {

    std::cout << "published" << std::endl;
    rclcpp::Time now = this->get_clock()->now();

    nav_msgs::msg::Odometry odom = in_odom;
    geometry_msgs::msg::PoseWithCovarianceStamped pose_covariance;
    geometry_msgs::msg::PoseStamped pose_stamped;
    tier4_debug_msgs::msg::Float32Stamped voxel_likelihood, transform_prob;
    tier4_system_msgs::msg::ModeChangeAvailable component_state;
    autoware_adapi_v1_msgs::msg::LocalizationInitializationState initialization_state;


    odom.child_frame_id = "base_link";

    pose_covariance.header.stamp = now;
    pose_covariance.header.frame_id = "map";
    pose_covariance.pose = odom.pose;

    pose_stamped.header.stamp = now;
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose = odom.pose.pose;

    voxel_likelihood.stamp = now;
    voxel_likelihood.data = 1.0;

    transform_prob.stamp = now;
    transform_prob.data = 1.0;

    component_state.stamp = now;
    component_state.available = true;

    initialization_state.stamp = now;
    initialization_state.state = autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZED;

    pub_kinematic_state->publish(odom);
    pub_pose_with_covariance->publish(pose_covariance);
    pub_voxel_transformation_likelihood->publish(voxel_likelihood);
    pub_transform_probability->publish(transform_prob);
    pub_componet_state_monitor_auto->publish(component_state);
    pub_componet_state_monitor_launch->publish(component_state);
    pub_pose_stamped->publish(pose_stamped);
    pub_initialization_state->publish(initialization_state);
}

void PseudoLocalization::egoVehicleStatusCb(const carla_msgs::msg::CarlaEgoVehicleStatus &in_status)
{
    std::cout << "published" << std::endl;
    rclcpp::Time now = this->get_clock()->now();

    geometry_msgs::msg::AccelWithCovarianceStamped accel;
    accel.header.stamp = now;
    accel.header.frame_id = "map";
    accel.accel.accel = in_status.acceleration;
    pub_accel_with_covariance->publish(accel);
}


int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PseudoLocalization>());
    return 0;
}
