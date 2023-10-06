#include "rclcpp/rclcpp.hpp"

#include "carla_msgs/msg/carla_ego_vehicle_status.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_control.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"

#include <memory>

using std::placeholders::_1;

class CarlaVehicleInterface: public rclcpp::Node
{
public:
    explicit CarlaVehicleInterface();
private:
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr pub_vel_state;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr pub_steering_state;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr pub_ctrl_mode;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr pub_gear_state;
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr pub_control;

    rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleStatus>::SharedPtr sub_status;
    rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr sub_control;

    void egoVehicleStatusCb(const carla_msgs::msg::CarlaEgoVehicleStatus &in_status);
    void controlCb(const autoware_auto_control_msgs::msg::AckermannControlCommand &in_cmd);
    float m_current_vel;
};

CarlaVehicleInterface::CarlaVehicleInterface(): Node("carla_vehicle_interface_node")
{
    pub_vel_state = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", 10);
    pub_steering_state = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", 10);
    pub_ctrl_mode = this->create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode", 10);
    pub_gear_state = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>("/vehicle/status/gear_status", 10);
    pub_control = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 10);

    sub_status = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleStatus>("/carla/ego_vehicle/vehicle_status", 10, std::bind(&CarlaVehicleInterface::egoVehicleStatusCb, this, _1));
    sub_control = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/trajectory_follower/control_cmd", 10, std::bind(&CarlaVehicleInterface::controlCb, this, _1));
}

void CarlaVehicleInterface::egoVehicleStatusCb(const carla_msgs::msg::CarlaEgoVehicleStatus &in_status)
{
    std::cout << "published" << std::endl;

    autoware_auto_vehicle_msgs::msg::VelocityReport out_vel_state;
    autoware_auto_vehicle_msgs::msg::SteeringReport out_steering_state;
    autoware_auto_vehicle_msgs::msg::ControlModeReport out_ctrl_mode;
    autoware_auto_vehicle_msgs::msg::GearReport out_gear_state;

    rclcpp::Time now = this->get_clock()->now();
    out_vel_state.header.stamp = in_status.header.stamp;
    out_vel_state.header.frame_id = "base_link";
    out_vel_state.longitudinal_velocity = in_status.velocity;
    out_vel_state.lateral_velocity = 0.0;
    out_vel_state.heading_rate = 0.0;
    m_current_vel = in_status.velocity;

    out_steering_state.stamp = in_status.header.stamp;
    out_steering_state.steering_tire_angle = -in_status.control.steer;

    out_gear_state.stamp = in_status.header.stamp;
    out_gear_state.report = in_status.control.gear+1;

    out_ctrl_mode.stamp = in_status.header.stamp;
    out_ctrl_mode.mode = 1;

    pub_vel_state->publish(out_vel_state);
    pub_steering_state->publish(out_steering_state);
    pub_ctrl_mode->publish(out_ctrl_mode);
    pub_gear_state->publish(out_gear_state);
}

void CarlaVehicleInterface::controlCb(const autoware_auto_control_msgs::msg::AckermannControlCommand &in_cmd)
{
    carla_msgs::msg::CarlaEgoVehicleControl out_cmd;
    float P = 1.0;
    float target_speed = abs(in_cmd.longitudinal.speed);
    out_cmd.throttle = P * (target_speed - m_current_vel);
    out_cmd.steer = -in_cmd.lateral.steering_tire_angle;
    out_cmd.brake = 0;
    out_cmd.hand_brake = (in_cmd.longitudinal.speed == 0) ? true : false;
    out_cmd.reverse = (in_cmd.longitudinal.speed < 0) ? true : false;
    out_cmd.gear = 1;
    out_cmd.manual_gear_shift = false;
    pub_control->publish(out_cmd);
}


int main(int argc, char **argv)
{
//    auto logger=rclcpp::get_logger("");

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarlaVehicleInterface>());
    return 0;
}
