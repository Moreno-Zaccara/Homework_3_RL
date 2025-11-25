#include <iostream>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <offboard_rl/utils.h>
 
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
 
// struttura waypoint: x, y, z, yaw, tempo per raggiungerlo
struct Waypoint {
    Eigen::Vector4d pos;
    double T;
};
 
class WaypointPlanner : public rclcpp::Node {
public:
    WaypointPlanner() : Node("waypoint_planner") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(5));
 
        local_position_subscription_ = this->create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos,
            std::bind(&WaypointPlanner::vehicle_local_position_callback, this, std::placeholders::_1));
 
        attitude_subscription_ = this->create_subscription<VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos,
            std::bind(&WaypointPlanner::vehicle_attitude_callback, this, std::placeholders::_1));
 
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
 
        timer_offboard_ = this->create_wall_timer(100ms, std::bind(&WaypointPlanner::activate_offboard, this));
        timer_trajectory_publish_ = this->create_wall_timer(20ms, std::bind(&WaypointPlanner::publish_trajectory_setpoint, this));
 
        // --- Inserisci qui i tuoi waypoint ---
        waypoints_ = {
            {{0.0, 0.0, -2.0, 0.0}, 10.0},
            {{2.0, 0.0, -2.0, M_PI/2}, 7.0},
            {{2.0, 2.0, -2.0, 0}, 7.0},
            {{0.0, 2.0, -2.0, M_PI/4}, 6.0},
            {{0.0, 0.0, -2.0, 0}, 8.0},
            {{-2.0, 0.0, -2.0, M_PI}, 8.0},
            {{0.0, 0.0, -2.0, 0.0}, 7.0}
        };
            velocity_ = {
        Eigen::Vector4d(0.0, 0.0, 0.0, 0.0),
        Eigen::Vector4d(0.0, 0.0, 0.0, 0.0),
        Eigen::Vector4d(0.2, 0.0, 0.0, 0.0),
        Eigen::Vector4d(0.0, 0.2, 0.0, 0.0),
        Eigen::Vector4d(-0.2, 0.0, 0.0, 0.0),
        Eigen::Vector4d(0.0, -0.2, 0.0, 0.0),
        Eigen::Vector4d(-0.2, 0.0, 0.0, 0.0),
        Eigen::Vector4d(0.0, 0.0, 0.0, 0.0)
    };
 
 
    }
 
private:
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_subscription_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr attitude_subscription_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
 
    rclcpp::TimerBase::SharedPtr timer_offboard_;
    rclcpp::TimerBase::SharedPtr timer_trajectory_publish_;
 
    bool offboard_active{false};
    VehicleLocalPosition current_position_{};
    VehicleAttitude current_attitude_{};
    double offboard_counter{0};
 
    size_t current_waypoint_idx{0};
    double t{0.0};
    Eigen::Vector4d pos_i, pos_f;
    Eigen::Vector<double, 6> x_x;
    Eigen::Vector<double, 6> x_y;
    Eigen::Vector<double, 6> x_z;
    Eigen::Vector<double, 6> x_w;
    bool trajectory_computed{false};
    std::vector<Waypoint> waypoints_;
 
 
    std::vector<Eigen::Vector4d> velocity_;
 
    void vehicle_local_position_callback(const VehicleLocalPosition::SharedPtr msg) {
        current_position_ = *msg;
    }
 
    void vehicle_attitude_callback(const VehicleAttitude::SharedPtr msg) {
        current_attitude_ = *msg;
    }
 
    void activate_offboard() {
        OffboardControlMode msg{};
        msg.position = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
 
        if(offboard_counter < 10) {
            TrajectorySetpoint init{};
            init.position = {float(current_position_.x), float(current_position_.y), float(current_position_.z)};
            init.yaw = 0.0;
            init.timestamp = this->get_clock()->now().nanoseconds()/1000;
            trajectory_setpoint_publisher_->publish(init);
            offboard_counter++;
            return;
        }
 
        if(offboard_counter == 10) {
            VehicleCommand cmd{};
            cmd.command = VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
            cmd.param1 = 1; cmd.param2 = 6;
            cmd.target_system = 1;
            cmd.target_component = 1;
            cmd.from_external = true;
            cmd.timestamp = this->get_clock()->now().nanoseconds()/1000;
            vehicle_command_publisher_->publish(cmd);
        }
 
        if(offboard_counter == 11) {
            VehicleCommand cmd{};
            cmd.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
            cmd.param1 = 1;
            cmd.target_system = 1;
            cmd.target_component = 1;
            cmd.from_external = true;
            cmd.timestamp = this->get_clock()->now().nanoseconds()/1000;
            vehicle_command_publisher_->publish(cmd);
 
            offboard_active = true;
            pos_i << current_position_.x, current_position_.y, current_position_.z,
                     utilities::quatToRpy(Eigen::Vector4d(current_attitude_.q[0], current_attitude_.q[1], current_attitude_.q[2], current_attitude_.q[3]))[2];
        }
 
        if(offboard_counter < 12) offboard_counter++;
    }
 
    void publish_trajectory_setpoint() {
        if(!offboard_active || current_waypoint_idx >= waypoints_.size()) return;
        
        Waypoint wp = waypoints_[current_waypoint_idx];//switching waypoint when drone reach one of them
        // look at b vectors definition, vel_i is the velocity of the point the drone has just reached.
        Eigen::Vector4d vel_i=velocity_[current_waypoint_idx];
        //the n-th waypoint velocity is the (n+1)-th element of velocity
        Eigen::Vector4d vel_f=velocity_[current_waypoint_idx+1];
       
        if(!trajectory_computed) {    
            pos_f = wp.pos;
 
            Eigen::Matrix<double, 6, 6> A;
            Eigen::VectorXd b_x(6);
            Eigen::VectorXd b_y(6);
            Eigen::VectorXd b_z(6);
            Eigen::VectorXd b_w(6);
 
            b_x << pos_i(0),vel_i(0),0,pos_f(0),vel_f(0),0;
            b_y << pos_i(1),vel_i(1),0,pos_f(1),vel_f(1),0;
            b_z << pos_i(2),vel_i(2),0,pos_f(2),vel_f(2),0;
            b_w << pos_i(3),vel_i(3),0,pos_f(3),vel_f(3),0;
            A << 0,0,0,0,0,1,
                 0,0,0,0,1,0,
                 0,0,0,1,0,0,
                 pow(wp.T,5),pow(wp.T,4),pow(wp.T,3),pow(wp.T,2),wp.T,1,
                 5*pow(wp.T,4),4*pow(wp.T,3),3*pow(wp.T,2),2*wp.T,1,0,
                 20*pow(wp.T,3),12*pow(wp.T,2),6*pow(wp.T,1),1,0,0;
 
            x_x = A.inverse()*b_x;
            x_y = A.inverse()*b_y;
            x_z = A.inverse()*b_z;
            x_w = A.inverse()*b_w;
            trajectory_computed = true;
            t = 0.0;
        }
 
        double ref_pos_x  = x_x(0)*pow(t,5) + x_x(1)*pow(t,4) + x_x(2)*pow(t,3) + x_x(3)*pow(t,2) + x_x(4)*t + x_x(5);
        double ref_vel_x = 5*x_x(0)*pow(t,4) + 4*x_x(1)*pow(t,3) + 3*x_x(2)*pow(t,2) + 2*x_x(3)*t + x_x(4);
        double ref_acc_x = 20*x_x(0)*pow(t,3) + 12*x_x(1)*pow(t,2) + 6*x_x(2)*t + x_x(3);
 
        double ref_pos_y  = x_y(0)*pow(t,5) + x_y(1)*pow(t,4) + x_y(2)*pow(t,3) + x_y(3)*pow(t,2) + x_y(4)*t + x_y(5);
        double ref_vel_y = 5*x_y(0)*pow(t,4) + 4*x_y(1)*pow(t,3) + 3*x_y(2)*pow(t,2) + 2*x_y(3)*t + x_y(4);
        double ref_acc_y = 20*x_y(0)*pow(t,3) + 12*x_y(1)*pow(t,2) + 6*x_y(2)*t + x_y(3);
 
        double ref_pos_z  = x_z(0)*pow(t,5) + x_z(1)*pow(t,4) + x_z(2)*pow(t,3) + x_z(3)*pow(t,2) + x_z(4)*t + x_z(5);
        double ref_vel_z = 5*x_z(0)*pow(t,4) + 4*x_z(1)*pow(t,3) + 3*x_z(2)*pow(t,2) + 2*x_z(3)*t + x_z(4);
        double ref_acc_z = 20*x_z(0)*pow(t,3) + 12*x_z(1)*pow(t,2) + 6*x_z(2)*t + x_z(3);
 
        double ref_pos_w  = x_w(0)*pow(t,5) + x_w(1)*pow(t,4) + x_w(2)*pow(t,3) + x_w(3)*pow(t,2) + x_w(4)*t + x_w(5);
        double ref_vel_w = 5*x_w(0)*pow(t,4) + 4*x_w(1)*pow(t,3) + 3*x_w(2)*pow(t,2) + 2*x_w(3)*t + x_w(4);
        double ref_acc_w = 20*x_w(0)*pow(t,3) + 12*x_w(1)*pow(t,2) + 6*x_w(2)*t + x_w(3);
 
        Eigen::Vector4d ref_pos;
        ref_pos << ref_pos_x, ref_pos_y, ref_pos_z, ref_pos_w;
        Eigen::Vector4d ref_vel;
        ref_vel << ref_vel_x, ref_vel_y, ref_vel_z, ref_vel_w;
        Eigen::Vector4d ref_acc;
        ref_acc << ref_acc_x, ref_acc_y, ref_acc_z, ref_acc_w;
 
 
        TrajectorySetpoint msg{};
        msg.position = {float(ref_pos(0)), float(ref_pos(1)), float(ref_pos(2))};
        msg.velocity = {float(ref_vel(0)), float(ref_vel(1)), float(ref_vel(2))};
        msg.acceleration = {float(ref_acc(0)), float(ref_acc(1)), float(ref_acc(2))};
        msg.yaw = float(ref_pos(3));
        msg.timestamp = this->get_clock()->now().nanoseconds()/1000;
        trajectory_setpoint_publisher_->publish(msg);
 
 
        double dt = 1/50.0;
        t += dt;
        // every time drone reaches a waypoint update pos_i and switch index
        if(t >= wp.T) {           
            pos_i = pos_f;
            trajectory_computed = false;
            current_waypoint_idx++;
        }
    }
};
 
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointPlanner>());
    rclcpp::shutdown();
    return 0;
}
 
 
 