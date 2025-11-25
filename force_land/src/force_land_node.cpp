#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
// punto 2
#include <px4_msgs/msg/vehicle_land_detected.hpp>
// per plottare
#include <std_msgs/msg/float32.hpp>

using namespace std::chrono_literals;

class ForceLand : public rclcpp::Node
{
	public:
	ForceLand() : Node("force_land"), need_land(false), can_force_land(true), was_above_threshold(false)
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position",
		qos, std::bind(&ForceLand::height_callback, this, std::placeholders::_1));
		publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
		// punto 2
		sub_land_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
   		"/fmu/out/vehicle_land_detected",
		qos,
		std::bind(&ForceLand::land_detected_callback, this, std::placeholders::_1));

		//questo lo aggiungo per plottare l'altitudine
		publisher_alt_ = this->create_publisher<std_msgs::msg::Float32>(
        "/out/altitude_enu", 10);

		timer_ = this->create_wall_timer(10ms, std::bind(&ForceLand::activate_switch, this));
	}

	private:
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr publisher_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr sub_land_;
	// per plottare altitudine
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_alt_;


	rclcpp::TimerBase::SharedPtr timer_;

	bool need_land;
	bool can_force_land;
	bool was_above_threshold;

	void height_callback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg)
	{
    float z_ = -msg->z;
    std::cout << "Current drone height: " << z_ << " meters" << std::endl;
	
	// il plot dell'altitudine lo prendo direttamente da qua
	std_msgs::msg::Float32 altitude_msg;
	altitude_msg.data = z_;
	publisher_alt_->publish(altitude_msg);

    // Se siamo sopra la soglia e possiamo ancora forzare il landing
    if (z_ > 20 && can_force_land) {
        was_above_threshold = true;  // segno che siamo mai andati sopra
        need_land = true;           // il timer inizierà a mandare NAV_LAND in loop
    }

	 // Se siamo scesi sotto la soglia DOPO essere saliti sopra, chiudo il giro
    if (z_ < 20 && was_above_threshold && can_force_land) {
        
        can_force_land = false;   // da ora in poi non attivo più landing
        need_land = false;        // smetto di mandare NAV_LAND
        std::cout << "[ForceLand] First cycle completed, disabling force land." << std::endl;
    }
    // se can_force_land è già false, non facciamo più niente
	}


	void land_detected_callback(const px4_msgs::msg::VehicleLandDetected::UniquePtr msg)
	{
		if (msg->landed){
			std::cout << "The drone has landed (VehicleLandDetected.landed = true)"<< std::endl;
			need_land = false;
        	can_force_land = true;
        	was_above_threshold = false;
		}
	}

	void activate_switch()
	{
    // Se devo iniziare il landing
    if (need_land)
    {
        std::cout << "Forcing landing! Sending NAV_LAND..." << std::endl;
        auto command = px4_msgs::msg::VehicleCommand();
        command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
        this->publisher_->publish(command);
        need_land = false;
    }
	}
};

int main(int argc, char *argv[])
{
	std::cout << "Starting vehicle_local_position listener node..." << std::endl;
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ForceLand>());
	rclcpp::shutdown();
	return 0;
}