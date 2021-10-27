
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>


const std::string BASE_FRAME = std::string{"base_link"};
const double MAX_SIDE_LIMIT = 0.50;
const double MIN_APPROACH_DIST = 0.30;
const double MAX_APPROACH_DIST = 0.50;

const double ROBOT_RADIUS = 0.20;

const double MAX_SPEED = 0.25;
const double MAX_TURN = 1.0;


class WallFollower : public rclcpp::Node
{
	public:
		explicit WallFollower();

		~WallFollower(){}

		void configure();
		void startup();
		void shutdown();

	private:

		geometry_msgs::msg::TransformStamped transform;
		bool paused, stopped;
		
		enum Side {LEFT, RIGHT};
		Side side;
		

		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr commandSub_;
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSub_;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twistPub_;

		void callbackScan(const sensor_msgs::msg::LaserScan::SharedPtr scan);
		void callbackControl(const std_msgs::msg::String::SharedPtr command);
};
