#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

class PointCloud : public rclcpp::Node {
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    
    public:
    PointCloud() : Node("lidar_node") {
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization/point", 10);
        subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
                "vision/point", 1, 
                [this](geometry_msgs::msg::PointStamped::SharedPtr ps) {
                    publisher_->publish(convert(*ps));
                });
    }

    private:
    /*
     * Convert geometry_msgs::msg::PointStamped to visualization_msgs::msg::Marker
     * i.e. turn a geometric point into a visual representation on RViz
     */
    visualization_msgs::msg::Marker convert(geometry_msgs::msg::PointStamped &ps) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = ps.header.frame_id;
        marker.ns = "";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = ps.point.x;
        marker.pose.position.y = ps.point.y;
        marker.pose.position.z = ps.point.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 2;
        marker.scale.y = 2;
        marker.scale.z = 2;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        return marker;
    }

    // create an actual point to pass into the publisher
    void point_callback() {
        geometry_msgs::msg::PointStamped ps;
        ps.header.frame_id = "base_scan";
        ps.point.x = 100;
        ps.point.y = 100;
        ps.point.z = 100;

        publisher_->publish(this->convert(ps));
    }
};

int main(int argc, char ** argv)
{
    printf("LiDAR package activated!\n");
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloud>());
    rclcpp::shutdown();

    return 0;
}
