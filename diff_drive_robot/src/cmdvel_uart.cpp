#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <boost/asio.hpp>
#include <iostream>
#include <sstream>
#include <vector>

using std::placeholders::_1;

class STM32Driver : public rclcpp::Node
{
public:
    STM32Driver() : Node("stm32_driver"), io(), serial(io)
    {
        // ===== PARAM =====
        this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        this->declare_parameter<int>("baudrate", 115200);

        std::string port = this->get_parameter("port").as_string();
        int baud = this->get_parameter("baudrate").as_int();

        // ===== OPEN SERIAL =====
        try
        {
            serial.open(port);
            serial.set_option(boost::asio::serial_port_base::baud_rate(baud));
            RCLCPP_INFO(this->get_logger(), "Connected to %s", port.c_str());
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot open serial port!");
            rclcpp::shutdown();
        }

        // ===== TF BROADCASTER =====
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // ===== SUBSCRIBE CMD_VEL =====
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&STM32Driver::cmdCallback, this, _1));

        // ===== PUBLISH ODOM RAW =====
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_raw", 10);

        // ===== TIMER READ SERIAL =====
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&STM32Driver::readSerial, this));
    }

private:
    // ===== ROS =====
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ===== SERIAL =====
    boost::asio::io_service io;
    boost::asio::serial_port serial;

    std::string buffer_;

    // ===== SEND CMD_VEL =====
    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double v = msg->linear.x;
        double w = msg->angular.z;

        char buffer[64];
        snprintf(buffer, sizeof(buffer), "%.3f,%.3f\n", v, w);

        try
        {
            boost::asio::write(serial, boost::asio::buffer(buffer));
        }
        catch (...)
        {
            RCLCPP_WARN(this->get_logger(), "Write UART failed");
        }
    }

    // ===== READ SERIAL =====
    void readSerial()
    {
        char buf[256];

        try
        {
            size_t n = serial.read_some(boost::asio::buffer(buf));
            buffer_.append(buf, n);

            size_t pos;
            while ((pos = buffer_.find('\n')) != std::string::npos)
            {
                std::string line = buffer_.substr(0, pos);
                buffer_.erase(0, pos + 1);

                parseOdom(line);
            }
        }
        catch (...)
        {
            RCLCPP_WARN(this->get_logger(), "Read UART failed");
        }
    }

    // ===== PARSE STM32 DATA =====
    void parseOdom(const std::string &line)
    {
        // FORMAT: x,y,theta,v,w
        std::stringstream ss(line);
        std::string item;
        std::vector<double> data;

        while (std::getline(ss, item, ','))
        {
            try
            {
                data.push_back(std::stod(item));
            }
            catch (...)
            {
                return;
            }
        }

        if (data.size() != 5)
            return;

        double x = data[0];
        double y = data[1];
        double theta = data[2];
        double v = data[3];
        double w = data[4];

        auto now = this->get_clock()->now();

        // ===== QUATERNION =====
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);

        // ===== ODOM MSG =====
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf2::toMsg(q);

        odom.twist.twist.linear.x = v;
        odom.twist.twist.angular.z = w;

        // Covariance (basic tuning)
        odom.pose.covariance[0] = 0.05;
        odom.pose.covariance[7] = 0.05;
        odom.pose.covariance[35] = 0.1;

        odom.twist.covariance[0] = 0.05;
        odom.twist.covariance[35] = 0.1;

        odom_pub_->publish(odom);

        // ===== TF BROADCAST =====
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";

        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = 0.0;
        t.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(t);

        // ===== DEBUG =====
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Odom: x=%.2f y=%.2f th=%.2f",
            x, y, theta);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<STM32Driver>());
    rclcpp::shutdown();
    return 0;
}