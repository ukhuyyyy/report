#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include <string>
#include <sstream>
#include <vector>
#include <cmath>

class ImuNode : public rclcpp::Node
{
public:
    ImuNode() : Node("imu_node")
    {
        // ===== PARAMETERS =====
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baudrate", 115200);

        port_ = this->get_parameter("port").as_string();
        baudrate_ = this->get_parameter("baudrate").as_int();

        // ===== SERIAL INIT =====
        fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY);

        if (fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot open serial port");
            rclcpp::shutdown();
            return;
        }

        struct termios tty;
        tcgetattr(fd_, &tty);

        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_iflag = 0;

        tcsetattr(fd_, TCSANOW, &tty);

        RCLCPP_INFO(this->get_logger(), "Connected to %s", port_.c_str());

        // ===== PUBLISHER =====
        pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

        // ===== TIMER =====
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ImuNode::readSerial, this));
    }

private:
    int fd_;
    std::string port_;
    int baudrate_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string buffer_;

    void readSerial()
    {
        char buf[256];
        int n = read(fd_, buf, sizeof(buf));

        if (n > 0)
        {
            buffer_.append(buf, n);

            size_t pos;
            while ((pos = buffer_.find('\n')) != std::string::npos)
            {
                std::string line = buffer_.substr(0, pos);
                buffer_.erase(0, pos + 1);

                parseLine(line);
            }
        }
    }

    void parseLine(const std::string &line)
    {
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

        if (data.size() != 6)
            return;

        double ax = data[0];
        double ay = data[1];
        double az = data[2];

        double gx = data[3];
        double gy = data[4];
        double gz = data[5];

        // ===== Nếu gyro là deg/s → đổi sang rad/s =====
        gx *= M_PI / 180.0;
        gy *= M_PI / 180.0;
        gz *= M_PI / 180.0;

        // ===== Nếu accel là g → đổi sang m/s^2 =====
        ax *= 9.81;
        ay *= 9.81;
        az *= 9.81;

        sensor_msgs::msg::Imu msg;

        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "imu_link";

        // ===== ACCEL =====
        msg.linear_acceleration.x = ax;
        msg.linear_acceleration.y = ay;
        msg.linear_acceleration.z = az;

        // ===== GYRO =====
        msg.angular_velocity.x = gx;
        msg.angular_velocity.y = gy;
        msg.angular_velocity.z = gz;

        // ===== ORIENTATION (chưa có) =====
        msg.orientation.w = 1.0;

        // ===== COVARIANCE =====
        msg.linear_acceleration_covariance = {
            0.1, 0, 0,
            0, 0.1, 0,
            0, 0, 0.1};

        msg.angular_velocity_covariance = {
            0.1, 0, 0,
            0, 0.1, 0,
            0, 0, 0.1};

        msg.orientation_covariance = {
            99999, 0, 0,
            0, 99999, 0,
            0, 0, 99999};

        pub_->publish(msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuNode>());
    rclcpp::shutdown();
    return 0;
}