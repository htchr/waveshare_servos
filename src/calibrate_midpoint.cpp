#include <rclcpp/rclcpp.hpp>
#include "SCServo.h"

class CalibMid : public rclcpp::Node
{
public:
    CalibMid() : Node("calibrate_midpoint")
    {
        this->declare_parameter<int>("id", 1);
        this->declare_parameter<std::string>("device_port", "/dev/ttyACM0");
        this->declare_parameter<int>("baud_rate", 1000000);

        id_ = static_cast<uint8_t>(this->get_parameter("id").as_int());
        device_port_ = this->get_parameter("device_port").as_string();
        baud_rate_   = this->get_parameter("baud_rate").as_int();

        if (!calib_mid())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set midpoint for servo ID %d", id_);
        }
    }

private:
    SMS_STS sm_st;
    uint8_t id_;
    std::string device_port_;
    int baud_rate_;

    bool calib_mid()
    {
        if (!sm_st.begin(baud_rate_, device_port_.c_str())) 
        {
            RCLCPP_ERROR(this->get_logger(), 
                "Cannot connect to motor controller on port %s", device_port_.c_str());
            return false;
        }
        if (!sm_st.Mode(id_, 0))
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot set servo ID %d mode", id_);
        }
        if (!sm_st.CalibrationOfs(id_))
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot calibrate midpoint of motor ID %d", id_);
        }
        sm_st.end();
        return true;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin_some(std::make_shared<CalibMid>());
    rclcpp::shutdown();
    return 0;
}
