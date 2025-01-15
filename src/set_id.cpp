#include <rclcpp/rclcpp.hpp>
#include "SCServo.h"

class SetId : public rclcpp::Node
{
public:
    SetId() : Node("set_id")
    {
        this->declare_parameter<int>("start_id", 1);
        this->declare_parameter<int>("new_id", 1);
        this->declare_parameter<std::string>("device_port", "/dev/ttyACM0");
        this->declare_parameter<int>("baud_rate", 1000000);

        start_id_ = static_cast<uint8_t>(this->get_parameter("start_id").as_int());
        new_id_   = static_cast<uint8_t>(this->get_parameter("new_id").as_int());
        device_port_ = this->get_parameter("device_port").as_string();
        baud_rate_   = this->get_parameter("baud_rate").as_int();

        if (!set_id()) 
        {
            RCLCPP_ERROR(this->get_logger(), 
                "Failed to set servo ID from %d to %d", start_id_, new_id_);
        }
    }

private:
    SMS_STS sm_st;
    uint8_t start_id_;
    uint8_t new_id_;
    std::string device_port_;
    int baud_rate_;

    bool set_id()
    {
        if (!sm_st.begin(baud_rate_, device_port_.c_str())) 
        {
            RCLCPP_ERROR(this->get_logger(), 
                "Cannot connect to motor controller on port %s", device_port_.c_str());
            return false;
        }
        if (!sm_st.unLockEprom(start_id_)) 
        {
            RCLCPP_ERROR(this->get_logger(), 
                "Cannot unlock EPROM for servo with ID %d", start_id_);
            sm_st.end();
            return false;
        }
        if (!sm_st.writeByte(start_id_, SMSBL_ID, new_id_)) 
        {
            RCLCPP_ERROR(this->get_logger(), 
                "Cannot set new ID %d for servo with ID %d", new_id_, start_id_);
            sm_st.LockEprom(start_id_);
            sm_st.end();
            return false;
        }
        if (!sm_st.LockEprom(new_id_)) 
        {
            RCLCPP_ERROR(this->get_logger(), 
                "Cannot lock EPROM for servo with ID %d", new_id_);
            sm_st.end();
            return false;
        }
        sm_st.end();
        RCLCPP_INFO(this->get_logger(), 
            "Successfully set servo ID from %d to %d", start_id_, new_id_);
        return true;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin_some(std::make_shared<SetId>());
    rclcpp::shutdown();
    return 0;
}
