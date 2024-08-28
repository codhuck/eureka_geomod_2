#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <memory>
#include <vector>
#include <boost/asio.hpp>

namespace cwt_npkphcth_reader 
{
    class Cwt_npkphcth_reader: public rclcpp::Node
    {
        public:
            Cwt_npkphcth_reader();
            ~Cwt_npkphcth_reader();

        private:
        void publisher();

        private:

        boost::asio::io_service io;
        std::unique_ptr<boost::asio::serial_port> serial;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publish;
        rclcpp::TimerBase::SharedPtr timer;
        
        float nitrogen;
        float phosphorus;
        float potassium;
        float ph;
        float conductivity;
        float temperature;
        float moisture;
    };

}