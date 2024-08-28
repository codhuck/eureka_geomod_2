#include <eureka_geomod_2/cwt_npkphcth_reader.hpp>

namespace cwt_npkphcth_reader
{
  Cwt_npkphcth_reader::Cwt_npkphcth_reader()
  : rclcpp::Node("cwt_npkphcth_reader"),
  nitrogen(0.0),
  phosphorus(0.0),
  potassium(0.0),
  ph(0.0),
  conductivity(0.0),
  temperature(0.0),
  moisture(0.0)
  {
        try {
        boost::asio::io_service io_service;
        serial = std::make_unique<boost::asio::serial_port>(io_service, "/dev/needle");
        serial->set_option(boost::asio::serial_port_base::baud_rate(4800));
        RCLCPP_INFO(this->get_logger(), "Serial port /dev/needle opened");
    } catch (const boost::system::system_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Unable to open port: %s", e.what());
        throw e;
    }
    publish=this->create_publisher<sensor_msgs::msg::JointState>("needle", 10);
    timer=this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&Cwt_npkphcth_reader::publisher, this));
      RCLCPP_INFO(this->get_logger(),"cwt_npkphcth_reader Started!") ;
    }

    Cwt_npkphcth_reader::~Cwt_npkphcth_reader(){
      RCLCPP_INFO(this->get_logger(),"cwt_npkphcth_reader Killed!");
    }

    void Cwt_npkphcth_reader::publisher()
    {
      sensor_msgs::msg::JointState message;
      std::vector<uint8_t> devs={0x01,0x03,0x00,0x00,0x00,0x07,0x04,0x08};
      boost::asio::write(*serial, boost::asio::buffer(devs));
      std::vector<uint8_t> response(19);
      boost::asio::read(*serial, boost::asio::buffer(response));
      if (response.size()==19)
      {
        moisture=static_cast<float>((response[3] << 8 | response[4])/10.0);
        temperature=static_cast<float>((response[5] << 8 | response[6])/10.0);
        conductivity=static_cast<float>((response[7] << 8 | response[8])/10.0);
        ph=static_cast<float>((response[9] << 8 | response[10])/10.0);
        nitrogen=static_cast<float>((response[11] << 8 | response[12])/10.0);
        phosphorus=static_cast<float>((response[13] << 8 | response[14])/10.0);
        potassium=static_cast<float>((response[15] << 8 | response[16])/10.0);
        message.name={"nitrogen", "phosphorus", "potassium", "ph", "conductivity", "soil_temperature", "moisture"};
        message.position={nitrogen, phosphorus, potassium, ph, conductivity, temperature, moisture};
        publish->publish(message);
      } 
    }

  }

  int main(int argc, char** argv)
  {
    rclcpp::init(argc, argv);


    std::shared_ptr<cwt_npkphcth_reader::Cwt_npkphcth_reader> node = std::make_shared<cwt_npkphcth_reader::Cwt_npkphcth_reader>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    return 0;
  }