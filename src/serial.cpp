/* CMAVNode
 * Monash UAS
 *
 * SERIAL CLASS
 * This class extends 'link' and overrides it methods to handle uart communications
 */

#include "serial.h"

serial::serial(const std::string& port, 
        const std::string& baudrate):
    io_service_(), port_(io_service_){

        //open the port with connection string
        port_.open(port);


        if(!port_.is_open()){
            //do something fancy
        }

        //configure the port
        port_.set_option(boost::asio::serial_port_base::baud_rate(std::stoi(baudrate)));
        port_.set_option(boost::asio::serial_port_base::flow_control(
                    boost::asio::serial_port_base::flow_control::none));
        port_.set_option(boost::asio::serial_port_base::parity(
                    boost::asio::serial_port_base::parity::none));
        port_.set_option(boost::asio::serial_port_base::stop_bits(
                    boost::asio::serial_port_base::stop_bits::one));

}

void serial::runWriteThread(){
    //stub
}

void serial::runReadThread(){
    //stub
}
