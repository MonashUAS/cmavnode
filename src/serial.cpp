/* CMAVNode
 * Monash UAS
 *
 * SERIAL CLASS
 * This class extends 'link' and overrides it methods to handle uart communications
 */

#include "serial.h"
#include "../include/logging/src/easylogging++.h"

serial::serial(const std::string& port, 
        const std::string& baudrate,
        int id,
        const std::string& raw):
    io_service_(), port_(io_service_){


        //store link info into the base class
        linkID = id;
        rawString = raw;

        LOG(INFO) << "Link " << linkID << " - opening with connection string: " << rawString;
        
        try{
            //open the port with connection string
            port_.open(port);


            //configure the port
            port_.set_option(boost::asio::serial_port_base::baud_rate((unsigned int)std::stoi(baudrate)));

            port_.set_option(boost::asio::serial_port_base::character_size(8));

            port_.set_option(boost::asio::serial_port_base::flow_control(
                        boost::asio::serial_port_base::flow_control::none));

            port_.set_option(boost::asio::serial_port_base::parity(
                        boost::asio::serial_port_base::parity::none));

            port_.set_option(boost::asio::serial_port_base::stop_bits(
                        boost::asio::serial_port_base::stop_bits::one));

        } catch (boost::system::system_error &error) {
            LOG(ERROR) << "Error opening Serial Port: " << port;
            throw Exception("Error opening serial port");
        }

        //Start the read and write threads
        write_thread = boost::thread(&serial::runWriteThread, this);

        //Start the receive
        port_.async_read_some(
                boost::asio::buffer(data_in_, MAV_INCOMING_BUFFER_LENGTH), 
                    boost::bind(&serial::handle_receive_from, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));

        read_thread = boost::thread(&serial::runReadThread, this);
}

serial::~serial()
{
    //Force run() to return then join thread
    io_service_.stop();
    read_thread.join();

    //force write thread to return then join thread
    exitFlag = true;
    write_thread.join();

    //Debind
    std::cout << "Serial: Socket Closed" << std::endl;
    port_.close();
}

void serial::send(uint8_t *buf, std::size_t buf_size) {
            port_.async_write_some(
                    boost::asio::buffer(buf, buf_size),
                    boost::bind(&serial::handle_send_to, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
}

void serial::processAndSend(mavlink_message_t *msgToConvert)
{
    //pack into buf and get size_t
    uint8_t tmplen = mavlink_msg_to_send_buffer(data_out_, msgToConvert);
    //ERROR HANDLING?

    //send on socket
    send(data_out_, tmplen);
}

//Async post send callback
void serial::handle_send_to(const boost::system::error_code& error,
    size_t bytes_recvd)
{
    if (!error && bytes_recvd > 0)
    {
        //Everything was ok
    } else
    {
        //There was an error
        throw Exception("Serial: Error in handle_send_to");
    }
}

//Async callback receiver
void serial::handle_receive_from(const boost::system::error_code& error,
    size_t bytes_recvd)
{
    if (!error & bytes_recvd > 0)
    {
        //message received
        //do something
        mavlink_message_t msg;
        mavlink_status_t status;
        unsigned int temp;

        for (size_t i = 0; i < bytes_recvd; i++)
        {
                temp = data_in_[i];
                if (mavlink_parse_char(MAVLINK_COMM_0, data_in_[i], &msg, &status))
                {
                    // Packet received
                    // if the above condition is true the packet must be complete
                    if(msg.msgid == MAVLINK_MSG_ID_HEARTBEAT){
                        //if this is a heartbeat, update the mapping
                        onHeartbeatRecv(msg.sysid);
                    }
                    //Try to push it onto the queue
                    bool returnCheck = qMavIn.push(msg);
                    if(!returnCheck) { //then the queue is full
                       throw Exception("Serial: The incoming message queue is full"); 
                    }
                }
        }

        //And start reading again
        port_.async_read_some(
                boost::asio::buffer(data_in_, MAV_INCOMING_BUFFER_LENGTH), 
                    boost::bind(&serial::handle_receive_from, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
    } else
    {
        //work out why this is throwing errors
        //nothing received or there was an error
        port_.async_read_some(
                boost::asio::buffer(data_in_, MAV_INCOMING_BUFFER_LENGTH), 
                    boost::bind(&serial::handle_receive_from, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
       // throw Exception("Serial: Error in handle_receive_from");
    }
}

void serial::runReadThread()
{
    //gets run in thread
    //Because io_service.run() will block while socket is open
    io_service_.run();
}

void serial::runWriteThread()
{
    //block so we dont send before the socket is initialized
    boost::this_thread::sleep(boost::posix_time::milliseconds(50));
    //busy wait on the spsc_queue
    mavlink_message_t tmpMsg;

    //thread loop
    while(!exitFlag)
    {
        if(qMavOut.pop(tmpMsg))
        {
            processAndSend(&tmpMsg);
            //keep going
            while(qMavOut.pop(tmpMsg)){
                    processAndSend(&tmpMsg);
            }
        } else {
            //queue is empty sleep the write thread
            boost::this_thread::sleep(boost::posix_time::milliseconds(OUT_QUEUE_EMPTY_SLEEP));
        }
    }
}
