/* CMAVNode
 * Monash UAS
 *
 * SERIAL CLASS
 * This class extends 'link' and overrides it methods to handle uart communications
 */

#include "serial.h"

serial::serial(const std::string& port,
               const std::string& baudrate,
               link_info info_):
    io_service_(), port_(io_service_), mlink(info_)
{



    try
    {
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

    }
    catch (boost::system::system_error &error)
    {
        LOG(ERROR) << "Error opening Serial Port: " << port;
        throw Exception("Error opening serial port");
    }

    //Start the read and write threads
    write_thread = boost::thread(&serial::runWriteThread, this);

    //Start the receive
    port_.async_read_some(
        boost::asio::buffer(data_in_, MAV_INCOMING_BUFFER_LENGTH),
        boost::bind(&serial::handleReceiveFrom, this,
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
    port_.close();
}

void serial::send(uint8_t *buf, std::size_t buf_size)
{
    port_.async_write_some(
        boost::asio::buffer(buf, buf_size),
        boost::bind(&serial::handleSendTo, this,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
}

void serial::processAndSend(mavlink_message_t *msgToConvert)
{
    //pack into buf and get size_t
    uint8_t tmplen = mavlink_msg_to_send_buffer(data_out_, msgToConvert);
    //ERROR HANDLING?

    bool should_drop = shouldDropPacket();
    //send on socket
    send(data_out_, tmplen);
}

//Async post send callback
void serial::handleSendTo(const boost::system::error_code& error,
                          size_t bytes_recvd)
{
    if (!error && bytes_recvd > 0)
    {
        //Everything was ok
    }
    else
    {
        //There was an error

        if(errorcount++ > SERIAL_PORT_MAX_ERROR_BEFORE_KILL)
        {
            is_kill = true;
            LOG(INFO) << "Link " << info.link_name << " is dead";
        }
    }
}

//Async callback receiver
void serial::handleReceiveFrom(const boost::system::error_code& error,
                               size_t bytes_recvd)
{
    if (!error & bytes_recvd > 0)
    {
        //message received
        //do something
        mavlink_message_t msg;
        mavlink_status_t status;

        for (size_t i = 0; i < bytes_recvd; i++)
        {
            if (mavlink_parse_char(MAVLINK_COMM_0, data_in_[i], &msg, &status))
            {
                if ((record_incoming_packet() == false &&
                    info.packet_drop_enable) ||
                    shouldDropPacket())
                {
                    // Repeated packet - don't process it further
                    continue;
                }

                onMessageRecv(&msg);

                // Update the packet sequence for outgoing packets
                msg.seq = link_quality.out_packet_sequence++;

                // Try to push it onto the queue
                bool returnCheck = qMavIn.push(msg);

                if(!returnCheck)   //then the queue is full
                {
                    throw Exception("Serial: The incoming message queue is full");
                }
            }
        }

        //And start reading again
        port_.async_read_some(
            boost::asio::buffer(data_in_, MAV_INCOMING_BUFFER_LENGTH),
            boost::bind(&serial::handleReceiveFrom, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
    }
    else if(bytes_recvd == 0)
    {
        //Sleep a little bit to keep the cpu cool
        boost::this_thread::sleep(boost::posix_time::milliseconds(SERIAL_PORT_SLEEP_ON_NOTHING_RECEIVED));
        port_.async_read_some(
            boost::asio::buffer(data_in_, MAV_INCOMING_BUFFER_LENGTH),
            boost::bind(&serial::handleReceiveFrom, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
    }
    else
    {
        //we have an error
        //need to look into what is causing these but for now just pretend it didn't happen
        boost::this_thread::sleep(boost::posix_time::milliseconds(SERIAL_PORT_SLEEP_ON_NOTHING_RECEIVED));
        port_.async_read_some(
            boost::asio::buffer(data_in_, MAV_INCOMING_BUFFER_LENGTH),
            boost::bind(&serial::handleReceiveFrom, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
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
            while(qMavOut.pop(tmpMsg))
            {
                processAndSend(&tmpMsg);
            }
        }
        else
        {
            //queue is empty sleep the write thread
            boost::this_thread::sleep(boost::posix_time::milliseconds(OUT_QUEUE_EMPTY_SLEEP));
        }
    }
}
