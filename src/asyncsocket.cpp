/* CMAVNode
 * Monash UAS
 *
 * SOCKET CLASS
 * This class extends 'link' and overrides it methods to handle socket communications
 */

#include "asyncsocket.h"

asyncsocket::asyncsocket(
        const std::string& host, 
        const std::string& hostport,
        const std::string& listenport,
        int id,
        const std::string& raw) : io_service_(),
    socket_(io_service_, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), std::stoi(listenport))) {

            boost::asio::ip::udp::resolver resolver(io_service_);
            boost::asio::ip::udp::resolver::query query(boost::asio::ip::udp::v4(), host, hostport);
            boost::asio::ip::udp::resolver::iterator iter = resolver.resolve(query);
            endpoint_ = *iter;

            linkID = id;
            rawString = raw;

            LOG(INFO) << "Link " << linkID << " - opening with connection string: " << rawString;

            //Start the read and write threads
            write_thread = boost::thread(&asyncsocket::runWriteThread, this);

            //Start the receive
            socket_.async_receive_from(
                    boost::asio::buffer(data_in_, MAV_INCOMING_BUFFER_LENGTH), endpoint_, 
                        boost::bind(&asyncsocket::handle_receive_from, this,
                            boost::asio::placeholders::error,
                            boost::asio::placeholders::bytes_transferred));
            
            read_thread = boost::thread(&asyncsocket::runReadThread, this);
}

asyncsocket::~asyncsocket()
{
    //Force run() to return then join thread
    io_service_.stop();
    read_thread.join();

    //force write thread to return then join thread
    exitFlag = true;
    write_thread.join();

    //Debind
    socket_.close();
    LOG(INFO) << "Link " << linkID << " - closing, connection string: " << rawString;
}

void asyncsocket::send(uint8_t *buf, std::size_t buf_size) {
            socket_.async_send_to(
                    boost::asio::buffer(buf, buf_size), endpoint_,
                    boost::bind(&asyncsocket::handle_send_to, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
}

void asyncsocket::processAndSend(mavlink_message_t *msgToConvert)
{
    //pack into buf and get size_t
    uint8_t tmplen = mavlink_msg_to_send_buffer(data_out_, msgToConvert);
    //ERROR HANDLING?

    //send on socket
    send(data_out_, tmplen);
}



//Async callback receiver
void asyncsocket::handle_receive_from(const boost::system::error_code& error,
    size_t bytes_recvd)
{
    if (!error && bytes_recvd > 0)
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
                    onMessageRecv(&msg);
                    //Try to push it onto the queue
                    bool returnCheck = qMavIn.push(msg);
                    if(!returnCheck) { //then the queue is full
                       throw Exception("MAVLink_AL: The incoming message queue is full"); 
                    }
                }
        }

        //And start reading again
        socket_.async_receive_from(
                boost::asio::buffer(data_in_, MAV_INCOMING_BUFFER_LENGTH), endpoint_, 
                    boost::bind(&asyncsocket::handle_receive_from, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
    } else
    {
        //nothing received or there was an error
        throw Exception("UDPClient: Error in handle_receive_from");
    }
}

//Async post send callback
void asyncsocket::handle_send_to(const boost::system::error_code& error,
    size_t bytes_recvd)
{
    if (!error && bytes_recvd > 0)
    {
        //Everything was ok
    } else
    {
        //There was an error
        throw Exception("UDPClient: Error in handle_send_to");
    }
}

void asyncsocket::runReadThread()
{
    //gets run in thread
    //Because io_service.run() will block while socket is open
    io_service_.run();
}

void asyncsocket::runWriteThread()
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
