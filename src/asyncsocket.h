/* CMAVNode
 * Monash UAS
 *
 * SOCKET CLASS
 * This class extends 'link' and overrides it methods to handle socket communications
 */
#ifndef ASYNCSOCKET_H
#define ASYNCSOCKET_H

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>

#include "mlink.h"

class asyncsocket: public mlink
{
public:
    //Construct specifying all
    asyncsocket(
        const std::string& host,
        const std::string& hostport,
        const std::string& listenport,
        link_info info_);

    //Specify only receive
    asyncsocket(
        const std::string& listenport,
        link_info info_);

    //bcast
    asyncsocket(bool bcastlock,
                const std::string& bindaddress,
                const std::string& bcastaddress,
                const std::string& bcastport,
                link_info info_);

    //Specify only target
    asyncsocket(
        const std::string& host,
        const std::string& hostport,
        link_info info_);

    ~asyncsocket();

    //override virtuals from link
    void runWriteThread();
    void runReadThread();

    // return endpoint corresponding to sender (if any)
    boost::asio::ip::udp::endpoint *sender_endpoint() override {
        return sender_endpoint_;
    }

private:
    //Callbacks for async send/recv
    void handleReceiveFrom(const boost::system::error_code& error,
                           size_t bytes_recvd);
    void handleSendTo(const boost::system::error_code& error,
                      size_t bytes_recvd);

    //UDP Stuff
    boost::asio::io_service io_service_;
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint endpoint_;

    boost::asio::ip::udp::endpoint *sender_endpoint_;

    bool endpointlock = true;

    //takes message, puts onto buff and calls send
    void processAndSend(mavlink_message_t *msgToConvert);

    //Actually sends
    void send(uint8_t *buf, std::size_t buf_size);
    void receive(); //Starts a async receive
};

#endif
