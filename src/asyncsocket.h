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

struct udp_properties{
    std::string host; //functions as bcastaddress if broadcast
    int hostport = -1; //functions as bcastport if broadcast
    int bindport = -1;

    // 0 = fully defined, 1 = client, 2 = server, 3 = bcastlock 4 = bcast TODO: make enum
    int udp_type = 0;
};

class asyncsocket: public mlink
{
public:
    //Construct specifying all
    asyncsocket(udp_properties properties_,int link_id_, LinkOptions info_);

    ~asyncsocket();

    // real construction happens here
    void createFullyDefined();
    void createClient();
    void createServer();
    void createBroadcast();


    //override virtuals from link
    void runWriteThread();
    void runReadThread();

    // return endpoint corresponding to sender (if any)
    boost::asio::ip::udp::endpoint *sender_endpoint() override
    {
        return sender_endpoint_;
    }

    udp_properties properties;

private:
    //Callbacks for async send/recv
    void handleReceiveFrom(const boost::system::error_code& error,
                           size_t bytes_recvd);
    void handleSendTo(const boost::system::error_code& error,
                      size_t bytes_recvd);

    //UDP Stuff
    boost::asio::io_service io_service_;
    std::unique_ptr<boost::asio::ip::udp::socket> socket_;
    boost::asio::ip::udp::endpoint endpoint_;

    boost::asio::ip::udp::endpoint *sender_endpoint_;

    bool endpointlock = true;

    //takes message, puts onto buff and calls send
    void processAndSend(mavlink_message_t *msgToConvert);

    //Actually sends
    void send(uint8_t *buf, std::size_t buf_size);
    void receive(); //Starts a async receive

    void prep(const std::string& host, int hostport);
};

#endif
