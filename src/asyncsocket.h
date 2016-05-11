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
        //Construct
        asyncsocket(
                const std::string& host,
                const std::string& hostport,
                const std::string& listenport);

        ~asyncsocket();

        //override virtuals from link
        void runWriteThread();
        void runReadThread();


    private:
        //Callbacks for async send/recv
        void handle_receive_from(const boost::system::error_code& error,
                size_t bytes_recvd);
        void handle_send_to(const boost::system::error_code& error,
            size_t bytes_recvd);

        //UDP Stuff
	boost::asio::io_service io_service_;
        boost::asio::ip::udp::socket socket_;
        boost::asio::ip::udp::endpoint endpoint_;

        //takes message, puts onto buff and calls send
        void processAndSend(mavlink_message_t *msgToConvert);

        //Actually sends
        void send(uint8_t *buf, std::size_t buf_size);
};

#endif
