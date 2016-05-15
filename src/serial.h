/* CMAVNode
 * Monash UAS
 *
 * SERIAL CLASS
 * This class extends 'link' and overrides it methods to handle uart communications
 */
#ifndef SERIAL_H
#define SERIAL_H

#include <string>
#include <boost/asio.hpp>
#include "../include/logging/src/easylogging++.h"

#include "mlink.h"

#define SERIAL_PORT_SLEEP_ON_NOTHING_RECEIVED 2

class serial: public mlink
{
    public:
        //construct and destruct
        serial(const std::string& port,
                const std::string& baudrate,
                int id,
                const std::string &raw);
        ~serial();

        //override virtuals from mlink
        void runWriteThread();
        void runReadThread();

    private:
        //Callbacks for async send/recv
        void handle_receive_from(const boost::system::error_code& error,
                size_t bytes_recvd);
        void handle_send_to(const boost::system::error_code& error,
            size_t bytes_recvd);

        mavlink_message_t getMavMsg();

	boost::asio::io_service io_service_;
        boost::asio::serial_port port_;

        //takes message, puts onto buff and calls send
        void processAndSend(mavlink_message_t *msgToConvert);

        //Actually sends
        void send(uint8_t *buf, std::size_t buf_size);

};

#endif
