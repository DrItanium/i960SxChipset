//
// Created by jwscoggins on 3/10/21.
//

#include <iostream>
#include <boost/asio.hpp>

int main(int argc, char** argv) {
    boost::asio::io_service service;
    boost::asio::serial_port sp(service, "/dev/ttyUSB0");
    return 0;
}