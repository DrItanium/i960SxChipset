//
// Created by jwscoggins on 3/10/21.
//

#include <iostream>
#include <boost/asio.hpp>
#include <boost/program_options.hpp>

int main(int argc, char** argv) {
    try {
        boost::asio::io_service service;
        boost::asio::serial_port sp(service, "/dev/ttyUSB0");
        return 0;
    } catch (boost::system::system_error& error) {
       std::cerr << "ERROR: " << error.what() << std::endl;
       std::cerr << "Terminating execution!" << std::endl;
       return 1;
    }
}