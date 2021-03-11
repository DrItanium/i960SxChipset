//
// Created by jwscoggins on 3/10/21.
//

#include <iostream>
#include <boost/asio/serial_port.hpp>

int main(int argc, char** argv) {
    boost::asio::serial_port sp("foo");
    return 0;
}