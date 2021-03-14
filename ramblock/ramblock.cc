//
// Created by jwscoggins on 3/10/21.
//

#include <iostream>
#include <string>
#include <memory>
#include <array>
#include <boost/asio.hpp>
#include <boost/program_options.hpp>
enum class MaskStyle : uint8_t {
    Error = 0b00,
    LowerEightBits = 0b01,
    UpperEightBits = 0b10,
    FullWord = 0b11,
};
class MemoryBlock {
    using Address = uint32_t;
    using BackingStorage = uint8_t[];
    using ActualBlock = std::unique_ptr<BackingStorage >;
public:
    explicit MemoryBlock(uint32_t size) : theBlock_(std::make_unique<BackingStorage>(size)) {}

    uint16_t
    read(Address address, MaskStyle style) {
        switch (style) {
            case MaskStyle::LowerEightBits:
            case MaskStyle::UpperEightBits:
            case MaskStyle::FullWord:
            default:
                throw std::runtime_error("Illegal memory style");
        }
    }

    void
    write(Address address, uint16_t value, MaskStyle style) {
        switch (style) {
            case MaskStyle::LowerEightBits:
            case MaskStyle::UpperEightBits:
            case MaskStyle::FullWord:
            default:
                throw std::runtime_error("Illegal memory style");
        }
    }
private:
    ActualBlock theBlock_;
};
int main(int argc, char** argv) {
    boost::program_options::options_description desc("Options");
    desc.add_options()
            ("help", "show this help message")
            ("serial-port", boost::program_options::value<std::string>(), "Set the serial port")
            ("baud", boost::program_options::value<uint32_t>(), "Set the baud rate")
            ("flow-control", boost::program_options::value<std::string>()->default_value("none"), "Set flow control kind [none, hardware, software]")
            ("parity", boost::program_options::value<std::string>()->default_value("none"), "Set port parity [even, odd, none]")
            ("stop-bits", boost::program_options::value<double>()->default_value(1.0), "set number of stop bits [1, 1.5, 2]")
            ("character-size", boost::program_options::value<uint32_t>()->default_value(8), "Set the character size (default 8)")
            ("memory-block-size", boost::program_options::value<uint32_t>()->default_value(1024*1024), "Set the size of the memory block in bytes");



    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }
    if (!vm.count("serial-port")) {
        std::cout << "Must specify serial port!" << std::endl;
        std::cout << desc << std::endl;
        return 1;
    } else {
        std::string serialPortName(vm["serial-port"].as<std::string>());
        auto getFlowControl = [](const std::string& value) {
            if (value == "none") {
                return boost::asio::serial_port::flow_control::none;
            } else if (value == "hardware") {
                return boost::asio::serial_port::flow_control::hardware;
            } else if (value == "software") {
                return boost::asio::serial_port::flow_control::software;
            } else {
                throw std::runtime_error("unknown flow control kind");
            }
        };
        auto getStopBits = [](double value) {
           if (value == 1.0) {
                return boost::asio::serial_port::stop_bits::type::one;
           } else if (value == 1.5) {
               return boost::asio::serial_port::stop_bits::type::onepointfive;
           } else if (value == 2) {
               return boost::asio::serial_port::stop_bits::type::two;
           } else {
               throw std::runtime_error("unknown stop bit size");
           }
        };
        auto getParity = [](const std::string& value) {
            if (value == "even") {
                return boost::asio::serial_port::parity::type::even;
            } else if (value == "odd") {
                return boost::asio::serial_port::parity::type::odd;
            } else if (value == "none") {
                return boost::asio::serial_port::parity::type::none;
            } else {
                throw std::runtime_error("unknown parity kind");
            }
        };
        try {
            boost::asio::io_service service;
            boost::asio::serial_port sp(service);
            sp.set_option(boost::asio::serial_port::baud_rate(vm["baud"].as<uint32_t>()));
            sp.set_option(boost::asio::serial_port::flow_control(getFlowControl(vm["flow-control"].as<std::string>())));
            sp.set_option(boost::asio::serial_port::character_size(vm["character-size"].as<uint32_t>()));
            sp.set_option(boost::asio::serial_port::stop_bits(getStopBits(vm["stop-bits"].as<double>())));
            sp.set_option(boost::asio::serial_port::parity(getParity(vm["parity"].as<std::string>())));
            MemoryBlock mb(vm["memory-block-size"].as<uint32_t>());
            boost::system::error_code ec;
            sp.open(serialPortName.c_str(), ec);
            if (ec) {
                std::cerr << "error: opening port " << serialPortName << " failed. Reason: " << ec.message() << std::endl;
                return 1;
            }
            std::array<uint8_t, 8> data = { 0 };
            while (true) {
                auto count = boost::asio::read(sp, boost::asio::buffer(data, 8));
                if (count != 8) {
                    throw std::runtime_error("not enough bytes!");
                } else {
                    auto operation = static_cast<uint16_t>(data[0])  | (static_cast<uint16_t>(data[1]) << 8);
                    auto style = static_cast<MaskStyle>(operation & 0b11);
                    auto address = static_cast<uint32_t>(data[2]) |
                                   (static_cast<uint32_t>(data[3]) << 8) |
                                   (static_cast<uint32_t>(data[4]) << 16) |
                                   (static_cast<uint32_t>(data[5]) << 24);
                    uint16_t result = 0;
                    if (operation & 0b100) {
                        // write operation
                        auto value = static_cast<uint16_t>(data[6])  | (static_cast<uint16_t>(data[7]) << 8);
                        mb.write(address, value, style);
                    } else {
                        // read operation
                        result = mb.read(address, style);
                    }
                    sp.write_some(boost::asio::buffer(&result, 2));
                }
            }
            return 0;
        } catch (boost::system::system_error &error) {
            std::cerr << "ERROR: " << error.what() << std::endl;
            std::cerr << "Terminating execution!" << std::endl;
            return 1;
        } catch (std::runtime_error& error) {
            std::cerr << "ERROR: " << error.what() << std::endl;
            return 1;
        }
    }
}