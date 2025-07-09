#include <iostream>
#include <Poco/Net/StreamSocket.h>
#include <Poco/Net/SocketAddress.h>
#include <Poco/Exception.h>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <ip-address> <torque>" << std::endl;
        return 1;
    }

    try {
        Poco::Net::SocketAddress sa(argv[1], 12345);
        Poco::Net::StreamSocket socket(sa);

        std::string torque_str = argv[2];
        socket.sendBytes(torque_str.c_str(), torque_str.length());

        std::cout << "Sent torque: " << torque_str << std::endl;

    } catch (Poco::Exception& exc) {
        std::cerr << "Poco error: " << exc.displayText() << std::endl;
        return 1;
    }

    return 0;
} 