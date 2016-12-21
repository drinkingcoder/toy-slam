#include "System.h"
#include "UDPSocket.h"

int main() {

    udp::socket::startup();

    slam::System system;
    int ret = system.run();

    udp::socket::cleanup();

    return ret;
}
