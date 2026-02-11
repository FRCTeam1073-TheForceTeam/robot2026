#include <utilities/UdpChannel.h>
#include <iostream>
#include <sys/ioctl.h>


UDPChannel::UDPChannel(int port) : m_fd(-1) {

    // Create a datagram socket:
    m_fd = socket(AF_INET, SOCK_DGRAM, 0);

    if (m_fd != -1) {
        // Change its behavior to be non-blocking:
        int flags = fcntl(m_fd, F_GETFL, 0);
        if (flags == -1) {
            perror("UDPListener:: fcntl F_GETFL");
            // handle error
        }
        if (fcntl(m_fd, F_SETFL, flags | O_NONBLOCK) == -1) {
            perror("UDPListener:: fcntl F_SETFL O_NONBLOCK");
            // handle error
        }

        // Clear last packet address to zeros:
        memset(&m_packet_address, 0, sizeof(m_packet_address));

        // Temporary for socket binding:
        sockaddr_in servaddr;

        // Initialize server address structure
        memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;         // IPv4
        servaddr.sin_addr.s_addr = INADDR_ANY; // Listen on all available interfaces
        servaddr.sin_port = htons(port);       // Host to network short

        // Bind the socket to the server address and port
        if (bind(m_fd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
            perror("UDPListener:: bind failed");
            close(m_fd);
            m_fd = -1;
        } else {
            std::cout << "UDPListener nonblocking socket bound to port " << port << std::endl;
        }

    } else {
        std::cerr << "UDPListener:: failed to create socket" << std::endl;
    }
}

UDPChannel::~UDPChannel() {
    if (m_fd != -1) {
        close(m_fd);
    }
}

bool UDPChannel::has_data() {
    int bytes_available = 0;
    if (m_fd < 0) return false;

    if (ioctl(m_fd, FIONREAD, &bytes_available) == -1) {
        // Handle error, check errno for specifics
        perror("UDPChannel:: ioctl FIONREAD failed");
        return false;
    } else {
        if (bytes_available > 0) {
            return true;
        } else {
            return false;
        }
    }
}

int UDPChannel::read(uint8_t *data, size_t max_len) {
    if (m_fd < 0)  {
        std::cerr << "UDPListener: socket not open" << std::endl;
        return 0;
    }
    
    socklen_t addrlen = sizeof(m_packet_address); // The size of address data.
    int n = recvfrom(m_fd, data, max_len, 0, (sockaddr *)&m_packet_address, &addrlen);

    // If there's nothing there but it is not an expected case, then indicate error.
    if (n < 0) {
     if (errno != EWOULDBLOCK && errno != EAGAIN) {
            perror("UDPListener:: "); // THere is an actual problem.
        } else {
            n = 0; // We read nothing.
        }
    }

    return n;
}

sockaddr_in UDPChannel::last_address() const {
    return m_packet_address;
}

int UDPChannel::send_to(uint8_t *data, size_t data_len, sockaddr_in& destination) {
    if (m_fd < 0) {
        std::cerr << "UDPChannel: socket not open" << std::endl;
    }
    int result = sendto(m_fd, data, data_len, 0, (sockaddr *) &destination, sizeof(sockaddr_in));
    if (result < 0) {
        perror("UDPChannel:");
    }
    return result;
}

sockaddr_in UDPChannel::create_address(std::string& address, int port) {
    sockaddr_in result;
    // Clear the result:
    memset(&result, 0, sizeof(struct sockaddr_in));

    result.sin_family = AF_INET; // IP socket.
    result.sin_port = htons(port); // Convert port to network byte order

    if (inet_pton(AF_INET, address.c_str(), &(result.sin_addr)) <= 0) {
        std::cerr << "Could not convert address: " << address << std::endl;
        memset(&result, 0, sizeof(struct sockaddr_in));
    }

    return result;
}
