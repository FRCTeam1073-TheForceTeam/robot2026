#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>


/**
 * This class provides a bidirectional, nonblocking UDP packet
 * channel. It allows sending packets to various addresses
 * and non-blocking / polled reception of packets as well.
 * 
 * Constructor argument takes a port number. Beware that
 * FRC has restricted ports for user code, check the game
 * manual!
 * 
 * Note that UDP packets can be variably sized, but generally
 * should not be larger than the MTU of the underlying network
 * this is typically something around 1400 bytes.
 * 
 */
class UDPChannel {
    public:

        /// Constructor takes a port agument and binds to address 0.0.0.0
        UDPChannel(int port = 5800);

        /// Destructor closes socket.
        ~UDPChannel();

        /// Is there data to read?
        bool has_data();

        /// Read whatever data we can. Does not block. Tells us how much we actually got (up to max)
        int read(uint8_t *data, size_t max_len);

        /// Source address of last packet we read.
        sockaddr_in last_address() const;

        /// Send a block of bytes to an address:
        int send_to(uint8_t *data, size_t max_len, sockaddr_in& destination);

        /// Create sockaddr_in for this channel from a string format:
        static sockaddr_in create_address(std::string& address, int port);

    private:

    int m_fd; // File handle of socket.
    sockaddr_in m_packet_address; // Last address we read from.

};
