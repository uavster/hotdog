#include "p2p_packet_stream.h"
#include "p2p_byte_stream_linux.h"

#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <signal.h>
#include <termios.h>
#include <sys/stat.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/file.h>
#include <string.h>

#include <chrono>

#define kSerialPath "/dev/ttyTHS1"
//#define kBaudRate B115200
//#define kBaudRate B2000000
//#define kBaudRate B9600
#define kBaudRate B1000000

bool doLoop = true;
int serial_fd = -1;

static void ExitHandler(int param) {
	doLoop = false;
}

static void CleanUp() {
	if (serial_fd >= 0) {
                std::cout << "Closing serial port" << std::endl;
                flock(serial_fd, LOCK_UN);
                close(serial_fd);
        }
	serial_fd = -1;
}

int main() {
	signal(SIGTERM, &ExitHandler);
	signal(SIGINT, &ExitHandler);


	std::cout << "Opening serial port" << std::endl;

	serial_fd = open(kSerialPath, O_RDWR | O_NONBLOCK);
	if (serial_fd < 0) {
		perror("Error opening serial port");
		exit(-errno);
	}

	// Lock device file
	if (flock(serial_fd, LOCK_EX | LOCK_NB) < 0) {
		perror("Error failed to lock device file");
		exit(-errno);
	}

	struct termios newtio;
	bzero(&newtio, sizeof(newtio)); // clear struct for new port settings

	// man termios get more info on below settings
	newtio.c_cflag = kBaudRate | CS8 | CLOCAL | CREAD;
	newtio.c_iflag = 0;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;
	
	//newtio.c_cflag |= CSTOPB;

	// Minimum number of chacters for read to return (as long as it is lower than the count passed to the call). 
	newtio.c_cc[VMIN] = 0; // 128;

	// Intercharacter timeout. Read returns no matter what after this time (reset at every character).
	newtio.c_cc[VTIME] = 0; // 5;

	// Discard everything in input and output buffers.
	tcflush(serial_fd, TCIOFLUSH);
	// Set the new config.
	tcsetattr(serial_fd, TCSANOW, &newtio);


	struct pollfd serial_poll;
	serial_poll.fd = serial_fd;
	serial_poll.events = POLLIN | POLLOUT;
	serial_poll.revents = 0;

	P2PByteStreamLinux byte_stream(serial_fd);
	P2PPacketInputStream<16, kLittleEndian> p2p_input_stream(&byte_stream);
	P2PPacketOutputStream<16, kLittleEndian> p2p_output_stream(&byte_stream);


	int sent_packets = 0;
	int received_packets = 0;
	auto start = std::chrono::system_clock::now();
	int last_sent_packets = 0;
	int last_received_packets = 0;
	int last_received_packet_value = -1;
	int lost_packets = 0;
	std::chrono::time_point<std::chrono::system_clock> last_sent_packet_time;
	while(doLoop) {

		auto now = std::chrono::system_clock::now();
                std::chrono::duration<double, std::chrono::seconds::period> elapsed_seconds = now-start;

		int retval = poll(&serial_poll, 1, 100);
		if (retval == -1) {
			perror("poll() error");
		} else if (retval != 0) {
			if (serial_poll.revents & POLLIN) {
				p2p_input_stream.Run();
				if (p2p_input_stream.OldestPacket().ok()) {
					/*
					std::cout << "packet: ";
					for (int i = 0; i < p2p_input_stream.OldestPacket()->length(); ++i) {
						printf("%x ", p2p_input_stream.OldestPacket()->content()[i]);
					}
					std::cout << std::endl;
					*/
					++received_packets;
					if (last_received_packet_value >= 0) {
						int diff = p2p_input_stream.OldestPacket()->content()[0] - last_received_packet_value;
						if (diff > 0) lost_packets += diff - 1;
						else lost_packets += 256 + diff - 1;
					}
					last_received_packet_value = p2p_input_stream.OldestPacket()->content()[0];
					p2p_input_stream.Consume();
				}
			}

			if (serial_poll.revents & POLLOUT) {
				StatusOr<P2PMutablePacketView> current_packet_view = p2p_output_stream.NewPacket();
    				if (current_packet_view.ok()) {
					int len = 84;
					if (sent_packets == 0 || now - last_sent_packet_time > std::chrono::microseconds((len+3)*28)) {
        					*reinterpret_cast<uint8_t *>(current_packet_view->content()) = sent_packets;
        					current_packet_view->length() = len; //sizeof(uint8_t);
        					p2p_output_stream.Commit();
						last_sent_packet_time = now;
						++sent_packets;
					}
    				}
				p2p_output_stream.Run();
			}
		}
		
		if (elapsed_seconds >= std::chrono::seconds(1)) {
			start = now;
			printf("tx:%.2f packets/s, rx:%.2f packets/s, lost packets:%d      \r", (sent_packets - last_sent_packets) / elapsed_seconds.count(), (received_packets - last_received_packets) / elapsed_seconds.count(), lost_packets);
			std::cout << std::flush;
			last_sent_packets = sent_packets;
			last_received_packets = received_packets;
		}
	}

	CleanUp();

	return 0;
}
