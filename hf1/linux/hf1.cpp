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
#include <inttypes.h>

#include "timer_linux.h"
#include "guid_factory.h"
#include "logger_interface.h"

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


	TimerLinux timer;
	P2PByteStreamLinux byte_stream(serial_fd);
	GUIDFactory guid_factory;
	P2PPacketStream<4, 1, kLittleEndian> p2p_stream(&byte_stream, &timer, guid_factory);

	int sent_packets[P2PPriority::kNumLevels];
	int received_packets[P2PPriority::kNumLevels];
	auto start = std::chrono::system_clock::now();
	int last_sent_packets[P2PPriority::kNumLevels];
	int last_received_packets[P2PPriority::kNumLevels];
	int last_received_packet_value[P2PPriority::kNumLevels];
	int lost_packets[P2PPriority::kNumLevels];
	//std::chrono::time_point<std::chrono::system_clock> last_sent_packet_time;

	for (int i = 0; i < P2PPriority::kNumLevels; ++i) {
    		last_received_packet_value[i] = -1;
    		received_packets[i] = 0;
    		sent_packets[i] = 0;
    		last_received_packets[i] = 0;
    		last_sent_packets[i] = 0;
    		lost_packets[i] = 0;
	}

	while(doLoop) {

		auto now = std::chrono::system_clock::now();
                std::chrono::duration<double, std::chrono::seconds::period> elapsed_seconds = now-start;

		struct pollfd serial_poll;
        	serial_poll.fd = serial_fd;
        	serial_poll.events = POLLIN | POLLOUT;
	        serial_poll.revents = 0;

		int retval = poll(&serial_poll, 1, 100);
		if (retval == -1) {
			perror("poll() error");
		} else if (retval != 0) {
			if (serial_poll.revents & POLLIN) {
				p2p_stream.input().Run();
				if (p2p_stream.input().OldestPacket().ok()) {
					/*
					std::cout << "packet: ";
					for (int i = 0; i < p2p_input_stream.OldestPacket()->length(); ++i) {
						printf("%x ", p2p_input_stream.OldestPacket()->content()[i]);
					}
					std::cout << std::endl;
					*/
					P2PPriority priority = p2p_stream.input().OldestPacket()->priority();
					++received_packets[priority];
					if (last_received_packet_value[priority] >= 0) {
						int diff = p2p_stream.input().OldestPacket()->content()[0] - last_received_packet_value[priority];
						if (diff > 0) lost_packets[priority] += diff - 1;
						else lost_packets[priority] += 256 + diff - 1;
						//printf("new:%d old:%d ", p2p_input_stream.OldestPacket()->content()[0], last_received_packet_value[priority]);
						//std::cout << "diff:"<<diff<<" p:"<<priority<<" lost:"<<lost_packets[priority] << std::endl;
					}
					last_received_packet_value[priority] = p2p_stream.input().OldestPacket()->content()[0];
					p2p_stream.input().Consume(priority);
				}
			}

			if (serial_poll.revents & POLLOUT) {
				P2PPriority priority = P2PPriority::Level::kMedium;
				StatusOr<P2PMutablePacketView> current_packet_view = p2p_stream.output().NewPacket(priority);
    				if (current_packet_view.ok()) {
					static int len = 1; //0xa8;
					for (int i = 0; i < len; ++i) {
						current_packet_view->content()[i] = 0;
					}
        				*reinterpret_cast<uint8_t *>(current_packet_view->content()) = sent_packets[priority];
        				current_packet_view->length() = len; //sizeof(uint8_t);
        				ASSERT(p2p_stream.output().Commit(priority, /*guarantee_delivery=*/false));
					++sent_packets[priority];
					++len;
					if (len == 0xa9) { len = 1; }
    				}
				uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
				p2p_stream.output().Run();
			}
		}
		
		if (elapsed_seconds >= std::chrono::seconds(1)) {
			start = now;
			printf("tx:");
    			for (int i = 0; i < P2PPriority::kNumLevels; ++i) {
      				printf("%d ", sent_packets[i] - last_sent_packets[i]);
    			}
    			printf(", rx:");
    			for (int i = 0; i < P2PPriority::kNumLevels; ++i) {
      				printf("%d ", received_packets[i] - last_received_packets[i]);
    			}
    			printf(", lost:");
    			for (int i = 0; i < P2PPriority::kNumLevels; ++i) {
      				printf("%d ", lost_packets[i]);
    			}
			printf(", tx_delay(ns):");
			for (int i = 0; i < P2PPriority::kNumLevels; ++i) {
				uint64_t delay = p2p_stream.output().stats().average_packet_delay_per_byte_ns(i);
      				if (delay != -1ULL) {
                                	printf("%" PRIu64 " ", delay);
				} else {
					printf("? ");
				}
                        }
			printf(", rx_delay(ns):");
                        for (int i = 0; i < P2PPriority::kNumLevels; ++i) {
                                uint64_t delay = p2p_stream.input().stats().average_packet_delay_per_byte_ns(i);
                                if (delay != -1ULL) {
                                        printf("%" PRIu64 " ", delay);
                                } else {
                                        printf("? ");
                                }
                        }
			std::cout << "                ";
			std::cout << "\r" << std::flush;
    			for (int i = 0; i < P2PPriority::kNumLevels; ++i) {
      				last_sent_packets[i] = sent_packets[i];
      				last_received_packets[i] = received_packets[i];
    			}
		}
	}

	CleanUp();

	return 0;
}
