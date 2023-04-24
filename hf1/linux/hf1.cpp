#include "p2p_packet_stream.h"
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
#define kBaudRate B9600

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
	
	newtio.c_cflag |= CSTOPB;

	// block for up till 128 characters
	newtio.c_cc[VMIN] = 128;

	// 0.5 seconds read timeout
	newtio.c_cc[VTIME] = 5;

	// now clean the modem line and activate the settings for the port
	tcflush(serial_fd, TCIOFLUSH);
	tcsetattr(serial_fd, TCSANOW, &newtio);


	struct pollfd serial_poll;
	serial_poll.fd = serial_fd;
	serial_poll.events |= POLLIN;
	serial_poll.events |= POLLOUT;


	int write_index = 0;
#pragma pack(push, 1)
	uint8_t write_packet[] = { 0xaa, 0x1, 0x40, 0x41 };
#pragma pack(pop)

	int sent_packets = 0;
	auto start = std::chrono::system_clock::now();
	while(doLoop) {
		int retval = poll(&serial_poll, 1, 1000);
		if (retval == -1) {
			perror("poll() error");
		} else if (retval != 0) {
			if (serial_poll.revents & POLLIN) {
			}

			if (serial_poll.revents & POLLOUT) {
				write_index += write(serial_fd, &write_packet[write_index], sizeof(write_packet) - write_index);
				if (write_index == sizeof(write_packet)) { ++sent_packets; write_index = 0; }
			}
		}
		auto now = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = now-start;
		if (elapsed_seconds >= std::chrono::seconds(1)) {
			start = now;
			std::cout << "packets: " << sent_packets << "\r" << std::flush;
		}

	}

	CleanUp();

	return 0;
}
