#include "uart.h"

#include <termios.h>
#include <sys/file.h>
#include <unistd.h>
#include <poll.h>
#include <string.h>

#include <logger_interface.h>

#define kSerialPath "/dev/ttyTHS1"
#define kBaudRate B1000000

Uart::Uart(bool is_async) {
	fd_ = open(kSerialPath, O_RDWR | (is_async ? O_NONBLOCK : 0));
	ASSERTM(fd_ >= 0, "Error opening serial port");

	// Lock device file.
	ASSERTM(flock(fd_, LOCK_EX | LOCK_NB) >= 0, "Error failed to lock device file");

	struct termios newtio;
	bzero(&newtio, sizeof(newtio));
	newtio.c_cflag = kBaudRate | CS8 | CLOCAL | CREAD;
	newtio.c_iflag = 0;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;
	
	// Minimum number of chacters for read to return (as long as it is lower than the count passed to the call). 
	newtio.c_cc[VMIN] = 0; // 128;

	// Intercharacter timeout. Read returns no matter what after this time (reset at every character).
	newtio.c_cc[VTIME] = 0; // 5;

	// Discard everything in input and output buffers.
	tcflush(fd_, TCIOFLUSH);

	// Set the new config.
	tcsetattr(fd_, TCSANOW, &newtio);
}

Uart::~Uart() {    
    if (fd_ >= 0) {
        flock(fd_, LOCK_UN);
        close(fd_);
        fd_ = -1;
    }
}

Uart::PollResult Uart::Poll() {
    struct pollfd serial_poll;
    serial_poll.fd = fd_;
    serial_poll.events = POLLIN | POLLOUT;
    serial_poll.revents = 0;

    int poll_res = poll(&serial_poll, 1, 100);
    ASSERT(poll_res != -1);

    PollResult result = { .can_receive = false, .can_send = false };    
    if (poll_res != 0) {
        result.can_receive = serial_poll.revents & POLLIN;
        result.can_send = serial_poll.revents & POLLOUT;
    }
    
    return result;
}

bool Uart::CanRead() {
    struct pollfd serial_poll;
    serial_poll.fd = fd_;
    serial_poll.events = POLLIN;
    serial_poll.revents = 0;

    int poll_res = poll(&serial_poll, 1, 100);
    ASSERT(poll_res != -1);

    return serial_poll.revents & POLLIN;
}

bool Uart::CanWrite() {
    struct pollfd serial_poll;
    serial_poll.fd = fd_;
    serial_poll.events = POLLIN;
    serial_poll.revents = 0;

    int poll_res = poll(&serial_poll, 1, 100);
    ASSERT(poll_res != -1);

    return serial_poll.revents & POLLIN;
}
