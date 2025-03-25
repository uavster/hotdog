#ifndef NULL_STREAM_INCLUDED_
#define NULL_STREAM_INCLUDED_

#include <Arduino.h>

class NullStream : public Stream {
public:
	virtual int available() override { return 0; }
	virtual int read() override { return 0; }
	virtual int peek() override { return 0; }
	virtual size_t write(uint8_t b) { return 0; }
	virtual size_t write(const uint8_t *buffer, size_t size) { return 0; }
};

extern NullStream null_stream;

#endif  // NULL_STREAM_INCLUDED_