class Uart {
public:
    enum Timeout { kInfinite = -1 };

    Uart();
    virtual ~Uart();

    struct PollResult {
        bool can_receive;
        bool can_send;
    };

    // Waits until there is data available to read or the timeout expires.
    // A timeout of zero returns immediately, while a timeout of Timeout::kInfinite won't
    // return until buffer space is available.
    bool CanRead(int timeout_ms);

    // Waits until there is buffer space available to write or the timeout expires.
    // A timeout of zero returns immediately, while a timeout of Timeout::kInfinite won't
    // return until data is available.
    bool CanWrite(int timeout_ms);
    
    // Waits until there are bytes available to read or space available to write, or the
    // timeout expires.
    // A timeout of zero returns immediately, while a timeout of Timeout::kInfinite won't
    // return until data is available or can be written.
    PollResult CanReadOrWrite(int timeout_ms);

    // Returns the UART's file descriptor. 
    // Reads or writes to this file descriptor will never block. This does not mean that
    // all the intended data is read or written. The read and write functions will output
    // the number of bytes that were processed successfully and the caller will have to
    // adapt accordingly.
    int fd() const { return fd_; }    

private:
    int fd_;
};