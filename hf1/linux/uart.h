class Uart {
public:
    Uart(bool is_async = false);
    virtual ~Uart();

    struct PollResult {
        bool can_receive;
        bool can_send;
    };

    // Tells if there are bytes available to read.
    // If this object was created with is_async==true, the call returns immediately.
    // If this object was created with is_async==false, the call blocks until there are
    // bytes available to read.
    bool CanRead();

    // Tells if there is buffer space available to write.
    // If this object was created with is_async==true, the call returns immediately.
    // If this object was created with is_async==false, the call blocks until there are
    // bytes available to read.
    bool CanWrite();
    
    // Tells if there are bytes available to read or space available to write.
    // If this object was created with is_async==true, the call returns immediately.
    // If this object was created with is_async==false, the call blocks until either
    // condition is true.
    PollResult Poll();

    int fd() const { return fd_; }    

private:
    int fd_;
};