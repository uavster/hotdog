class Uart {
public:
    Uart();
    virtual ~Uart();

    struct PollResult {
        bool can_receive;
        bool can_send;
    };

    PollResult Poll();
    
    int fd() const { return fd_; }    

private:
    int fd_;
};