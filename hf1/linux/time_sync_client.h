#ifndef TIME_SYNC_CLIENT_INCLUDED_
#define TIME_SYNC_CLIENT_INCLUDED_

class TimeSyncClient {
public:
    TimeSyncClient();
    ~TimeSyncClient();

    void Run();

    void RequestTimeSync();

private:
    enum { IDLE, REQUEST_TIME_SYNC, WAIT_FOR_TIME_SYNC_REPLY } state_;
};

#endif  // TIME_SYNC_CLIENT_INCLUDED_