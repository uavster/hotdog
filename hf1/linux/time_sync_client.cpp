#include "time_sync_client.h"

#include <JetsonGPIO.h>

#define kTimeSyncRequestPinNumber 33

TimeSyncClient::TimeSyncClient() {
    GPIO::setmode(GPIO::BOARD);
    GPIO::setup(kTimeSyncRequestPinNumber, GPIO::OUT, GPIO::HIGH);    
}

TimeSyncClient::~TimeSyncClient() {
    GPIO::cleanup();    
}

void TimeSyncClient::Run() {
    switch(state_) {
        case IDLE:
            break;
        case REQUEST_TIME_SYNC: {
            break;
        }
    }
}

void TimeSyncClient::RequestTimeSync() {
    if (state_ != IDLE) {
        return;
    }
    state_ = REQUEST_TIME_SYNC;
}