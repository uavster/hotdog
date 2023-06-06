#include "p2p_packet_stream.h"
#include "p2p_byte_stream_linux.h"

#include <iostream>
#include <signal.h>

#include <chrono>
#include <inttypes.h>

#include "timer_linux.h"
#include "guid_factory.h"
#include "logger_interface.h"

#include "time_sync_client.h"

#include "uart.h"

#define kP2PInputCapacity 4
#define kP2POutputCapacity 1
#define kP2PLocalEndianness kLittleEndian

bool doLoop = true;

static void ExitHandler(int param) {
	doLoop = false;
}

int main() {
	signal(SIGTERM, &ExitHandler);
	signal(SIGINT, &ExitHandler);

	TimerLinux timer;
	Uart serial;
	P2PByteStreamLinux byte_stream(serial.fd());
	GUIDFactory guid_factory;
	P2PPacketStream<kP2PInputCapacity, kP2POutputCapacity, kP2PLocalEndianness> p2p_stream(&byte_stream, &timer, guid_factory);

	int sent_packets[P2PPriority::kNumLevels];
	int received_packets[P2PPriority::kNumLevels];
	// auto start = std::chrono::system_clock::now();
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

	// TODO: Define abstract interfaces in the P2P API so not everything has to be a template and drag the P2P parameters.
	TimeSyncClient<kP2PInputCapacity, kP2POutputCapacity, kP2PLocalEndianness> time_sync_client(&p2p_stream, &timer);
	time_sync_client.RequestTimeSync();

    uint64_t next_global_time_event_s = -1ULL; 

	while(doLoop) {

		// auto now = std::chrono::system_clock::now();
        // std::chrono::duration<double, std::chrono::seconds::period> elapsed_seconds = now - start;

		Uart::PollResult poll_result = serial.Poll();
		if (poll_result.can_receive) {
			p2p_stream.input().Run();
			// if (p2p_stream.input().OldestPacket().ok()) {
			// 	P2PPriority priority = p2p_stream.input().OldestPacket()->priority();
			// 	++received_packets[priority];
			// 	if (last_received_packet_value[priority] >= 0) {
			// 		int diff = p2p_stream.input().OldestPacket()->content()[0] - last_received_packet_value[priority];
			// 		if (diff > 0) lost_packets[priority] += diff - 1;
			// 		else lost_packets[priority] += 256 + diff - 1;
			// 		//printf("new:%d old:%d ", p2p_input_stream.OldestPacket()->content()[0], last_received_packet_value[priority]);
			// 		//std::cout << "diff:"<<diff<<" p:"<<priority<<" lost:"<<lost_packets[priority] << std::endl;
			// 	}
			// 	last_received_packet_value[priority] = p2p_stream.input().OldestPacket()->content()[0];
			// 	p2p_stream.input().Consume(priority);
			// }
		}

		if (poll_result.can_send) {
			// P2PPriority priority = P2PPriority::Level::kMedium;
			// StatusOr<P2PMutablePacketView> current_packet_view = p2p_stream.output().NewPacket(priority);
			// 	if (current_packet_view.ok()) {
			// 	static int len = 1; //0xa8;
			// 	for (int i = 0; i < len; ++i) {
			// 		current_packet_view->content()[i] = 0;
			// 	}
			// 		*reinterpret_cast<uint8_t *>(current_packet_view->content()) = sent_packets[priority];
			// 		current_packet_view->length() = len; //sizeof(uint8_t);
			// 		ASSERT(p2p_stream.output().Commit(priority, /*guarantee_delivery=*/false));
			// 	++sent_packets[priority];
			// 	++len;
			// 	if (len == 0xa9) { len = 1; }
			// 	}
			// uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
			p2p_stream.output().Run();
		}
/*		
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
*/
		if (next_global_time_event_s == -1ULL) {
			next_global_time_event_s = (timer.GetGlobalNanoseconds() + 1000000000ULL) / 1000000000ULL;
		}
		uint64_t cur_global_time_ns = timer.GetGlobalNanoseconds();
		if (cur_global_time_ns / 1000000000ULL >= next_global_time_event_s) {
			std::cout << cur_global_time_ns << std::endl;
			next_global_time_event_s = (cur_global_time_ns + 1000000000ULL) / 1000000000ULL;
		}

		time_sync_client.Run();
	}

	return 0;
}
