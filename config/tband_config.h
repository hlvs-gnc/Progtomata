#ifndef TBAND_CONFIG_H_
#define TBAND_CONFIG_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Enable Tonbandgerät
#define tband_configENABLE 1

// Use streaming backend
#define tband_configUSE_BACKEND_STREAMING 1

// Use metadata buffer
#define tband_configUSE_METADATA_BUF 1

// Enable FreeRTOS tracing
#define tband_configFREERTOS_TRACE_ENABLE 1

// Enable ISR tracing
#define tband_configISR_TRACE_ENABLE      1

// Enable marker tracing
#define tband_configMARKER_TRACE_ENABLE   1

// Periodic dropped event reporting
#define tband_configTRACE_DROP_CNT_EVERY  5000

// Timestamp configuration
uint64_t traceport_timestamp(void);
#define tband_portTIMESTAMP()             traceport_timestamp()
#define tband_portTIMESTAMP_RESOLUTION_NS 25  // Adjust for your system

// Streaming data handler (shares UART with Trice)
bool traceport_stream_data(const uint8_t *buf, size_t len);
#define tband_portBACKEND_STREAM_DATA(_buf_, _len_) \
    traceport_stream_data((_buf_), (_len_))

#endif /* TBAND_CONFIG_H_ */