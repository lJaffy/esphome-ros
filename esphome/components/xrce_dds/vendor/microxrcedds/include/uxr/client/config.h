// Generated vendored config for ESPHome xrce_dds component.
// Mirrors Micro-XRCE-DDS-Client CMake defaults (v3.0.2) with a minimal
// profile: custom transport + stream framing (serial) only. No discovery,
// UDP/TCP/serial/CAN platform transports, no multithread/shared-memory.
// See vendor/VENDORED.md for the source list and regeneration notes.

#ifndef _UXR_CLIENT_CONFIG_H_
#define _UXR_CLIENT_CONFIG_H_

#define UXR_CLIENT_VERSION_MAJOR 3
#define UXR_CLIENT_VERSION_MINOR 0
#define UXR_CLIENT_VERSION_MICRO 2
#define UXR_CLIENT_VERSION_STR "3.0.2"

#define UCLIENT_PROFILE_CUSTOM_TRANSPORT
#define UCLIENT_PROFILE_STREAM_FRAMING

#define UXR_CONFIG_MAX_OUTPUT_BEST_EFFORT_STREAMS 1
#define UXR_CONFIG_MAX_OUTPUT_RELIABLE_STREAMS 1
#define UXR_CONFIG_MAX_INPUT_BEST_EFFORT_STREAMS 1
#define UXR_CONFIG_MAX_INPUT_RELIABLE_STREAMS 1

#define UXR_CONFIG_MAX_SESSION_CONNECTION_ATTEMPTS 10
#define UXR_CONFIG_MIN_SESSION_CONNECTION_INTERVAL 1000
#define UXR_CONFIG_MIN_HEARTBEAT_TIME_INTERVAL 100

#define UXR_CONFIG_CUSTOM_TRANSPORT_MTU 512

#define UCLIENT_TWEAK_XRCE_WRITE_LIMIT

// Version checks
#if UXR_CLIENT_VERSION_MAJOR >= 4
#error UCLIENT_HARD_LIVELINESS_CHECK shall be included in session API
#error MTU must be included in CREATE_CLIENT_Payload properties
#error Reorder ObjectInfo https://github.com/eProsima/Micro-XRCE-DDS/issues/137
#endif

#endif  // _UXR_CLIENT_CONFIG_H_
