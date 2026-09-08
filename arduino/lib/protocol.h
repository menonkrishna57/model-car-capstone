#ifndef CAR_PROTOCOL_H
#define CAR_PROTOCOL_H

// =============================================================================
// Shared Serial Protocol Constants (9600 Baud)
// Source of truth: arduino/PROTOCOL.md
// =============================================================================

#define SERIAL_BAUD_RATE 9600
#define SERIAL_BUFFER_SIZE 64

// Telemetry prefixes
#define TELEM_PREFIX "[TELEM] "
#define ACK_PREFIX   "[ACK] "

#endif // CAR_PROTOCOL_H
