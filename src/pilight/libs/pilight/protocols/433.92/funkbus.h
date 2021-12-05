#ifndef _PROTOCOL_FUNKBUS_H_
#define _PROTOCOL_FUNKBUS_H_

#include "../protocol.h"

PROTOCOL_STRUCT_EXTERN struct protocol_t * funkbus;
void funkbusInit(void);

typedef enum {
    FB_ACTION_STOP,
    FB_ACTION_OFF,
    FB_ACTION_ON,
    FB_ACTION_SCENE
} funkbus_action_t;

typedef struct {
    uint8_t typ : 4;    // there are multible types
    uint8_t subtyp : 4;

    uint32_t sn : 20;

    uint8_t r1 : 2;     // unknown
    uint8_t bat : 1;    // 1 == battery low
    uint8_t r2 : 1;     // unknown

    uint8_t command : 3;       // button on the remote
    uint8_t group : 2;    // remote channel group 0-2 (A-C) are switches, 3 == light scene
    uint8_t r3 : 1;       // unknown

    funkbus_action_t action : 2;
    uint8_t repeat : 1;       // 1 == not first send of packet
    uint8_t longpress : 1;    // longpress of button for (dim up/down, scene lerning)
    uint8_t parity : 1;       // parity over all bits before
    uint8_t check : 4;        // lfsr with 8bit mask 0x8C sifted left by 2 bit
} __attribute__((packed)) funkbus_packet_t;

#endif
