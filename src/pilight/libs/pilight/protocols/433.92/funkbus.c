
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "../../core/pilight.h"
#include "../../core/common.h"

#include "../../core/log.h"
#include "../../core/binary.h"
#include "../../core/gc.h"
#include "../protocol.h"
#include "funkbus.h"

#include "config.h"

#define FUNKBUS_START_BIT_HI_MIN 3800    // 3,8ms
#define FUNKBUS_START_BIT_HI 4000        // 4ms
#define FUNKBUS_START_BIT_HI_MAX 4200    // 4,2ms

// HI
#define FUNKBUS_LONG_MIN 800     // 0,8ms
#define FUNKBUS_LONG 1000        // 1ms
#define FUNKBUS_LONG_MAX 1200    // 1,2ms

// LOW
#define FUNKBUS_SHORT_MIN 350    // 0,35ms
#define FUNKBUS_SHORT 500        // 0,5ms
#define FUNKBUS_SHORT_MAX 680    // 0,68ms

#define FUNKBUS_END 1500    // 1,5ms

#define RAW_LENGTH_MIN 48
#define RAW_LENGTH_MAX (RAW_LENGTH_MIN * 2)

static void createMessageRemote(const funkbus_packet_t * packet, int raw[], size_t raw_len) {
    funkbus->message = json_mkobject();

    json_append_member(funkbus->message, "type", json_mkstring("remote"));
    json_append_member(funkbus->message, "id", json_mknumber(packet->sn, 0));
    json_append_member(funkbus->message, "battery_ok", json_mkbool(packet->bat ? 0 : 1));
    json_append_member(funkbus->message, "command", json_mknumber(packet->command, 0));
    json_append_member(funkbus->message, "group", json_mknumber(packet->group, 0));
    json_append_member(funkbus->message, "channel", json_mknumber(((packet->group << 3) + packet->command), 0));
    json_append_member(funkbus->message, "action", json_mknumber(packet->action, 0));
    json_append_member(funkbus->message, "repeat", json_mkbool(packet->repeat));
    json_append_member(funkbus->message, "longpress", json_mkbool(packet->longpress));

#ifdef FUNKBUS_RAW
    if(raw_len) {
        json_append_member(funkbus->message, "parity", json_mkbool(packet->parity));
        json_append_member(funkbus->message, "check", json_mknumber(packet->check, 0));

        struct JsonNode * jraw = json_mkarray();
        for(uint8_t i = 0; i < raw_len; i++) {
            json_append_element(jraw, json_mknumber(raw[i], 0));
        }
        json_append_member(funkbus->message, "raw", jraw);
    }
#endif
}

static int validate(void) {
    if(funkbus->rawlen >= RAW_LENGTH_MIN && funkbus->rawlen <= RAW_LENGTH_MAX) {
        if(funkbus->raw[0] >= (FUNKBUS_SHORT_MIN) &&
            funkbus->raw[0] <= (FUNKBUS_LONG_MAX)) {
            return 0;
        }
    }
    //  logprintf(LOG_ERR, "funkbus: validate -1 %d %d", funkbus->rawlen, funkbus->raw[0]);
    return -1;
}

static uint8_t parity8(uint8_t byte) {
    byte ^= byte >> 4;
    byte &= 0xf;
    return (0x6996 >> byte) & 0x01;
}

static uint8_t xor_bytes(int raw[], size_t len) {
    uint8_t result = 0;
    for(uint8_t i = 0; i < len * 8; i += 8) {
        result ^= binToDecRev(raw, i, i + 7);
    }
    return result;
}

static uint8_t calc_checksum(int raw[], size_t len) {
    const uint8_t full_bytes = len / 8;
    const uint8_t bits_left  = len % 8;

    uint8_t xor = xor_bytes(raw, full_bytes);
    if(bits_left) {
        xor ^= (binToDecRev(raw, full_bytes * 8, len - 1) << (8 - bits_left));
    }

    const uint8_t xor_nibble = ((xor&0xF0) >> 4) ^ (xor&0x0F);

    uint8_t result = 0;
    if(xor_nibble & 0x8) {
        result ^= 0x8C;
    }
    if(xor_nibble & 0x4) {
        result ^= 0x32;
    }
    if(xor_nibble & 0x2) {
        result ^= 0xC8;
    }
    if(xor_nibble & 0x01) {
        result ^= 0x23;
    }

    result = result & 0xF;
    result |= (parity8(xor) << 4);

    return result;
}

static void varToBin(uint32_t value, int binary[], uint8_t len) {
    uint32_t mask = 1;
    for(uint8_t i = 0; i < len; i++) {
        binary[i] = value & mask ? 1 : 0;
        mask      = mask << 1;
    }
}

static void varToBinR(uint32_t value, int binary[], uint8_t len) {
    uint32_t mask = 1 << (len - 1);
    for(uint8_t i = 0; i < len; i++) {
        binary[i] = value & mask ? 1 : 0;
        mask      = mask >> 1;
    }
}

static uint8_t packet_to_bin(const funkbus_packet_t * packet, int binary[], size_t binary_max) {
    uint8_t set_c = 0;
#define set(var, len)                      \
    if(set_c + len > binary_max) return 0; \
    varToBin(var, &binary[set_c], len);    \
    set_c += len

    set(packet->typ, 4);
    set(packet->subtyp, 4);

    set(packet->sn, 20);
    set(packet->r1, 2);
    set(packet->bat, 1);
    set(packet->r2, 2);
    set(packet->command, 3);
    set(packet->group, 2);
    set(packet->r3, 1);
    set(packet->action, 2);
    set(packet->repeat, 1);
    set(packet->longpress, 1);

    const uint8_t checksum = calc_checksum(binary, 43);

    if(set_c + 5 > binary_max) return 0;
    varToBinR(checksum, &binary[set_c], 5);
    set_c += 5;

    // end bit
    set(1, 1);

    // signal needs to end in OFF check parity
    if(!(checksum & 0x10)) {
        set(1, 1);
    }

    return set_c;
}

static bool packet_to_raw(const funkbus_packet_t * packet) {
    int binary[RAW_LENGTH_MAX + 2];
    size_t binary_len = packet_to_bin(packet, binary, RAW_LENGTH_MAX + 2);

    if(!binary_len) {
        logprintf(LOG_ERR, "funkbus: packet_to_raw ERROR: !binary_len");
        return false;
    }

    uint8_t raw_c         = 0;
    funkbus->raw[raw_c++] = FUNKBUS_START_BIT_HI;

    for(uint8_t i = 0; i < binary_len; i++) {
        if(raw_c > 255) {
            logprintf(LOG_ERR, "funkbus: packet_to_raw ERROR: raw_c > 255");
            return false;
        }

        if(binary[i]) {
            funkbus->raw[raw_c++] = FUNKBUS_SHORT;
            funkbus->raw[raw_c++] = FUNKBUS_SHORT;
        } else {
            funkbus->raw[raw_c++] = FUNKBUS_LONG;
        }
    }
    funkbus->rawlen = raw_c;
    return true;
}

static void parseCode(void) {
    uint8_t binary_len         = 0;
    int binary[RAW_LENGTH_MAX] = { 0 };

    if(funkbus->rawlen > RAW_LENGTH_MAX) {
        logprintf(LOG_ERR, "funkbus: parsecode - invalid parameter passed %d", funkbus->rawlen);
        return;
    }

    for(uint8_t i = 0; i < funkbus->rawlen; i++) {
        // long pule
        if(funkbus->raw[i] >= FUNKBUS_LONG_MIN && funkbus->raw[i] <= FUNKBUS_LONG_MAX) {
            binary[binary_len++] = 0;
            // 2x short pule
        } else if(funkbus->raw[i] >= FUNKBUS_SHORT_MIN && funkbus->raw[i] <= FUNKBUS_SHORT_MAX &&
                  funkbus->raw[i + 1] >= FUNKBUS_SHORT_MIN && funkbus->raw[i + 1] <= FUNKBUS_SHORT_MAX) {
            binary[binary_len++] = 1;
            i++;
            // short pule at end
        } else if(funkbus->raw[i] >= FUNKBUS_SHORT_MIN && funkbus->raw[i] <= FUNKBUS_SHORT_MAX &&
                  (funkbus->raw[i + 1] >= FUNKBUS_END || i + 1 >= funkbus->rawlen)) {
            binary[binary_len++] = 1;
            // found end of packet 0
            break;
        } else if(funkbus->raw[i] > FUNKBUS_END) {
            binary[binary_len++] = 0;
            // found end of packet 1
            break;
        } else {
            logprintf(LOG_ERR, "funkbus: parsecode - Differential Manchester encoding failed at %d with %d", i, funkbus->raw[i]);
            return;
        }
    }
    if(binary_len < 24) {
        logprintf(LOG_ERR, "funkbus: binary_len < 24 (%d)", binary_len);
        return;
    }
    uint8_t get_c = 0;
#define get(x)                              \
    binToDec(binary, get_c, get_c + x - 1); \
    get_c += x

    funkbus_packet_t packet;
    packet.typ    = get(4);
    packet.subtyp = get(4);

    // remote
    if(packet.typ == 0x4 && packet.subtyp == 0x3) {
        if(binary_len < 48) {
            logprintf(LOG_ERR, "funkbus: binary_len < 47 (%d)", binary_len);
            return;
        }

        packet.sn        = get(20);
        packet.r1        = get(2);
        packet.bat       = get(1);
        packet.r2        = get(2);
        packet.command   = get(3);
        packet.group     = get(2);
        packet.r3        = get(1);
        packet.action    = get(2);
        packet.repeat    = get(1);
        packet.longpress = get(1);
        packet.parity    = get(1);
        packet.check     = get(4);

        uint8_t checksum    = calc_checksum(binary, 43);
        uint8_t checksum_is = binToDecRev(binary, 43, 47);
        if(checksum_is != checksum) {
            logprintf(LOG_ERR, "funkbus: checksum wrong! %X != %X", checksum_is, checksum);
            return;
        }

        createMessageRemote(&packet, binary, binary_len);
    } else {
        logprintf(LOG_ERR, "funkbus: get_c: %d", get_c);
        logprintf(LOG_ERR, "funkbus: len: %d", binary_len);
        logprintf(LOG_ERR, "funkbus: typ: %X", packet.typ);
        logprintf(LOG_ERR, "funkbus: subtyp: %X", packet.subtyp);
        logprintf(LOG_ERR, "funkbus: ERROR: typ not implemented");
    }
}

static int createCode(struct JsonNode * code) {
    funkbus_packet_t packet;
    double get_data = 0;

    packet.typ    = 0x4;
    packet.subtyp = 0x3;

    if(json_find_number(code, "id", &get_data) == 0) {
        packet.sn = get_data;
    } else {
        logprintf(LOG_ERR, "funkbus: createCode ERROR: missing id");
        return EXIT_FAILURE;
    }

    if(json_find_bool(code, "battery_ok", &get_data) == 0) {
        packet.bat = get_data;
    } else {
        packet.bat = 0;
    }

    if(json_find_number(code, "command", &get_data) == 0) {
        packet.command = get_data;
        if(json_find_number(code, "group", &get_data) == 0) {
            packet.group = get_data;
        } else {
            logprintf(LOG_ERR, "funkbus: createCode ERROR: missing group/command");
            return EXIT_FAILURE;
        }
    } else if(json_find_number(code, "channel", &get_data) == 0) {
        packet.command = ((uint8_t)get_data) & 0x7;
        packet.group   = ((uint8_t)get_data) >> 3 & 0x03;
    } else {
        logprintf(LOG_ERR, "funkbus: createCode ERROR: missing channel");
        return EXIT_FAILURE;
    }

    if(json_find_number(code, "action", &get_data) == 0) {
        packet.action = get_data;
    } else {
        logprintf(LOG_ERR, "funkbus: createCode ERROR: missing action");
        return EXIT_FAILURE;
    }

    if(json_find_bool(code, "repeat", &get_data) == 0) {
        packet.repeat = get_data;
    } else {
        packet.repeat = 0;
    }

    if(json_find_bool(code, "longpress", &get_data) == 0) {
        packet.longpress = get_data;
    } else {
        packet.longpress = 0;
    }

    createMessageRemote(&packet, NULL, 0);

    return packet_to_raw(&packet) ? EXIT_SUCCESS : EXIT_FAILURE;
}

void funkbusInit(void) {
    protocol_register(&funkbus);
    protocol_set_id(funkbus, "funkbus");
    protocol_device_add(funkbus, "funkbus", "Insta funkbus");
    funkbus->devtype   = SWITCH;
    funkbus->hwtype    = RF433;
    funkbus->minrawlen = RAW_LENGTH_MIN;
    funkbus->maxrawlen = RAW_LENGTH_MAX;
    funkbus->maxgaplen = FUNKBUS_LONG_MAX * PULSE_DIV;
    funkbus->mingaplen = FUNKBUS_SHORT_MIN * PULSE_DIV;

    funkbus->parseCode  = &parseCode;
    funkbus->createCode = &createCode;
    // funkbus->printHelp = &printHelp;
    funkbus->validate = &validate;
    funkbus->txrpt = 1;
}

#if defined(MODULE) && !defined(_WIN32)
void compatibility(struct module_t * module) {
    module->name       = "funkbus";
    module->version    = "1.1";
    module->reqversion = "6.0";
    module->reqcommit  = "187";
}

void init(void) {
    funkbusInit();
}
#endif
