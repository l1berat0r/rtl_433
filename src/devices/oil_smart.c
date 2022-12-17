/** @file
    Oil tank monitor using manchester encoded FSK protocol with CRC.

    Copyright (C) 2022 Christian W. Zuckschwerdt <zany@triq.net>
    Device analysis by StarMonkey1

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

#include "decoder.h"

/**
Oil tank monitor using manchester encoded FSK protocol with CRC.

Tested devices:
- Apollo Ultrasonic Smart liquid monitor (FSK, 433.92M) Issue #2244

Should apply to similar Watchman, Beckett, and Apollo devices too.

There is a preamble plus de-sync of 555558, then MC coded an inner preamble of 5558 (raw 9999996a).
End of frame is the last half-bit repeated additional 2 times, then 4 times mark.

FIXME: confirm this
The sensor sends a single packet once every hour or twice a second
for 11 minutes when in pairing/test mode (pairing needs 35 sec).
depth reading is in cm, lowest reading is ~3, highest is ~305, 0 is invalid

Data Format:

    PRE?16h ID?16h CTR?16h CM:8d CRC:8h

Data Layout:

    PP PP II II FF CC DD XX

- P: 16 bit Preamble of 0x5558
- I: 16 bit Sensor ID
- F: 8 bit Flags maybe
- C: 8 bit Counter maybe
- D: 8 bit Depth in cm, could have a MSB somewhere?
- X: 8 bit CRC-8, poly 0x31 init 0x00, bit reflected

example packets are:

raw: {158}555558 9999 996a 6559aaa99996a55696a9a5963c
aligned: {134}9999996a 6559aaa999969aa6aa9a6995fc

FIXME: this is not confirmed
Start of frame full preamble is depending on first data bit either

    0101 0101 0101 0101 0101 0111 01
    0101 0101 0101 0101 0101 1000 10
*/
static int oil_smart_decode(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos)
{
    bitbuffer_t databits = {0};
    bitbuffer_manchester_decode(bitbuffer, row, bitpos, &databits, 64);

    if (databits.bits_per_row[0] < 64) {
        return 0; // DECODE_ABORT_LENGTH; // TODO: fix calling code to handle negative return values
    }

    uint8_t *b = databits.bb[0];

    if (b[0] != 0x55 || b[1] != 0x58) {
        decoder_log(decoder, 2, __func__, "Couldn't find preamble");
        return 0; // DECODE_FAIL_SANITY; // TODO: fix calling code to handle negative return values
    }

    if (crc8le(b, 8, 0x31, 0x00)) {
        decoder_log(decoder, 2, __func__, "CRC8 fail");
        return 0; // DECODE_FAIL_MIC; // TODO: fix calling code to handle negative return values
    }

    // FIXME: does not seem to be the case here
    // The unit ID changes when you rebind by holding a magnet to the
    // sensor for long enough.
    uint16_t unit_id = (b[2] << 8) | b[3];

    // FIXME: none of these are confirmed.
    // 0x01: Rebinding (magnet held to sensor)
    // 0x02: High-bit for depth
    // 0x04: (always zero?)
    // 0x08: Leak/theft alarm
    // 0x10: (unknown toggle)
    // 0x20: (unknown toggle)
    // 0x40: (unknown toggle)
    // 0x80: (always zero?)
    uint8_t flags = b[4];
    uint8_t alarm = b[5];

    // FIXME: confirm there is a binding counter
    uint16_t depth             = 0;
    uint16_t binding_countdown = 0;
    if (flags & 1) {
        // When binding, the countdown counts up from 0x40 to 0x4a
        // (as long as you hold the magnet to it for long enough)
        // before the device ID changes. The receiver unit needs
        // to receive this *strongly* in order to change its
        // allegiance.
        binding_countdown = b[6];
    }
    else {
        // A depth reading of zero indicates no reading.
        //depth = ((b[5] & 0x02) << 7) | b[6];
        depth = b[6];
    }

    /* clang-format off */
    data_t *data = data_make(
            "model",                "", DATA_STRING, "Oil-Ultrasonic",
            "id",                   "", DATA_FORMAT, "%04x", DATA_INT, unit_id,
            "flags",                "", DATA_FORMAT, "%02x", DATA_INT, flags,
            "alarm",                "", DATA_FORMAT, "%02x", DATA_INT, alarm,
            "binding_countdown",    "", DATA_INT,    binding_countdown,
            "depth_cm",             "", DATA_INT,    depth,
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/**
Oil tank monitor using manchester encoded FSK protocol with CRC.
@sa oil_smart_decode()
*/
static int oil_smart_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    uint8_t const preamble_pattern[2] = {0x55, 0x58};
    // End of frame is the last half-bit repeated additional 2 times, then 4 times mark.

    unsigned bitpos = 0;
    int events      = 0;

    // Find a preamble with enough bits after it that it could be a complete packet
    while ((bitpos = bitbuffer_search(bitbuffer, 0, bitpos, preamble_pattern, 16)) + 128 <=
            bitbuffer->bits_per_row[0]) {
        events += oil_smart_decode(decoder, bitbuffer, 0, bitpos + 16);
        bitpos += 2;
    }

    return events;
}

static char *output_fields[] = {
        "model",
        "id",
        "flags",
        "alarm",
        "binding_countdown",
        "depth_cm",
        NULL,
};

r_device oil_smart = {
        .name        = "Oil Ultrasonic SMART FSK",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 500,
        .long_width  = 500,
        .reset_limit = 2000,
        .decode_fn   = &oil_smart_callback,
        .fields      = output_fields,
};
