/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

// pgrgich: need to handle invalid obs...

#include "rtcm3_decode.h"
#include <math.h>

/** Get bit field from buffer as an unsigned integer.
 * Unpacks `len` bits at bit position `pos` from the start of the buffer.
 * Maximum bit field length is 32 bits, i.e. `len <= 32`.
 *
 * \param buff
 * \param pos Position in buffer of start of bit field in bits.
 * \param len Length of bit field in bits.
 * \return Bit field as an unsigned value.
 */
u32 getbitu(const u8 *buff, u32 pos, u8 len) {
  u32 bits = 0;

  for (u32 i = pos; i < pos + len; i++) {
    bits = (bits << 1) + ((buff[i / 8] >> (7 - i % 8)) & 1u);
  }

  return bits;
}

/** Get bit field from buffer as an unsigned integer.
 * Unpacks `len` bits at bit position `pos` from the start of the buffer.
 * Maximum bit field length is 32 bits, i.e. `len <= 32`.
 *
 * \param buff
 * \param pos Position in buffer of start of bit field in bits.
 * \param len Length of bit field in bits.
 * \return Bit field as an unsigned value.
 */
u64 getbitul(const u8 *buff, u32 pos, u8 len) {
  u64 bits = 0;

  for (u32 i = pos; i < pos + len; i++) {
    bits = (bits << 1) + ((buff[i / 8] >> (7 - i % 8)) & 1u);
  }

  return bits;
}

void init_data(rtcm_sat_data *sat_data) {
  for (u8 freq = 0; freq < NUM_FREQS; ++freq) {
    sat_data->obs[freq].flags.data = 0;
  }
}

static u32 from_lock_ind(u8 lock) {
  if (lock < 24)
    return lock;
  if (lock < 48)
    return 2 * lock - 24;
  if (lock < 72)
    return 4 * lock - 120;
  if (lock < 96)
    return 8 * lock - 408;
  if (lock < 120)
    return 16 * lock - 1176;
  if (lock < 127)
    return 32 * lock - 3096;
  return 937;
}

void decode_basic_gps_l1_freq_data(const u8 *buff, u16 *bit,
                                   rtcm_freq_data *freq_data, u32 *pr,
                                   s32 *phr_pr_diff) {
  freq_data->code = getbitu(buff, *bit, 1);
  *bit += 1;
  *pr = getbitu(buff, *bit, 24);
  *bit += 24;
  *phr_pr_diff = getbits(buff, *bit, 20);
  *bit += 20;

  freq_data->lock = from_lock_ind(getbitu(buff, *bit, 7));
  *bit += 7;
  freq_data->flags.valid_lock = 1;

  return;
}

void decode_basic_glo_l1_freq_data(const u8 *buff, u16 *bit,
                                   rtcm_freq_data *freq_data, u32 *pr,
                                   s32 *phr_pr_diff, u8 *fcn) {
  freq_data->code = getbitu(buff, *bit, 1);
  *bit += 1;
  *fcn = getbitu(buff, *bit, 5);
  *bit += 5;
  *pr = getbitu(buff, *bit, 25);
  *bit += 25;
  *phr_pr_diff = getbits(buff, *bit, 20);
  *bit += 20;
  freq_data->lock = from_lock_ind(getbitu(buff, *bit, 7));
  *bit += 7;
  freq_data->flags.valid_lock = 1;
  return;
}

void decode_basic_l2_freq_data(const u8 *buff, u16 *bit,
                               rtcm_freq_data *freq_data, s32 *pr,
                               s32 *phr_pr_diff) {
  freq_data->code = getbitu(buff, *bit, 2);
  *bit += 2;
  *pr = getbits(buff, *bit, 14);
  *bit += 14;
  *phr_pr_diff = getbits(buff, *bit, 20);
  *bit += 20;

  freq_data->lock = from_lock_ind(getbitu(buff, *bit, 7));
  *bit += 7;
  freq_data->flags.valid_lock = 1;

  return;
}

/** Get bit field from buffer as a signed integer.
 * Unpacks `len` bits at bit position `pos` from the start of the buffer.
 * Maximum bit field length is 32 bits, i.e. `len <= 32`.
 *
 * This function sign extends the `len` bit field to a signed 32 bit integer.
 *
 * \param buff
 * \param pos Position in buffer of start of bit field in bits.
 * \param len Length of bit field in bits.
 * \return Bit field as a signed value.
 */
s32 getbits(const u8 *buff, u32 pos, u8 len) {
  s32 bits = (s32)getbitu(buff, pos, len);

  /* Sign extend, taken from:
   * http://graphics.stanford.edu/~seander/bithacks.html#VariableSignExtend
   */
  s32 m = 1u << (len - 1);
  return (bits ^ m) - m;
}

/** Get bit field from buffer as a signed integer.
 * Unpacks `len` bits at bit position `pos` from the start of the buffer.
 * Maximum bit field length is 64 bits, i.e. `len <= 64`.
 *
 * This function sign extends the `len` bit field to a signed 64 bit integer.
 *
 * \param buff
 * \param pos Position in buffer of start of bit field in bits.
 * \param len Length of bit field in bits.
 * \return Bit field as a signed value.
 */
s64 getbitsl(const u8 *buff, u32 pos, u8 len) {
  s64 bits = (s64)getbitul(buff, pos, len);

  /* Sign extend, taken from:
   * http://graphics.stanford.edu/~seander/bithacks.html#VariableSignExtend
   */
  s64 m = ((u64)1) << (len - 1);
  return (bits ^ m) - m;
}

/** Read RTCM header for observation message types 1001..1004.
 *
 * The data message header will be read starting from byte zero of the
 * buffer. If the buffer also contains a frame header then be sure to pass a
 * pointer to the start of the data message rather than a pointer to the start
 * of the frame buffer. The RTCM observation header is 8 bytes (64 bits) long.
 *
 * All return values are written into the parameters passed by reference.
 *
 * \param buff A pointer to the RTCM data message buffer.
 * \param type Message type number, i.e. 1001..1004 (DF002).
 * \param id Reference station ID (DF003).
 * \param tow GPS time of week of the epoch (DF004).
 * \param sync Synchronous GNSS Flag (DF005).
 * \param n_sat Number of GPS satellites included in the message (DF006).
 * \param div_free GPS Divergence-free Smoothing Indicator (DF007).
 * \param smooth GPS Smoothing Interval indicator (DF008).
 */
u16 rtcm3_read_header(const u8 *buff, rtcm_obs_header *header) {
  u16 bit = 0;
  header->msg_num = getbitu(buff, bit, 12);
  bit += 12;
  header->stn_id = getbitu(buff, bit, 12);
  bit += 12;
  header->tow = getbitu(buff, bit, 30) / 1e3;
  bit += 30;
  header->sync = getbitu(buff, bit, 1);
  bit += 1;
  header->n_sat = getbitu(buff, bit, 5);
  bit += 5;
  header->div_free = getbitu(buff, bit, 1);
  bit += 1;
  header->smooth = getbitu(buff, bit, 3);
  bit += 3;
  return bit;
}

/** Read RTCM header for observation message types 1009..1012.
 *
 * The data message header will be read starting from byte zero of the
 * buffer. If the buffer also contains a frame header then be sure to pass a
 * pointer to the start of the data message rather than a pointer to the start
 * of the frame buffer. The RTCM observation header is 8 bytes (61 bits) long.
 *
 * All return values are written into the parameters passed by reference.
 *
 * \param buff A pointer to the RTCM data message buffer.
 * \param header A pointer to the header to be completed
 */
u16 rtcm3_read_glo_header(const u8 *buff, rtcm_obs_header *header) {
  u16 bit = 0;
  header->msg_num = getbitu(buff, bit, 12);
  bit += 12;
  header->stn_id = getbitu(buff, bit, 12);
  bit += 12;
  header->tow = getbitu(buff, bit, 27) / 1e3;
  bit += 27;
  header->sync = getbitu(buff, bit, 1);
  bit += 1;
  header->n_sat = getbitu(buff, bit, 5);
  bit += 5;
  header->div_free = getbitu(buff, bit, 1);
  bit += 1;
  header->smooth = getbitu(buff, bit, 3);
  bit += 3;
  return bit;
}

s8 rtcm3_decode_1001(const u8 *buff, rtcm_obs_message *msg_1001) {
  u16 bit = 0;
  bit += rtcm3_read_header(buff, &msg_1001->header);

  if (msg_1001->header.msg_num != 1001)
    /* Unexpected message type. */
    return -1;

  for (u8 i = 0; i < msg_1001->header.n_sat; i++) {
    init_data(&msg_1001->sats[i]);

    /* TODO: Handle SBAS prns properly, numbered differently in RTCM? */
    msg_1001->sats[i].svId = getbitu(buff, bit, 6);
    bit += 6;

    rtcm_freq_data *l1_freq_data = &msg_1001->sats[i].obs[L1_FREQ];

    u32 l1_pr;
    s32 phr_pr_diff;
    decode_basic_gps_l1_freq_data(buff, &bit, l1_freq_data, &l1_pr,
                                  &phr_pr_diff);

    l1_freq_data->pseudorange = 0.02 * l1_pr;
    l1_freq_data->flags.valid_pr = 1;
    l1_freq_data->carrier_phase =
        (l1_freq_data->pseudorange + 0.0005 * phr_pr_diff) /
        (CLIGHT / GPS_L1_FREQ);
    l1_freq_data->flags.valid_cp = 1;
  }

  return 0;
}

/** Decode an RTCMv3 message type 1002 (Extended L1-Only GPS RTK Observables)
 *
 * \param buff A pointer to the RTCM data message buffer.
 * \param id Reference station ID (DF003).
 * \param tow GPS time of week of epoch (DF004).
 * \param n_sat Number of GPS satellites included in the message (DF006).
 * \param nm Struct containing the observation.
 * \param sync Synchronous GNSS Flag (DF005).
 * \return If valid then return 0.
 *         Returns a negative number if the message is invalid:
 *          - `-1` : Message type mismatch
 *          - `-2` : Message uses unsupported P(Y) code
 */
s8 rtcm3_decode_1002(const u8 *buff, rtcm_obs_message *msg_1002) {
  u16 bit = 0;
  bit += rtcm3_read_header(buff, &msg_1002->header);

  if (msg_1002->header.msg_num != 1002)
    /* Unexpected message type. */
    return -1;

  for (u8 i = 0; i < msg_1002->header.n_sat; i++) {
    init_data(&msg_1002->sats[i]);

    /* TODO: Handle SBAS prns properly, numbered differently in RTCM? */
    msg_1002->sats[i].svId = getbitu(buff, bit, 6);
    bit += 6;

    rtcm_freq_data *l1_freq_data = &msg_1002->sats[i].obs[L1_FREQ];

    u32 l1_pr;
    s32 phr_pr_diff;
    decode_basic_gps_l1_freq_data(buff, &bit, l1_freq_data, &l1_pr,
                                  &phr_pr_diff);

    u8 amb = getbitu(buff, bit, 8);
    bit += 8;
    l1_freq_data->cnr = 0.25 * getbitu(buff, bit, 8);
    bit += 8;
    l1_freq_data->flags.valid_cnr = 1;

    l1_freq_data->pseudorange = 0.02 * l1_pr + PRUNIT_GPS * amb;
    l1_freq_data->flags.valid_pr = 1;
    l1_freq_data->carrier_phase =
        (l1_freq_data->pseudorange + 0.0005 * phr_pr_diff) /
        (CLIGHT / GPS_L1_FREQ);
    l1_freq_data->flags.valid_cp = 1;
  }

  return 0;
}

s8 rtcm3_decode_1003(const u8 *buff, rtcm_obs_message *msg_1003) {
  u16 bit = 0;
  bit += rtcm3_read_header(buff, &msg_1003->header);

  if (msg_1003->header.msg_num != 1003)
    /* Unexpected message type. */
    return -1;

  for (u8 i = 0; i < msg_1003->header.n_sat; i++) {
    init_data(&msg_1003->sats[i]);

    /* TODO: Handle SBAS prns properly, numbered differently in RTCM? */
    msg_1003->sats[i].svId = getbitu(buff, bit, 6);
    bit += 6;

    rtcm_freq_data *l1_freq_data = &msg_1003->sats[i].obs[L1_FREQ];

    u32 l1_pr;
    s32 l2_pr;
    s32 phr_pr_diff;
    decode_basic_gps_l1_freq_data(buff, &bit, l1_freq_data, &l1_pr,
                                  &phr_pr_diff);

    l1_freq_data->pseudorange = 0.02 * l1_pr;
    l1_freq_data->flags.valid_pr = 1;
    l1_freq_data->carrier_phase =
        (l1_freq_data->pseudorange + 0.0005 * phr_pr_diff) /
        (CLIGHT / GPS_L1_FREQ);
    l1_freq_data->flags.valid_cp = 1;

    rtcm_freq_data *l2_freq_data = &msg_1003->sats[i].obs[L2_FREQ];

    decode_basic_l2_freq_data(buff, &bit, l2_freq_data, &l2_pr,
                              &phr_pr_diff);

    l2_freq_data->pseudorange = 0.02 * l2_pr + l1_freq_data->pseudorange;
    l2_freq_data->flags.valid_pr = 1;
    l2_freq_data->carrier_phase =
        (l1_freq_data->pseudorange + 0.0005 * phr_pr_diff) /
        (CLIGHT / GPS_L2_FREQ);
    l2_freq_data->flags.valid_cp = 1;
  }

  return 0;
}

s8 rtcm3_decode_1004(const u8 *buff, rtcm_obs_message *msg_1004) {
  u16 bit = 0;
  bit += rtcm3_read_header(buff, &msg_1004->header);

  if (msg_1004->header.msg_num != 1004)
    /* Unexpected message type. */
    return -1;

  for (u8 i = 0; i < msg_1004->header.n_sat; i++) {
    init_data(&msg_1004->sats[i]);

    /* TODO: Handle SBAS prns properly, numbered differently in RTCM? */
    msg_1004->sats[i].svId = getbitu(buff, bit, 6);
    bit += 6;

    rtcm_freq_data *l1_freq_data = &msg_1004->sats[i].obs[L1_FREQ];

    u32 l1_pr;
    s32 l2_pr;
    s32 phr_pr_diff;
    decode_basic_gps_l1_freq_data(buff, &bit, l1_freq_data, &l1_pr,
                                  &phr_pr_diff);

    u8 amb = getbitu(buff, bit, 8);
    bit += 8;
    l1_freq_data->cnr = 0.25 * getbitu(buff, bit, 8);
    bit += 8;
    l1_freq_data->flags.valid_cnr = 1;

    l1_freq_data->pseudorange = 0.02 * l1_pr + PRUNIT_GPS * amb;
    l1_freq_data->flags.valid_pr = 1;
    l1_freq_data->carrier_phase =
        (l1_freq_data->pseudorange + 0.0005 * phr_pr_diff) /
        (CLIGHT / GPS_L1_FREQ);
    l1_freq_data->flags.valid_cp = 1;

    rtcm_freq_data *l2_freq_data = &msg_1004->sats[i].obs[L2_FREQ];

    decode_basic_l2_freq_data(buff, &bit, l2_freq_data, &l2_pr,
                              &phr_pr_diff);

    l2_freq_data->cnr = 0.25 * getbitu(buff, bit, 8);
    bit += 8;
    l2_freq_data->flags.valid_cnr = 1;

    l2_freq_data->pseudorange = 0.02 * l2_pr + l1_freq_data->pseudorange;
    l2_freq_data->flags.valid_pr = 1;
    l2_freq_data->carrier_phase =
        (l1_freq_data->pseudorange + 0.0005 * phr_pr_diff) /
        (CLIGHT / GPS_L2_FREQ);
    l2_freq_data->flags.valid_cp = 1;
  }

  return 0;
}

s8 rtcm3_decode_1005_base(const u8 *buff, rtcm_msg_1005 *msg_1005,
                          u16 *bit) {
  msg_1005->stn_id = getbitu(buff, *bit, 12);
  *bit += 12;
  msg_1005->ITRF = getbitu(buff, *bit, 6);
  *bit += 6;
  msg_1005->GPS_ind = getbitu(buff, *bit, 1);
  *bit += 1;
  msg_1005->GLO_ind = getbitu(buff, *bit, 1);
  *bit += 1;
  msg_1005->GAL_ind = getbitu(buff, *bit, 1);
  *bit += 1;
  msg_1005->ref_stn_ind = getbitu(buff, *bit, 1);
  *bit += 1;
  msg_1005->arp_x = (double)(getbitsl(buff, *bit, 38)) / 10000.0;
  *bit += 38;
  msg_1005->osc_ind = getbitu(buff, *bit, 1);
  *bit += 1;
  getbitu(buff, *bit, 1);
  *bit += 1;
  msg_1005->arp_y = (double)(getbitsl(buff, *bit, 38)) / 10000.0;
  *bit += 38;
  msg_1005->quart_cycle_ind = getbitu(buff, *bit, 2);
  *bit += 2;
  msg_1005->arp_z = (double)(getbitsl(buff, *bit, 38)) / 10000.0;
  *bit += 38;

  return 0;
}

s8 rtcm3_decode_1005(const u8 *buff, rtcm_msg_1005 *msg_1005) {
  u16 bit = 0;
  u16 msg_num = getbitu(buff, bit, 12);
  bit += 12;

  if (msg_num != 1005)
    /* Unexpected message type. */
    return -1;

  return rtcm3_decode_1005_base(buff, msg_1005, &bit);
}

s8 rtcm3_decode_1006(const u8 *buff, rtcm_msg_1006 *msg_1006) {
  u16 bit = 0;
  u16 msg_num = getbitu(buff, bit, 12);
  bit += 12;

  if (msg_num != 1006)
    /* Unexpected message type. */
    return -1;

  rtcm3_decode_1005_base(buff, &msg_1006->msg_1005, &bit);
  msg_1006->ant_height = (double)(getbitu(buff, bit, 16)) / 10000.0;
  bit += 16;
  return 0;
}

s8 rtcm3_decode_1007_base(const u8 *buff, rtcm_msg_1007 *msg_1007,
                          u16 *bit) {
  msg_1007->stn_id = getbitu(buff, *bit, 12);
  *bit += 12;
  msg_1007->desc_count = getbitu(buff, *bit, 8);
  *bit += 8;
  for (u8 i = 0; i < msg_1007->desc_count; ++i) {
    msg_1007->desc[i] = getbitu(buff, *bit, 8);
    *bit += 8;
  }
  msg_1007->ant_id = getbitu(buff, *bit, 8);
  *bit += 8;

  return 0;
}

s8 rtcm3_decode_1007(const u8 *buff, rtcm_msg_1007 *msg_1007) {
  u16 bit = 0;
  u16 msg_num = getbitu(buff, bit, 12);
  bit += 12;

  if (msg_num != 1007)
    /* Unexpected message type. */
    return -1;

  rtcm3_decode_1007_base(buff, msg_1007, &bit);

  return 0;
}

s8 rtcm3_decode_1008(const u8 *buff, rtcm_msg_1008 *msg_1008) {
  u16 bit = 0;
  u16 msg_num = getbitu(buff, bit, 12);
  bit += 12;

  if (msg_num != 1008)
    /* Unexpected message type. */
    return -1;

  rtcm3_decode_1007_base(buff, &msg_1008->msg_1007, &bit);
  msg_1008->serial_count = getbitu(buff, bit, 8);
  bit += 8;
  for (u8 i = 0; i < msg_1008->serial_count; ++i) {
    msg_1008->serial_num[i] = getbitu(buff, bit, 8);
    bit += 8;
  }
  return 0;
}

s8 rtcm3_decode_1010(const u8 *buff, rtcm_obs_message *msg_1010) {
  u16 bit = 0;
  bit += rtcm3_read_glo_header(buff, &msg_1010->header);

  if (msg_1010->header.msg_num != 1010)
    /* Unexpected message type. */
    return -1;

  for (u8 i = 0; i < msg_1010->header.n_sat; i++) {
    init_data(&msg_1010->sats[i]);

    msg_1010->sats[i].svId = getbitu(buff, bit, 6);
    bit += 6;

    rtcm_freq_data *l1_freq_data = &msg_1010->sats[i].obs[L1_FREQ];

    u32 l1_pr;
    s32 phr_pr_diff;
    decode_basic_glo_l1_freq_data(buff, &bit, l1_freq_data, &l1_pr,
                                  &phr_pr_diff, &msg_1010->sats[i].fcn);

    u8 amb = getbitu(buff, bit, 7);
    bit += 7;
    l1_freq_data->cnr = 0.25 * getbitu(buff, bit, 8);
    bit += 8;
    l1_freq_data->flags.valid_cnr = 1;

    l1_freq_data->pseudorange = 0.02 * l1_pr + PRUNIT_GLO * amb;
    l1_freq_data->flags.valid_pr = 1;
    s8 glo_fcn = msg_1010->sats[i].fcn - 7;
    l1_freq_data->carrier_phase =
      (l1_freq_data->pseudorange + 0.0005 * phr_pr_diff) /
      (CLIGHT / (GLO_L1_FREQ + glo_fcn * GLO_L1_CH_OFFSET) );
    l1_freq_data->flags.valid_cp = 1;
  }

  return 0;
}

s8 rtcm3_decode_1012(const u8 *buff, rtcm_obs_message *msg_1012) {
  u16 bit = 0;
  bit += rtcm3_read_glo_header(buff, &msg_1012->header);

  if (msg_1012->header.msg_num != 1012)
    /* Unexpected message type. */
    return -1;

  for (u8 i = 0; i < msg_1012->header.n_sat; i++) {
    init_data(&msg_1012->sats[i]);

    /* TODO: Handle SBAS prns properly, numbered differently in RTCM? */
    msg_1012->sats[i].svId = getbitu(buff, bit, 6);
    bit += 6;

    rtcm_freq_data *l1_freq_data = &msg_1012->sats[i].obs[L1_FREQ];

    u32 l1_pr;
    s32 l2_pr;
    s32 phr_pr_diff;
    decode_basic_glo_l1_freq_data(buff, &bit, l1_freq_data, &l1_pr,
                                  &phr_pr_diff, &msg_1012->sats[i].fcn);

    u8 amb = getbitu(buff, bit, 8);
    bit += 8;
    l1_freq_data->cnr = 0.25 * getbitu(buff, bit, 8);
    bit += 8;
    l1_freq_data->flags.valid_cnr = 1;

    l1_freq_data->pseudorange = 0.02 * l1_pr + PRUNIT_GLO * amb;
    l1_freq_data->flags.valid_pr = 1;
    s8 glo_fcn = msg_1012->sats[i].fcn - 7;
    l1_freq_data->carrier_phase =
      (l1_freq_data->pseudorange + 0.0005 * phr_pr_diff) /
      (CLIGHT / (GLO_L1_FREQ + glo_fcn * GLO_L1_CH_OFFSET));
    l1_freq_data->flags.valid_cp = 1;

    rtcm_freq_data *l2_freq_data = &msg_1012->sats[i].obs[L2_FREQ];

    decode_basic_l2_freq_data(buff, &bit, l2_freq_data, &l2_pr,
                                    &phr_pr_diff);

    l2_freq_data->cnr = 0.25 * getbitu(buff, bit, 8);
    bit += 8;
    l2_freq_data->flags.valid_cnr = 1;

    l2_freq_data->pseudorange = 0.02 * l2_pr + l1_freq_data->pseudorange;
    l2_freq_data->flags.valid_pr = 1;
    l2_freq_data->carrier_phase =
      (l1_freq_data->pseudorange + 0.0005 * phr_pr_diff) /
      (CLIGHT / (GLO_L2_FREQ + glo_fcn * GLO_L2_CH_OFFSET));
    l2_freq_data->flags.valid_cp = 1;
  }

  return 0;
}

