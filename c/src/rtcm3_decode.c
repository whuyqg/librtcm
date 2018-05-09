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

#include "rtcm3_decode.h"
#include <math.h>
#include "bits.h"

void init_sat_data(rtcm_sat_data *sat_data) {
  for (uint8_t freq = 0; freq < NUM_FREQS; ++freq) {
    sat_data->obs[freq].flags.data = 0;
  }
}

static uint32_t from_lock_ind(uint8_t lock) {
  if (lock < 24) return lock;
  if (lock < 48) return 2 * lock - 24;
  if (lock < 72) return 4 * lock - 120;
  if (lock < 96) return 8 * lock - 408;
  if (lock < 120) return 16 * lock - 1176;
  if (lock < 127) return 32 * lock - 3096;
  return 937;
}

/* Table 3.5-75 */
static uint32_t from_msm_lock_ind(uint8_t lock) {
  if (lock == 0) {
    return 0;
  } else {
    return (32 << lock) / 1000;
  }
}

void decode_basic_gps_l1_freq_data(const uint8_t *buff,
                                   uint16_t *bit,
                                   rtcm_freq_data *freq_data,
                                   uint32_t *pr,
                                   int32_t *phr_pr_diff) {
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

void decode_basic_glo_l1_freq_data(const uint8_t *buff,
                                   uint16_t *bit,
                                   rtcm_freq_data *freq_data,
                                   uint32_t *pr,
                                   int32_t *phr_pr_diff,
                                   uint8_t *fcn) {
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

void decode_basic_l2_freq_data(const uint8_t *buff,
                               uint16_t *bit,
                               rtcm_freq_data *freq_data,
                               int32_t *pr,
                               int32_t *phr_pr_diff) {
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

uint16_t rtcm3_read_header(const uint8_t *buff, rtcm_obs_header *header) {
  uint16_t bit = 0;
  header->msg_num = getbitu(buff, bit, 12);
  bit += 12;
  header->stn_id = getbitu(buff, bit, 12);
  bit += 12;
  header->tow_ms = getbitu(buff, bit, 30);
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

uint16_t rtcm3_read_glo_header(const uint8_t *buff, rtcm_obs_header *header) {
  uint16_t bit = 0;
  header->msg_num = getbitu(buff, bit, 12);
  bit += 12;
  header->stn_id = getbitu(buff, bit, 12);
  bit += 12;
  header->tow_ms = getbitu(buff, bit, 27);
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

uint16_t rtcm3_read_msm_header(const uint8_t *buff, rtcm_msm_header *header) {
  uint16_t bit = 0;
  header->msg_num = getbitu(buff, bit, 12);
  bit += 12;
  header->stn_id = getbitu(buff, bit, 12);
  bit += 12;
  header->tow_ms = getbitu(buff, bit, 30);
  bit += 30;
  header->multiple = getbitu(buff, bit, 1);
  bit += 1;
  header->reserved = getbitu(buff, bit, 7);
  bit += 7;
  header->steering = getbitu(buff, bit, 2);
  bit += 2;
  header->ext_clock = getbitu(buff, bit, 2);
  bit += 2;
  header->div_free = getbitu(buff, bit, 1);
  bit += 1;
  header->smooth = getbitu(buff, bit, 3);
  bit += 3;
  header->satellite_mask = getbitu(buff, bit, 64);
  bit += 64;
  header->signal_mask = getbitu(buff, bit, 32);
  bit += 32;
  header->cell_mask = getbitu(buff, bit, 64);
  bit += 64;
  return bit;
}

uint8_t construct_L1_code(rtcm_freq_data *l1_freq_data,
                          int32_t pr,
                          double amb_correction) {
  l1_freq_data->pseudorange = 0.02 * pr + amb_correction;
  if (pr != (int)PR_L1_INVALID) {
    return 1;
  }
  return 0;
}

uint8_t construct_L1_phase(rtcm_freq_data *l1_freq_data,
                           int32_t phr_pr_diff,
                           double freq) {
  l1_freq_data->carrier_phase =
      (l1_freq_data->pseudorange + 0.0005 * phr_pr_diff) / (CLIGHT / freq);
  if (phr_pr_diff != (int)CP_INVALID) {
    return 1;
  }
  return 0;
}

uint8_t construct_L2_code(rtcm_freq_data *l2_freq_data,
                          const rtcm_freq_data *l1_freq_data,
                          int32_t pr) {
  l2_freq_data->pseudorange = 0.02 * pr + l1_freq_data->pseudorange;
  if (pr != (int)PR_L2_INVALID) {
    return 1;
  }
  return 0;
}

uint8_t construct_L2_phase(rtcm_freq_data *l2_freq_data,
                           const rtcm_freq_data *l1_freq_data,
                           int32_t phr_pr_diff,
                           double freq) {
  l2_freq_data->carrier_phase =
      (l1_freq_data->pseudorange + 0.0005 * phr_pr_diff) / (CLIGHT / freq);
  if (phr_pr_diff != (int)CP_INVALID) {
    return 1;
  }
  return 0;
}

uint8_t get_cnr(rtcm_freq_data *freq_data, const uint8_t *buff, uint16_t *bit) {
  uint8_t cnr = getbitu(buff, *bit, 8);
  *bit += 8;
  if (cnr == 0) {
    return 0;
  }
  freq_data->cnr = 0.25 * cnr;
  return 1;
}

/** Decode an RTCMv3 message type 1001 (L1-Only GPS RTK Observables)
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return If valid then return 0.
 *         Returns a negative number if the message is invalid:
 *          - `-1` : Message type mismatch
 */
int8_t rtcm3_decode_1001(const uint8_t *buff, rtcm_obs_message *msg_1001) {
  uint16_t bit = 0;
  bit += rtcm3_read_header(buff, &msg_1001->header);

  if (msg_1001->header.msg_num != 1001) /* Unexpected message type. */
    return -1;

  for (uint8_t i = 0; i < msg_1001->header.n_sat; i++) {
    init_sat_data(&msg_1001->sats[i]);

    /* TODO: Handle SBAS prns properly, numbered differently in RTCM? */
    msg_1001->sats[i].svId = getbitu(buff, bit, 6);
    bit += 6;

    rtcm_freq_data *l1_freq_data = &msg_1001->sats[i].obs[L1_FREQ];

    uint32_t l1_pr;
    int32_t phr_pr_diff;
    decode_basic_gps_l1_freq_data(
        buff, &bit, l1_freq_data, &l1_pr, &phr_pr_diff);

    l1_freq_data->flags.valid_pr = construct_L1_code(l1_freq_data, l1_pr, 0);
    l1_freq_data->flags.valid_cp =
        construct_L1_phase(l1_freq_data, phr_pr_diff, GPS_L1_FREQ);
  }

  return 0;
}

/** Decode an RTCMv3 message type 1002 (Extended L1-Only GPS RTK Observables)
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return If valid then return 0.
 *         Returns a negative number if the message is invalid:
 *          - `-1` : Message type mismatch
 */
int8_t rtcm3_decode_1002(const uint8_t *buff, rtcm_obs_message *msg_1002) {
  uint16_t bit = 0;
  bit += rtcm3_read_header(buff, &msg_1002->header);

  if (msg_1002->header.msg_num != 1002) /* Unexpected message type. */
    return -1;

  for (uint8_t i = 0; i < msg_1002->header.n_sat; i++) {
    init_sat_data(&msg_1002->sats[i]);

    /* TODO: Handle SBAS prns properly, numbered differently in RTCM? */
    msg_1002->sats[i].svId = getbitu(buff, bit, 6);
    bit += 6;

    rtcm_freq_data *l1_freq_data = &msg_1002->sats[i].obs[L1_FREQ];

    uint32_t l1_pr;
    int32_t phr_pr_diff;
    decode_basic_gps_l1_freq_data(
        buff, &bit, l1_freq_data, &l1_pr, &phr_pr_diff);

    uint8_t amb = getbitu(buff, bit, 8);
    bit += 8;
    l1_freq_data->flags.valid_cnr = get_cnr(l1_freq_data, buff, &bit);
    l1_freq_data->flags.valid_pr =
        construct_L1_code(l1_freq_data, l1_pr, amb * PRUNIT_GPS);
    l1_freq_data->flags.valid_cp =
        construct_L1_phase(l1_freq_data, phr_pr_diff, GPS_L1_FREQ);
  }

  return 0;
}

/** Decode an RTCMv3 message type 1003 (L1/L2 GPS RTK Observables)
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return If valid then return 0.
 *         Returns a negative number if the message is invalid:
 *          - `-1` : Message type mismatch
 */
int8_t rtcm3_decode_1003(const uint8_t *buff, rtcm_obs_message *msg_1003) {
  uint16_t bit = 0;
  bit += rtcm3_read_header(buff, &msg_1003->header);

  if (msg_1003->header.msg_num != 1003) /* Unexpected message type. */
    return -1;

  for (uint8_t i = 0; i < msg_1003->header.n_sat; i++) {
    init_sat_data(&msg_1003->sats[i]);

    /* TODO: Handle SBAS prns properly, numbered differently in RTCM? */
    msg_1003->sats[i].svId = getbitu(buff, bit, 6);
    bit += 6;

    rtcm_freq_data *l1_freq_data = &msg_1003->sats[i].obs[L1_FREQ];

    uint32_t l1_pr;
    int32_t l2_pr;
    int32_t phr_pr_diff;
    decode_basic_gps_l1_freq_data(
        buff, &bit, l1_freq_data, &l1_pr, &phr_pr_diff);

    l1_freq_data->flags.valid_pr = construct_L1_code(l1_freq_data, l1_pr, 0);
    l1_freq_data->flags.valid_cp =
        construct_L1_phase(l1_freq_data, phr_pr_diff, GPS_L1_FREQ);

    rtcm_freq_data *l2_freq_data = &msg_1003->sats[i].obs[L2_FREQ];

    decode_basic_l2_freq_data(buff, &bit, l2_freq_data, &l2_pr, &phr_pr_diff);

    l2_freq_data->flags.valid_pr =
        construct_L2_code(l2_freq_data, l1_freq_data, l2_pr);
    l2_freq_data->flags.valid_cp = construct_L2_phase(
        l2_freq_data, l1_freq_data, phr_pr_diff, GPS_L2_FREQ);
  }

  return 0;
}

/** Decode an RTCMv3 message type 1004 (Extended L1/L2 GPS RTK Observables)
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return If valid then return 0.
 *         Returns a negative number if the message is invalid:
 *          - `-1` : Message type mismatch
 */
int8_t rtcm3_decode_1004(const uint8_t *buff, rtcm_obs_message *msg_1004) {
  uint16_t bit = 0;
  bit += rtcm3_read_header(buff, &msg_1004->header);

  if (msg_1004->header.msg_num != 1004) /* Unexpected message type. */
    return -1;

  for (uint8_t i = 0; i < msg_1004->header.n_sat; i++) {
    init_sat_data(&msg_1004->sats[i]);

    /* TODO: Handle SBAS prns properly, numbered differently in RTCM? */
    msg_1004->sats[i].svId = getbitu(buff, bit, 6);
    bit += 6;

    rtcm_freq_data *l1_freq_data = &msg_1004->sats[i].obs[L1_FREQ];

    uint32_t l1_pr;
    int32_t l2_pr;
    int32_t phr_pr_diff;
    decode_basic_gps_l1_freq_data(
        buff, &bit, l1_freq_data, &l1_pr, &phr_pr_diff);

    uint8_t amb = getbitu(buff, bit, 8);
    bit += 8;

    l1_freq_data->flags.valid_cnr = get_cnr(l1_freq_data, buff, &bit);
    l1_freq_data->flags.valid_pr =
        construct_L1_code(l1_freq_data, l1_pr, amb * PRUNIT_GPS);
    l1_freq_data->flags.valid_cp =
        construct_L1_phase(l1_freq_data, phr_pr_diff, GPS_L1_FREQ);

    rtcm_freq_data *l2_freq_data = &msg_1004->sats[i].obs[L2_FREQ];

    decode_basic_l2_freq_data(buff, &bit, l2_freq_data, &l2_pr, &phr_pr_diff);

    l2_freq_data->flags.valid_cnr = get_cnr(l2_freq_data, buff, &bit);
    l2_freq_data->flags.valid_pr =
        construct_L2_code(l2_freq_data, l1_freq_data, l2_pr);
    l2_freq_data->flags.valid_cp = construct_L2_phase(
        l2_freq_data, l1_freq_data, phr_pr_diff, GPS_L2_FREQ);
  }

  return 0;
}

int8_t rtcm3_decode_1005_base(const uint8_t *buff,
                              rtcm_msg_1005 *msg_1005,
                              uint16_t *bit) {
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

/** Decode an RTCMv3 message type 1005 (Stationary RTK Reference Station ARP)
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return If valid then return 0.
 *         Returns a negative number if the message is invalid:
 *          - `-1` : Message type mismatch
 */
int8_t rtcm3_decode_1005(const uint8_t *buff, rtcm_msg_1005 *msg_1005) {
  uint16_t bit = 0;
  uint16_t msg_num = getbitu(buff, bit, 12);
  bit += 12;

  if (msg_num != 1005) /* Unexpected message type. */
    return -1;

  return rtcm3_decode_1005_base(buff, msg_1005, &bit);
}

/** Decode an RTCMv3 message type 1005 (Stationary RTK Reference Station ARP
 * with antenna height)
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return If valid then return 0.
 *         Returns a negative number if the message is invalid:
 *          - `-1` : Message type mismatch
 */
int8_t rtcm3_decode_1006(const uint8_t *buff, rtcm_msg_1006 *msg_1006) {
  uint16_t bit = 0;
  uint16_t msg_num = getbitu(buff, bit, 12);
  bit += 12;

  if (msg_num != 1006) /* Unexpected message type. */
    return -1;

  rtcm3_decode_1005_base(buff, &msg_1006->msg_1005, &bit);
  msg_1006->ant_height = (double)(getbitu(buff, bit, 16)) / 10000.0;
  bit += 16;
  return 0;
}

int8_t rtcm3_decode_1007_base(const uint8_t *buff,
                              rtcm_msg_1007 *msg_1007,
                              uint16_t *bit) {
  msg_1007->stn_id = getbitu(buff, *bit, 12);
  *bit += 12;
  msg_1007->desc_count = getbitu(buff, *bit, 8);
  *bit += 8;
  for (uint8_t i = 0; i < msg_1007->desc_count; ++i) {
    msg_1007->desc[i] = getbitu(buff, *bit, 8);
    *bit += 8;
  }
  msg_1007->ant_id = getbitu(buff, *bit, 8);
  *bit += 8;

  return 0;
}

/** Decode an RTCMv3 message type 1007 (Antenna Descriptor)
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return If valid then return 0.
 *         Returns a negative number if the message is invalid:
 *          - `-1` : Message type mismatch
 */
int8_t rtcm3_decode_1007(const uint8_t *buff, rtcm_msg_1007 *msg_1007) {
  uint16_t bit = 0;
  uint16_t msg_num = getbitu(buff, bit, 12);
  bit += 12;

  if (msg_num != 1007) /* Unexpected message type. */
    return -1;

  rtcm3_decode_1007_base(buff, msg_1007, &bit);

  return 0;
}

/** Decode an RTCMv3 message type 1008 (Antenna Descriptor & Serial Number)
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return If valid then return 0.
 *         Returns a negative number if the message is invalid:
 *          - `-1` : Message type mismatch
 */
int8_t rtcm3_decode_1008(const uint8_t *buff, rtcm_msg_1008 *msg_1008) {
  uint16_t bit = 0;
  uint16_t msg_num = getbitu(buff, bit, 12);
  bit += 12;

  if (msg_num != 1008) /* Unexpected message type. */
    return -1;

  rtcm3_decode_1007_base(buff, &msg_1008->msg_1007, &bit);
  msg_1008->serial_count = getbitu(buff, bit, 8);
  bit += 8;
  for (uint8_t i = 0; i < msg_1008->serial_count; ++i) {
    msg_1008->serial_num[i] = getbitu(buff, bit, 8);
    bit += 8;
  }
  return 0;
}

/** Decode an RTCMv3 message type 1010 (Extended L1-Only GLO RTK Observables)
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return If valid then return 0.
 *         Returns a negative number if the message is invalid:
 *          - `-1` : Message type mismatch
 */
int8_t rtcm3_decode_1010(const uint8_t *buff, rtcm_obs_message *msg_1010) {
  uint16_t bit = 0;
  bit += rtcm3_read_glo_header(buff, &msg_1010->header);

  if (msg_1010->header.msg_num != 1010) /* Unexpected message type. */
    return -1;

  for (uint8_t i = 0; i < msg_1010->header.n_sat; i++) {
    init_sat_data(&msg_1010->sats[i]);

    msg_1010->sats[i].svId = getbitu(buff, bit, 6);
    bit += 6;

    rtcm_freq_data *l1_freq_data = &msg_1010->sats[i].obs[L1_FREQ];

    uint32_t l1_pr;
    int32_t phr_pr_diff;
    decode_basic_glo_l1_freq_data(
        buff, &bit, l1_freq_data, &l1_pr, &phr_pr_diff, &msg_1010->sats[i].fcn);

    uint8_t amb = getbitu(buff, bit, 7);
    bit += 7;

    l1_freq_data->flags.valid_cnr = get_cnr(l1_freq_data, buff, &bit);

    int8_t glo_fcn = msg_1010->sats[i].fcn - 7;
    l1_freq_data->flags.valid_pr =
        construct_L1_code(l1_freq_data, l1_pr, PRUNIT_GLO * amb);
    l1_freq_data->flags.valid_cp = construct_L1_phase(
        l1_freq_data, phr_pr_diff, GLO_L1_FREQ + glo_fcn * GLO_L1_CH_OFFSET);
  }

  return 0;
}

/** Decode an RTCMv3 message type 1012 (Extended L1/L2 GLO RTK Observables)
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return If valid then return 0.
 *         Returns a negative number if the message is invalid:
 *          - `-1` : Message type mismatch
 */
int8_t rtcm3_decode_1012(const uint8_t *buff, rtcm_obs_message *msg_1012) {
  uint16_t bit = 0;
  bit += rtcm3_read_glo_header(buff, &msg_1012->header);

  if (msg_1012->header.msg_num != 1012) /* Unexpected message type. */
    return -1;

  for (uint8_t i = 0; i < msg_1012->header.n_sat; i++) {
    init_sat_data(&msg_1012->sats[i]);

    /* TODO: Handle SBAS prns properly, numbered differently in RTCM? */
    msg_1012->sats[i].svId = getbitu(buff, bit, 6);
    bit += 6;

    rtcm_freq_data *l1_freq_data = &msg_1012->sats[i].obs[L1_FREQ];

    uint32_t l1_pr;
    int32_t l2_pr;
    int32_t phr_pr_diff;
    decode_basic_glo_l1_freq_data(
        buff, &bit, l1_freq_data, &l1_pr, &phr_pr_diff, &msg_1012->sats[i].fcn);

    uint8_t amb = getbitu(buff, bit, 7);
    bit += 7;

    int8_t glo_fcn = msg_1012->sats[i].fcn - 7;
    l1_freq_data->flags.valid_cnr = get_cnr(l1_freq_data, buff, &bit);
    l1_freq_data->flags.valid_pr =
        construct_L1_code(l1_freq_data, l1_pr, amb * PRUNIT_GLO);
    l1_freq_data->flags.valid_cp = construct_L1_phase(
        l1_freq_data, phr_pr_diff, GLO_L1_FREQ + glo_fcn * GLO_L1_CH_OFFSET);

    rtcm_freq_data *l2_freq_data = &msg_1012->sats[i].obs[L2_FREQ];

    decode_basic_l2_freq_data(buff, &bit, l2_freq_data, &l2_pr, &phr_pr_diff);

    l2_freq_data->flags.valid_cnr = get_cnr(l2_freq_data, buff, &bit);
    l2_freq_data->flags.valid_pr =
        construct_L2_code(l2_freq_data, l1_freq_data, l2_pr);
    l2_freq_data->flags.valid_cp =
        construct_L2_phase(l2_freq_data,
                           l1_freq_data,
                           phr_pr_diff,
                           GLO_L2_FREQ + glo_fcn * GLO_L2_CH_OFFSET);
  }

  return 0;
}

/** Decode an RTCMv3 message type 1029 (Unicode Text String Message)
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return If valid then return 0.
 *         Returns a negative number if the message is invalid:
 *          - `-1` : Message type mismatch
 */
int8_t rtcm3_decode_1029(const uint8_t *buff, rtcm_msg_1029 *msg_1029) {
  uint16_t bit = 0;
  uint16_t msg_num = getbitu(buff, bit, 12);
  bit += 12;

  if (msg_num != 1029) /* Unexpected message type. */
    return -1;

  msg_1029->stn_id = getbitu(buff, bit, 12);
  bit += 12;

  msg_1029->mjd_num = getbitu(buff, bit, 16);
  bit += 16;

  msg_1029->utc_sec_of_day = getbitu(buff, bit, 17);
  bit += 17;

  msg_1029->unicode_chars = getbitu(buff, bit, 7);
  bit += 7;

  msg_1029->utf8_code_units_n = getbitu(buff, bit, 8);
  bit += 8;
  for (uint8_t i = 0; i < msg_1029->utf8_code_units_n; ++i) {
    msg_1029->utf8_code_units[i] = getbitu(buff, bit, 8);
    bit += 8;
  }

  return 0;
}

/** Decode an RTCMv3 message type 1033 (Rcv and Ant descriptor)
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return If valid then return 0.
 *         Returns a negative number if the message is invalid:
 *          - `-1` : Message type mismatch
 */
int8_t rtcm3_decode_1033(const uint8_t *buff, rtcm_msg_1033 *msg_1033) {
  uint16_t bit = 0;
  uint16_t msg_num = getbitu(buff, bit, 12);
  bit += 12;

  if (msg_num != 1033) /* Unexpected message type. */
    return -1;

  msg_1033->stn_id = getbitu(buff, bit, 12);
  bit += 12;

  msg_1033->antenna_desc_counter = getbitu(buff, bit, 8);
  bit += 8;
  for (uint8_t i = 0; i < msg_1033->antenna_desc_counter; ++i) {
    msg_1033->antenna_descriptor[i] = getbitu(buff, bit, 8);
    bit += 8;
  }

  msg_1033->antenna_setup_ID = getbitu(buff, bit, 8);
  bit += 8;

  msg_1033->antenna_serial_num_counter = getbitu(buff, bit, 8);
  bit += 8;
  for (uint8_t i = 0; i < msg_1033->antenna_serial_num_counter; ++i) {
    msg_1033->antenna_serial_num[i] = getbitu(buff, bit, 8);
    bit += 8;
  }

  msg_1033->rcv_descriptor_counter = getbitu(buff, bit, 8);
  bit += 8;
  for (uint8_t i = 0; i < msg_1033->rcv_descriptor_counter; ++i) {
    msg_1033->rcv_descriptor[i] = getbitu(buff, bit, 8);
    bit += 8;
  }

  msg_1033->rcv_fw_counter = getbitu(buff, bit, 8);
  bit += 8;
  for (uint8_t i = 0; i < msg_1033->rcv_fw_counter; ++i) {
    msg_1033->rcv_fw_version[i] = getbitu(buff, bit, 8);
    bit += 8;
  }

  msg_1033->rcv_serial_num_counter = getbitu(buff, bit, 8);
  bit += 8;
  for (uint8_t i = 0; i < msg_1033->rcv_serial_num_counter; ++i) {
    msg_1033->rcv_serial_num[i] = getbitu(buff, bit, 8);
    bit += 8;
  }

  return 0;
}

/** Decode an RTCMv3 message type 1230 (Code-Phase Bias Message)
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return If valid then return 0.
 *         Returns a negative number if the message is invalid:
 *          - `-1` : Message type mismatch
 */
int8_t rtcm3_decode_1230(const uint8_t *buff, rtcm_msg_1230 *msg_1230) {
  uint16_t bit = 0;
  uint16_t msg_num = getbitu(buff, bit, 12);
  bit += 12;

  if (msg_num != 1230) /* Unexpected message type. */
    return -1;

  msg_1230->stn_id = getbitu(buff, bit, 12);
  bit += 12;
  msg_1230->bias_indicator = getbitu(buff, bit, 1);
  bit += 1;
  /* 3 Reserved bits */
  bit += 3;
  msg_1230->fdma_signal_mask = getbitu(buff, bit, 4);
  bit += 4;
  if (msg_1230->fdma_signal_mask & 0x08) {
    msg_1230->L1_CA_cpb_meter = getbits(buff, bit, 16) * 0.02;
    bit += 16;
  }
  if (msg_1230->fdma_signal_mask & 0x04) {
    msg_1230->L1_P_cpb_meter = getbits(buff, bit, 16) * 0.02;
    bit += 16;
  }
  if (msg_1230->fdma_signal_mask & 0x02) {
    msg_1230->L2_CA_cpb_meter = getbits(buff, bit, 16) * 0.02;
    bit += 16;
  }
  if (msg_1230->fdma_signal_mask & 0x01) {
    msg_1230->L2_P_cpb_meter = getbits(buff, bit, 16) * 0.02;
    bit += 16;
  }

  return 0;
}

void decode_msm_sat_data(const uint8_t *buff,
                         const uint8_t num_sats,
                         const msm_enum msm_type,
                         double rough_pseudorange[num_sats],
                         double sat_info[num_sats],
                         double rough_rate[num_sats],
                         uint16_t *bit) {
  /* number of integer milliseconds, DF397 */
  for (uint8_t i = 0; i < num_sats; i++) {
    uint32_t range_ms = getbitu(buff, *bit, 8);
    *bit += 8;
    /* TODO: handle invalid value */
    rough_pseudorange[i] = (double)range_ms * PRUNIT_GPS;
  }

  /* satellite info (constellation-dependent)*/
  for (uint8_t i = 0; i < num_sats; i++) {
    if (MSM5 == msm_type || MSM7 == msm_type) {
      sat_info[i] = getbitu(buff, *bit, 4);
      *bit += 4;
    } else {
      sat_info[i] = 0;
    }
  }

  /* rough range modulo 1 ms, DF398 */
  for (uint8_t i = 0; i < num_sats; i++) {
    uint32_t rough_pr = getbitu(buff, *bit, 10);
    *bit += 10;
    rough_pseudorange[i] += (double)rough_pr / 1024 * PRUNIT_GPS;
  }

  /* range rate, m/s, DF399*/
  for (uint8_t i = 0; i < num_sats; i++) {
    if (MSM5 == msm_type) {
      rough_rate[i] = (double)getbits(buff, *bit, 14);
      *bit += 14;
    } else {
      rough_rate[i] = 0;
    }
  }
}

void decode_msm_fine_pseudoranges(const uint8_t *buff,
                                  const uint8_t num_cells,
                                  double fine_pr[num_cells],
                                  flag_bf flags[num_cells],
                                  uint16_t *bit) {
  /* DF400 */
  for (uint16_t i = 0; i < num_cells; i++) {
    int16_t decoded = (int16_t)getbits(buff, *bit, 15);
    *bit += 15;
    flags[i].valid_pr = (decoded != MSM_PR_INVALID);
    fine_pr[i] = (double)decoded * C_1_2P24 * PRUNIT_GPS;
  }
}

void decode_msm_fine_phaseranges(const uint8_t *buff,
                                 const uint8_t num_cells,
                                 double fine_cp[num_cells],
                                 flag_bf flags[num_cells],
                                 uint16_t *bit) {
  /* DF401 */
  for (uint16_t i = 0; i < num_cells; i++) {
    int32_t decoded = getbits(buff, *bit, 22);
    *bit += 22;
    flags[i].valid_cp = (decoded != MSM_CP_INVALID);
    fine_cp[i] = (double)decoded * C_1_2P29 * PRUNIT_GPS;
  }
}

void decode_msm_lock_times(const uint8_t *buff,
                           const uint8_t num_cells,
                           uint32_t lock_time[num_cells],
                           flag_bf flags[num_cells],
                           uint16_t *bit) {
  /* DF402 */
  for (uint16_t i = 0; i < num_cells; i++) {
    uint32_t lock_ind = getbitu(buff, *bit, 4);
    *bit += 4;
    lock_time[i] = from_msm_lock_ind(lock_ind);
    flags[i].valid_lock = 1;
  }
}

void decode_msm_hca_indicators(const uint8_t *buff,
                               const uint8_t num_cells,
                               bool hca_indicator[num_cells],
                               flag_bf flags[num_cells],
                               uint16_t *bit) {
  /* DF420 */
  (void)flags;
  for (uint16_t i = 0; i < num_cells; i++) {
    hca_indicator[i] = (bool)getbitu(buff, *bit, 1);
    *bit += 1;
  }
}

void decode_msm_cnrs(const uint8_t *buff,
                     const uint8_t num_cells,
                     double cnr[num_cells],
                     flag_bf flags[num_cells],
                     uint16_t *bit) {
  /* DF403 */
  for (uint16_t i = 0; i < num_cells; i++) {
    uint32_t decoded = getbitu(buff, *bit, 6);
    *bit += 6;
    flags[i].valid_cnr = (decoded != 0);
    cnr[i] = (double)decoded;
  }
}

void decode_msm_fine_phaserangerates(const uint8_t *buff,
                                     const uint8_t num_cells,
                                     double fine_dop[num_cells],
                                     flag_bf flags[num_cells],
                                     uint16_t *bit) {
  /* DF404 */
  (void)flags;
  for (uint16_t i = 0; i < num_cells; i++) {
    int32_t decoded = getbits(buff, *bit, 15);
    *bit += 15;
    fine_dop[i] = (double)decoded * 0.0001;
  }
}

/** Decode an RTCMv3 Multi System Message
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return If valid then return 0.
 *         Returns a negative number if the message is invalid:
 *          - `-1` : Message type mismatch
 */
int8_t rtcm3_decode_msm(const uint8_t *buff, rtcm_msm_message *msg) {
  uint16_t bit = 0;
  bit += rtcm3_read_msm_header(buff, &msg->header);

  msm_enum msm_type = to_msm_type(msg->header.msg_num);

  if (msg->header.msg_num != 1074 && msg->header.msg_num != 1075) {
    /* Unexpected message type. */
    return -1;
  }

  uint8_t num_sats = count_bits_u64(msg->header.satellite_mask, 1);
  uint8_t num_sigs = count_bits_u32(msg->header.signal_mask, 1);
  uint8_t num_cells = count_bits_u64(msg->header.cell_mask, 1);

  /* Satellite Data */

  double rough_pseudorange[num_sats];
  double rough_rate[num_sats];
  double sat_info[num_sats];
  decode_msm_sat_data(
      buff, num_sats, msm_type, rough_pseudorange, sat_info, rough_rate, &bit);

  /* Signal Data */

  double fine_pr[num_cells];
  double fine_cp[num_cells];
  uint32_t lock_time[num_cells];
  bool hca_indicator[num_cells];
  double cnr[num_cells];
  double fine_dop[num_cells];
  flag_bf flags[num_cells];

  decode_msm_fine_pseudoranges(buff, num_cells, fine_pr, flags, &bit);
  decode_msm_fine_phaseranges(buff, num_cells, fine_cp, flags, &bit);
  decode_msm_lock_times(buff, num_cells, lock_time, flags, &bit);
  decode_msm_hca_indicators(buff, num_cells, hca_indicator, flags, &bit);
  decode_msm_cnrs(buff, num_cells, cnr, flags, &bit);
  if (MSM5 == msm_type) {
    decode_msm_fine_phaserangerates(
        buff, num_cells, fine_dop, flags, &bit);
  }

  uint8_t i = 0;
  for (uint8_t sat = 0; sat < num_sats; sat++) {
    msg->sats[sat].rough_pseudorange = rough_pseudorange[sat];
    msg->sats[sat].sat_info = sat_info[sat];
    msg->sats[sat].rough_range_rate = rough_rate[sat];
    for (uint8_t sig = 0; sig < num_sigs; sig++) {
      uint64_t index = (uint64_t)1 << (sat * num_sigs + sig);
      if (msg->header.cell_mask & index) {
        msg->signals[i].flags = flags[i];
        msg->signals[i].pseudorange = rough_pseudorange[sat] + fine_pr[i];
        double freq = (sig == 0) ? GPS_L1_FREQ : GPS_L2_FREQ;
        msg->signals[i].carrier_phase =
            (rough_pseudorange[sat] + fine_cp[i]) * (freq / CLIGHT);
        msg->signals[i].lock_time_s = lock_time[i];
        msg->signals[i].hca_indicator = hca_indicator[i];
        msg->signals[i].cnr = cnr[i];
        msg->signals[i].range_rate = rough_rate[sat] + fine_dop[i];
        i++;
      }
    }
  }

  return 0;
}
