/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "rtcm3_eph_decode.h"
#include "bits.h"

/** Decode an RTCMv3 GPS Ephemeris Message
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 *          - RC_INVALID_MESSAGE : Cell mask too large or invalid TOW
 */
rtcm3_rc decode_gps_eph(const uint8_t buff[], uint16_t *bit, rtcm_msg_eph *msg_eph) {
  msg_eph->sat_id = getbitu(buff, *bit, 6);
  *bit += 6;
  msg_eph->wn = getbitu(buff, *bit, 10);
  *bit += 10;
  msg_eph->ura = getbitu(buff, *bit, 4);
  *bit += 4;
  /*uint8_t l2_code = */ getbitu(buff, *bit, 2);
  *bit += 2;
  msg_eph->kepler.inc_dot = getbits(buff, *bit, 14);
  *bit += 14;
  msg_eph->kepler.iode = getbitu(buff, *bit, 8);
  *bit += 8;
  msg_eph->kepler.toc = getbitu(buff, *bit, 16);
  *bit += 16;
  msg_eph->kepler.af2 = getbits(buff, *bit, 8);
  *bit += 8;
  msg_eph->kepler.af1 = getbits(buff, *bit, 16);
  *bit += 16;
  msg_eph->kepler.af0 = getbits(buff, *bit, 22);
  *bit += 22;
  msg_eph->kepler.iodc = getbitu(buff, *bit, 10);
  *bit += 10;
  msg_eph->kepler.crs = getbits(buff, *bit, 16);
  *bit += 16;
  msg_eph->kepler.dn = getbits(buff, *bit, 16);
  *bit += 16;
  msg_eph->kepler.m0 = getbits(buff, *bit, 32);
  *bit += 32;
  msg_eph->kepler.cuc = getbits(buff, *bit, 16);
  *bit += 16;
  msg_eph->kepler.ecc = getbitu(buff, *bit, 32);
  *bit += 32;
  msg_eph->kepler.cus = getbits(buff, *bit, 16);
  *bit += 16;
  msg_eph->kepler.sqrta = getbitu(buff, *bit, 32);
  *bit += 32;
  msg_eph->kepler.toc = getbitu(buff, *bit, 16);
  *bit += 16;
  msg_eph->kepler.cic = getbits(buff, *bit, 16);
  *bit += 16;
  msg_eph->kepler.omega0 = getbits(buff, *bit, 32);
  *bit += 32;
  msg_eph->kepler.cis = getbits(buff, *bit, 16);
  *bit += 16;
  msg_eph->kepler.inc = getbits(buff, *bit, 32);
  *bit += 32;
  msg_eph->kepler.crc = getbits(buff, *bit, 32);
  *bit += 32;
  msg_eph->kepler.w = getbits(buff, *bit, 32);
  *bit += 32;
  msg_eph->kepler.omegadot = getbits(buff, *bit, 24);
  *bit += 24;
  msg_eph->kepler.tgd_gps_s = getbits(buff, *bit, 8);
  *bit += 8;
  msg_eph->health_bits = getbitu(buff, *bit, 6);
  bit += 6;
  /* L2 data bit */ getbitu(buff, *bit, 1);
  *bit += 1;
  msg_eph->fit_interval = getbitu(buff, *bit, 1);
  *bit += 1;

  return RC_OK;
}

/** Decode an RTCMv3 GPS Ephemeris Message
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 *          - RC_INVALID_MESSAGE : Cell mask too large or invalid TOW
 */
rtcm3_rc decode_glo_eph(const uint8_t buff[], uint16_t *bit, rtcm_msg_eph *msg_eph) {
  msg_eph->sat_id = getbitu(buff, *bit, 6);
  *bit += 6;
  msg_eph->wn = getbitu(buff, *bit, 10);
  *bit += 10;
  msg_eph->ura = getbitu(buff, *bit, 4);
  *bit += 4;
  /*uint8_t l2_code = */ getbitu(buff, *bit, 2);
  *bit += 2;
  msg_eph->kepler.inc_dot = getbits(buff, *bit, 14);
  *bit += 14;
  msg_eph->kepler.iode = getbitu(buff, *bit, 8);
  *bit += 8;
  msg_eph->kepler.toc = getbitu(buff, *bit, 16);
  *bit += 16;
  msg_eph->kepler.af2 = getbits(buff, *bit, 8);
  *bit += 8;
  msg_eph->kepler.af1 = getbits(buff, *bit, 16);
  *bit += 16;
  msg_eph->kepler.af0 = getbits(buff, *bit, 22);
  *bit += 22;
  msg_eph->kepler.iodc = getbitu(buff, *bit, 10);
  *bit += 10;
  msg_eph->kepler.crs = getbits(buff, *bit, 16);
  *bit += 16;
  msg_eph->kepler.dn = getbits(buff, *bit, 16);
  *bit += 16;
  msg_eph->kepler.m0 = getbits(buff, *bit, 32);
  *bit += 32;
  msg_eph->kepler.cuc = getbits(buff, *bit, 16);
  *bit += 16;
  msg_eph->kepler.ecc = getbitu(buff, *bit, 32);
  *bit += 32;
  msg_eph->kepler.cus = getbits(buff, *bit, 16);
  *bit += 16;
  msg_eph->kepler.sqrta = getbitu(buff, *bit, 32);
  *bit += 32;
  msg_eph->kepler.toc = getbitu(buff, *bit, 16);
  *bit += 16;
  msg_eph->kepler.cic = getbits(buff, *bit, 16);
  *bit += 16;
  msg_eph->kepler.omega0 = getbits(buff, *bit, 32);
  *bit += 32;
  msg_eph->kepler.cis = getbits(buff, *bit, 16);
  *bit += 16;
  msg_eph->kepler.inc = getbits(buff, *bit, 32);
  *bit += 32;
  msg_eph->kepler.crc = getbits(buff, *bit, 32);
  *bit += 32;
  msg_eph->kepler.w = getbits(buff, *bit, 32);
  *bit += 32;
  msg_eph->kepler.omegadot = getbits(buff, *bit, 24);
  *bit += 24;
  msg_eph->kepler.tgd_gps_s = getbits(buff, *bit, 8);
  *bit += 8;
  msg_eph->health_bits = getbitu(buff, *bit, 6);
  bit += 6;
  /* L2 data bit */ getbitu(buff, *bit, 1);
  *bit += 1;
  msg_eph->fit_interval = getbitu(buff, *bit, 1);
  *bit += 1;

  return RC_OK;
}
