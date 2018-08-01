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

#ifndef PIKSI_BUILDROOT_RTCM3_MESSAGES_H_H
#define PIKSI_BUILDROOT_RTCM3_MESSAGES_H_H

#include <constants.h>
#include <stdbool.h>
#include <stdint.h>

typedef enum { L1_FREQ, L2_FREQ, NUM_FREQS } freq_enum;

typedef enum {
  MSM_UNKNOWN = 0,
  MSM1,
  MSM2,
  MSM3,
  MSM4,
  MSM5,
  MSM6,
  MSM7
} msm_enum;

/** Constellation identifier. */
typedef enum constellation_e {
  CONSTELLATION_INVALID = -1,
  CONSTELLATION_GPS,
  CONSTELLATION_SBAS,
  CONSTELLATION_GLO,
  CONSTELLATION_BDS2,
  CONSTELLATION_QZS,
  CONSTELLATION_GAL,
  CONSTELLATION_COUNT,
} constellation_t;

/** Code identifier (from libswiftnav-private) */
typedef enum code_e {
  CODE_INVALID = -1,
  CODE_GPS_L1CA = 0,
  CODE_GPS_L2CM = 1,
  CODE_SBAS_L1CA = 2,
  CODE_GLO_L1OF = 3,
  CODE_GLO_L2OF = 4,
  CODE_GPS_L1P = 5,
  CODE_GPS_L2P = 6,
  CODE_GPS_L2CL = 7,
  CODE_GPS_L2CX = 8, /* combined L2C tracking */
  CODE_GPS_L5I = 9,
  CODE_GPS_L5Q = 10,
  CODE_GPS_L5X = 11,  /* combined L5 tracking */
  CODE_BDS2_B11 = 12, /* data channel at 1526 * 1.023 MHz */
  CODE_BDS2_B2 = 13,  /* data channel at 1180 * 1.023 MHz */
  CODE_GAL_E1B = 14,  /* data channel at E1 (1540 * 1.023 MHz) */
  CODE_GAL_E1C = 15,  /* pilot channel at E1 */
  CODE_GAL_E1X = 16,  /* combined tracking on E1 */
  CODE_GAL_E6B = 17,
  CODE_GAL_E6C = 18,
  CODE_GAL_E6X = 19, /* combined tracking on E6 */
  CODE_GAL_E7I = 20,
  CODE_GAL_E7Q = 21,
  CODE_GAL_E7X = 22, /* combined tracking on E5b */
  CODE_GAL_E8 = 23,  /* E5 AltBOC tracking */
  CODE_GAL_E5I = 24,
  CODE_GAL_E5Q = 25,
  CODE_GAL_E5X = 26, /* combined tracking on E5a */
  CODE_QZS_L1CA = 27,
  CODE_QZS_L2CM = 28,
  CODE_QZS_L2CL = 29,
  CODE_QZS_L2CX = 30,
  CODE_QZS_L5I = 31,
  CODE_QZS_L5Q = 32,
  CODE_QZS_L5X = 33,
  CODE_COUNT
} code_t;

/* return codes for the decoders */
typedef enum rtcm3_rc_e {
  RC_OK = 0,
  RC_MESSAGE_TYPE_MISMATCH = -1,
  RC_INVALID_MESSAGE = -2
} rtcm3_rc;

typedef struct {
  uint16_t msg_num; /* Msg Num DF002 uint16 12*/
  uint16_t stn_id;  /* Station Id DF003 uint16 12*/
  uint32_t tow_ms;  /* GPS/GLO time of week DF004/DF034 uint32 30/27 */
  uint8_t sync;     /* Syncronous flag DF005 bit(1) 1 */
  uint8_t n_sat;    /* Number of satellites DF006 uint8 5 */
  uint8_t div_free; /* Divergance free flag DF007 bit(1) 1 */
  uint8_t smooth;   /* GPS Smoothing Interval DF008 bit(3) 3 */
} rtcm_obs_header;

#define MSM_SATELLITE_MASK_SIZE 64
#define MSM_SIGNAL_MASK_SIZE 32
typedef struct {
  uint16_t msg_num;  /* Msg Num DF002 uint16 12*/
  uint16_t stn_id;   /* Station Id DF003 uint16 12*/
  uint32_t tow_ms;   /* System-specific epoch time uint32 30 */
  uint8_t multiple;  /* Multiple Message Bit DF393 bit(1) 1 */
  uint8_t iods;      /* Issue of Data Station DF409 uint8 3 */
  uint8_t reserved;  /* Reserved DF001 bit(7) 7 */
  uint8_t steering;  /* Clock Steering Indicator DF411 uint2 2 */
  uint8_t ext_clock; /* External Clock Indicator DF412 uint2 2 */
  uint8_t div_free;  /* Divergance free flag DF417 bit(1) 1 */
  uint8_t smooth;    /* GPS Smoothing Interval DF418 bit(3) 3 */
  /* GNSS Satellite Mask DF394 bit(64) 64 */
  bool satellite_mask[MSM_SATELLITE_MASK_SIZE];
  /* GNSS Signal Mask DF395 bit(32) 32 */
  bool signal_mask[MSM_SIGNAL_MASK_SIZE];
  /* GNSS Cell Mask DF396 bit(X) (X<=64) */
  bool cell_mask[MSM_MAX_CELLS];
} rtcm_msm_header;

typedef union {
  uint8_t data;
  struct {
    uint8_t valid_pr : 1;
    uint8_t valid_cp : 1;
    uint8_t valid_cnr : 1;
    uint8_t valid_lock : 1;
    uint8_t valid_dop : 1;
  };
} flag_bf;

typedef struct {
  uint8_t code;
  double pseudorange;
  double carrier_phase;
  uint32_t lock;
  double cnr;
  flag_bf flags;

} rtcm_freq_data;

typedef struct {
  uint8_t svId;
  uint8_t fcn;
  rtcm_freq_data obs[NUM_FREQS];
} rtcm_sat_data;

typedef struct {
  uint8_t sat_info;
  double rough_range_m;
  double rough_range_rate_m_s;
} rtcm_msm_sat_data;

typedef struct {
  double pseudorange_m;
  double carrier_phase_cyc;
  double lock_time_s;
  bool hca_indicator;
  double cnr;
  flag_bf flags;
  double range_rate_Hz;
} rtcm_msm_signal_data;

typedef struct {
  rtcm_obs_header header;
  rtcm_sat_data sats[RTCM_MAX_SATS];
} rtcm_obs_message;

typedef struct {
  rtcm_msm_header header;
  rtcm_msm_sat_data sats[RTCM_MAX_SATS];
  rtcm_msm_signal_data signals[MSM_MAX_CELLS];
} rtcm_msm_message;

typedef struct {
  uint16_t stn_id;
  uint8_t ITRF;        /* Reserved for ITRF Realization Year DF021 uint6 6 */
  uint8_t GPS_ind;     /* GPS Indicator DF022 bit(1) 1 */
  uint8_t GLO_ind;     /* GLONASS Indicator DF023 bit(1) 1 */
  uint8_t GAL_ind;     /* Reserved for Galileo Indicator DF024 bit(1) 1 */
  uint8_t ref_stn_ind; /* Reference-Station Indicator DF141 bit(1) 1 */
  double arp_x;        /* Antenna Reference Point ECEF-X DF025 int38 38 */
  uint8_t osc_ind;     /* Single Receiver Oscillator Indicator DF142 bit(1) 1 */
  uint8_t reserved;    /* Reserved DF001 bit(1) 1 */
  double arp_y;        /* Antenna Reference Point ECEF-Y DF026 int38 38 */
  uint8_t quart_cycle_ind; /* Quarter Cycle Indicator DF364 bit(2) 2 */
  double arp_z;            /* Antenna Reference Point ECEF-Z DF027 int38 38 */
} rtcm_msg_1005;

typedef struct {
  rtcm_msg_1005 msg_1005;
  double ant_height; /* Antenna Height DF028 uint16 16 */
} rtcm_msg_1006;

typedef struct {
  uint16_t stn_id;    /* Reference Station ID DF003 uint12 12 */
  uint8_t desc_count; /* Descriptor Counter N DF029 uint8 8 N <= 31 */
  char desc[32];      /* Antenna Descriptor DF030 char8(N) 8*N */
  uint8_t ant_id;     /* Antenna Setup ID DF031 uint8 8 */
} rtcm_msg_1007;

typedef struct {
  rtcm_msg_1007 msg_1007;
  uint8_t serial_count; /* Serial Number Counter M DF032 uint8 8 M <= 31 */
  char serial_num[32];  /* Antenna Serial Number DF033 char8(M) 8*M */
} rtcm_msg_1008;

#define RTCM_1029_MAX_CODE_UNITS (255u)
typedef struct {
  uint16_t stn_id;
  uint16_t mjd_num;
  uint32_t utc_sec_of_day;
  uint8_t unicode_chars;
  uint8_t utf8_code_units_n;
  uint8_t utf8_code_units[RTCM_1029_MAX_CODE_UNITS];
} rtcm_msg_1029;

typedef struct {
  uint16_t stn_id;
  uint8_t antenna_desc_counter;
  char antenna_descriptor[32];
  uint8_t antenna_setup_ID;
  uint8_t antenna_serial_num_counter;
  char antenna_serial_num[32];
  uint8_t rcv_descriptor_counter;
  char rcv_descriptor[32];
  uint8_t rcv_fw_counter;
  char rcv_fw_version[32];
  uint8_t rcv_serial_num_counter;
  char rcv_serial_num[32];
} rtcm_msg_1033;

typedef struct {
  uint16_t stn_id;
  uint8_t bias_indicator;
  uint8_t fdma_signal_mask;
  double L1_CA_cpb_meter;
  double L1_P_cpb_meter;
  double L2_CA_cpb_meter;
  double L2_P_cpb_meter;
} rtcm_msg_1230;

/** Structure containing the ephemeris for one satellite.
 * (TODO) Anthony - this was lifted with minor alteratons from LSNP ephemeris.c,
 * if we split this out to be a common repo, we should submodule it in here
 */

typedef struct {
  union {
    int8_t tgd_gps_s;    /**< GPS TGD  */
    double tgd_qzss_s;   /**< QZSS TGD */
    double tgd_bds_s[2]; /**< tgd_bds_s[0] = BDS TGD1,
                              tgd_bds_s[1] = BDS TGD2 */
    double tgd_gal_s[2]; /**< tgd_gal_s[0] = GAL E5a/E1 BGD,
                              tgd_gal_s[1] = GAL E5b/E1 BGD*/
  };
  int32_t crc;      /**< Amplitude of the cosine harmonic correction term
                        to the orbit radius [m] */
  int16_t crs;      /**< Amplitude of the sine harmonic correction term
                        to the orbit radius [m] */
  int16_t cuc;      /**< Amplitude of the cosine harmonic correction term
                        to the argument of latitude [rad] */
  int16_t cus;      /**< Amplitude of the sine harmonic correction term
                        to the argument of latitude [rad] */
  int16_t cic;      /**< Amplitude of the cosine harmonic correction term
                        to the angle of inclination [rad] */
  int16_t cis;      /**< Amplitude of the sine harmonic correction term
                        to the angle of inclination [rad] */
  int16_t dn;       /**< Mean motion difference from computed value
                        [semi-circles/s] */
  int32_t m0;       /**< Mean anomaly at reference time [semi-circles] */
  uint32_t ecc;      /**< Eccentricity, dimensionless */
  uint32_t sqrta;    /**< Square root of the semi-major axis [sqrt(m)] */
  int32_t omega0;   /**< Longitude of ascending node
                        of orbit plane at weekly epoch [semi-circles] */
  int32_t omegadot; /**< Rate of right ascension [semi-circles/s] */
  int32_t w;        /**< Argument of perigee [semi-circles] */
  int32_t inc;      /**< Inclindation angle at reference time [semi-circles] */
  int16_t inc_dot;  /**< Rate of inclination angle [semi-circles/s] */
  int32_t af0;      /**< Time offset of the sat clock [s] **/
  int16_t af1;      /**< Drift of the sat clock [s/s] **/
  int8_t af2;      /**< Acceleration of the sat clock [s/s^2] **/
  uint16_t toc;  /**< Reference time of clock. */
  uint16_t iodc;        /**< Issue of data clock. */
  uint16_t iode;        /**< Issue of data ephemeris. */
} ephemeris_kepler_t;

/** Structure containing the SBAS ephemeris for one satellite. */
typedef struct {
  double pos[3]; /**< Position of the GEO at time toe [m] */
  double vel[3]; /**< velocity of the GEO at time toe [m/s] */
  double acc[3]; /**< velocity of the GEO at time toe [m/s^2] */
  double a_gf0;  /**< Time offset of the GEO clock w.r.t. SNT [s] */
  double a_gf1;  /**< Drift of the GEO clock w.r.t. SNT [s/s] */
} ephemeris_xyz_t;

/** Structure containing the GLONASS ephemeris for one satellite. */
typedef struct {
  double gamma;  /**< Relative deviation of predicted carrier frequency
                      from nominal value, dimensionless */
  double tau;    /**< Correction to the SV time [s]*/
  double d_tau;  /**< Equipment delay between L1 and L2 [s] */
  double pos[3]; /**< Position of the SV at tb in PZ-90.02 coordinates
                      system [m] */
  double vel[3]; /**< Velocity vector of the SV at tb in PZ-90.02
                      coordinates system [m/s] */
  double acc[3]; /**< Acceleration vector of the SV at tb in PZ-90.02
                      coordinates system [m/s^2] */
  uint16_t fcn;       /**< Frequency slot associated with the GLO SV */
  uint8_t iod;        /**< Issue of ephemeris data */
} ephemeris_glo_t;

/** Structure containing the ephemeris for one satellite. */
typedef struct {
  uint8_t sat_id;       /**< Signal ID. */
  constellation_t constellation; /**< Constellation > */
  uint16_t wn;       /**< Reference wn of ephemeris */
  uint16_t ura;         /**< User range accuracy [m] */
  uint32_t fit_interval;  /**< Curve fit interval [s] */
  uint8_t valid;          /**< Ephemeris is valid. */
  uint8_t health_bits;    /**< Satellite health status. */
  union {
    ephemeris_kepler_t kepler; /**< Parameters specific to GPS. */
    ephemeris_xyz_t xyz;       /**< Parameters specific to SBAS. */
    ephemeris_glo_t glo;       /**< Parameters specific to GLONASS. */
  };
} rtcm_msg_eph;

#endif /* PIKSI_BUILDROOT_RTCM3_MESSAGES_H_H */
