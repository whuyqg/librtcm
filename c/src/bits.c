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

#include "bits.h"

static const uint8_t bitn[16] = {
    0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4};

/** Get bit field from buffer as an unsigned integer.
 * Unpacks `len` bits at bit position `pos` from the start of the buffer.
 * Maximum bit field length is 32 bits, i.e. `len <= 32`.
 *
 * \param buff
 * \param pos Position in buffer of start of bit field in bits.
 * \param len Length of bit field in bits.
 * \return Bit field as an unsigned value.
 */
uint32_t getbitu(const uint8_t *buff, uint32_t pos, uint8_t len) {
  uint32_t bits = 0;

  for (uint32_t i = pos; i < pos + len; i++) {
    bits = (bits << 1) + ((buff[i / 8] >> (7 - i % 8)) & 1u);
  }

  return bits;
}

/** Get bit field from buffer as an unsigned long integer.
 * Unpacks `len` bits at bit position `pos` from the start of the buffer.
 * Maximum bit field length is 64 bits, i.e. `len <= 64`.
 *
 * \param buff
 * \param pos Position in buffer of start of bit field in bits.
 * \param len Length of bit field in bits.
 * \return Bit field as an unsigned value.
 */
uint64_t getbitul(const uint8_t *buff, uint32_t pos, uint8_t len) {
  uint64_t bits = 0;

  for (uint32_t i = pos; i < pos + len; i++) {
    bits = (bits << 1) + ((buff[i / 8] >> (7 - i % 8)) & 1u);
  }

  return bits;
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
int32_t getbits(const uint8_t *buff, uint32_t pos, uint8_t len) {
  int32_t bits = (int32_t)getbitu(buff, pos, len);

  /* Sign extend, taken from:
   * http://graphics.stanford.edu/~seander/bithacks.html#VariableSignExtend
   */
  int32_t m = 1u << (len - 1);
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
int64_t getbitsl(const uint8_t *buff, uint32_t pos, uint8_t len) {
  int64_t bits = (int64_t)getbitul(buff, pos, len);

  /* Sign extend, taken from:
   * http://graphics.stanford.edu/~seander/bithacks.html#VariableSignExtend
   */
  int64_t m = ((uint64_t)1) << (len - 1);
  return (bits ^ m) - m;
}

/** Set bit field in buffer from an unsigned integer.
 * Packs `len` bits into bit position `pos` from the start of the buffer.
 * Maximum bit field length is 32 bits, i.e. `len <= 32`.
 *
 * \param buff
 * \param pos Position in buffer of start of bit field in bits.
 * \param len Length of bit field in bits.
 * \param data Unsigned integer to be packed into bit field.
 */
void setbitu(uint8_t *buff, uint32_t pos, uint32_t len, uint32_t data) {
  uint32_t mask = 1u << (len - 1);

  if (len <= 0 || 32 < len) return;

  for (uint32_t i = pos; i < pos + len; i++, mask >>= 1) {
    if (data & mask)
      buff[i / 8] |= 1u << (7 - i % 8);
    else
      buff[i / 8] &= ~(1u << (7 - i % 8));
  }
}

/** Set bit field in buffer from an unsigned integer.
 * Packs `len` bits into bit position `pos` from the start of the buffer.
 * Maximum bit field length is 32 bits, i.e. `len <= 32`.
 *
 * \param buff
 * \param pos Position in buffer of start of bit field in bits.
 * \param len Length of bit field in bits.
 * \param data Unsigned integer to be packed into bit field.
 */
void setbitul(uint8_t *buff, uint32_t pos, uint32_t len, uint64_t data) {
  uint64_t mask = ((uint64_t)1) << (len - 1);

  if (len <= 0 || 64 < len) return;

  for (uint32_t i = pos; i < pos + len; i++, mask >>= 1) {
    if (data & mask)
      buff[i / 8] |= ((uint64_t)1) << (7 - i % 8);
    else
      buff[i / 8] &= ~(((uint64_t)1) << (7 - i % 8));
  }
}

/** Set bit field in buffer from a signed integer.
 * Packs `len` bits into bit position `pos` from the start of the buffer.
 * Maximum bit field length is 32 bits, i.e. `len <= 32`.
 *
 * \param buff
 * \param pos Position in buffer of start of bit field in bits.
 * \param len Length of bit field in bits.
 * \param data Signed integer to be packed into bit field.
 */
void setbits(uint8_t *buff, uint32_t pos, uint32_t len, int32_t data) {
  setbitu(buff, pos, len, (uint32_t)data);
}

/** Set bit field in buffer from a signed integer.
 * Packs `len` bits into bit position `pos` from the start of the buffer.
 * Maximum bit field length is 32 bits, i.e. `len <= 32`.
 *
 * \param buff
 * \param pos Position in buffer of start of bit field in bits.
 * \param len Length of bit field in bits.
 * \param data Signed integer to be packed into bit field.
 */
void setbitsl(uint8_t *buff, uint32_t pos, uint32_t len, int64_t data) {
  setbitul(buff, pos, len, (uint64_t)data);
}

/**
 * Count number of bits set to certain value in 64 bits word
 *
 * \param[in]     v      input 64 bits word to count bits in
 * \param[in]     bv     1 or 0 - which value to count
 *
 * \return        Number of bits set to one or zero.
 */
uint8_t count_bits_u64(uint64_t v, uint8_t bv) {
  uint8_t r = 0;
  for (int i = 0; i < 16; i++) r += bitn[(v >> (i * 4)) & 0xf];
  return bv == 1 ? r : 64 - r;
}

/**
 * Count number of bits set to certain value in 32 bits word
 *
 * \param[in]     v      input 32 bits word to count bits in
 * \param[in]     bv     1 or 0 - which value to count
 *
 * \return        Number of bits set to one or zero.
 */
uint8_t count_bits_u32(uint32_t v, uint8_t bv) {
  uint8_t r = 0;
  for (int i = 0; i < 8; i++) r += bitn[(v >> (i * 4)) & 0xf];
  return bv == 1 ? r : 32 - r;
}
