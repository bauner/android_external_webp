// Copyright 2012 Google Inc. All Rights Reserved.
//
// Use of this source code is governed by a BSD-style license
// that can be found in the COPYING file in the root of the source
// tree. An additional intellectual property rights grant can be found
// in the file PATENTS. All contributing project authors may
// be found in the AUTHORS file in the root of the source tree.
// -----------------------------------------------------------------------------
//
// Author: Jyrki Alakuijala (jyrki@google.com)
//

#ifndef WEBP_ENC_BACKWARD_REFERENCES_H_
#define WEBP_ENC_BACKWARD_REFERENCES_H_

#include <assert.h>
#include <stdlib.h>
#include "webp/types.h"
#include "webp/format_constants.h"

#ifdef __cplusplus
extern "C" {
#endif

// The spec allows 11, we use 9 bits to reduce memory consumption in encoding.
// Having 9 instead of 11 only removes about 0.25 % of compression density.
#define MAX_COLOR_CACHE_BITS 9

// Max ever number of codes we'll use:
#define PIX_OR_COPY_CODES_MAX \
    (NUM_LITERAL_CODES + NUM_LENGTH_CODES + (1 << MAX_COLOR_CACHE_BITS))

// -----------------------------------------------------------------------------
// PrefixEncode()

// use GNU builtins where available.
#if defined(__GNUC__) && \
    ((__GNUC__ == 3 && __GNUC_MINOR__ >= 4) || __GNUC__ >= 4)
static WEBP_INLINE int BitsLog2Floor(uint32_t n) {
  assert(n != 0);
  return 31 ^ __builtin_clz(n);
}
#elif defined(_MSC_VER) && (defined(_M_X64) || defined(_M_IX86))
#include <intrin.h>
#pragma intrinsic(_BitScanReverse)

static WEBP_INLINE int BitsLog2Floor(uint32_t n) {
  unsigned long first_set_bit;
  assert(n != 0);
  _BitScanReverse(&first_set_bit, n);
  return first_set_bit;
}
#else
// Returns (int)floor(log2(n)). n must be > 0.
static WEBP_INLINE int BitsLog2Floor(uint32_t n) {
  int log = 0;
  uint32_t value = n;
  int i;

  assert(n != 0);
  for (i = 4; i >= 0; --i) {
    const int shift = (1 << i);
    const uint32_t x = value >> shift;
    if (x != 0) {
      value = x;
      log += shift;
    }
  }
  return log;
}
#endif

static WEBP_INLINE int VP8LBitsLog2Ceiling(uint32_t n) {
  const int log_floor = BitsLog2Floor(n);
  if (n == (n & ~(n - 1)))  // zero or a power of two.
    return log_floor;
  else
    return log_floor + 1;
}

// Splitting of distance and length codes into prefixes and
// extra bits. The prefixes are encoded with an entropy code
// while the extra bits are stored just as normal bits.
static WEBP_INLINE void PrefixEncode(int distance, int* const code,
                                     int* const extra_bits_count,
                                     int* const extra_bits_value) {
  if (distance > 2) {  // Collect the two most significant bits.
    const int highest_bit = BitsLog2Floor(--distance);
    const int second_highest_bit = (distance >> (highest_bit - 1)) & 1;
    *extra_bits_count = highest_bit - 1;
    *extra_bits_value = distance & ((1 << *extra_bits_count) - 1);
    *code = 2 * highest_bit + second_highest_bit;
  } else {
    *extra_bits_count = 0;
    *extra_bits_value = 0;
    *code = (distance == 2) ? 1 : 0;
  }
}

// -----------------------------------------------------------------------------
// PixOrCopy

enum Mode {
  kLiteral,
  kCacheIdx,
  kCopy,
  kNone
};

typedef struct {
  // mode as uint8_t to make the memory layout to be exactly 8 bytes.
  uint8_t mode;
  uint16_t len;
  uint32_t argb_or_distance;
} PixOrCopy;

static WEBP_INLINE PixOrCopy PixOrCopyCreateCopy(uint32_t distance,
                                                 uint16_t len) {
  PixOrCopy retval;
  retval.mode = kCopy;
  retval.argb_or_distance = distance;
  retval.len = len;
  return retval;
}

static WEBP_INLINE PixOrCopy PixOrCopyCreateCacheIdx(int idx) {
  PixOrCopy retval;
  assert(idx >= 0);
  assert(idx < (1 << MAX_COLOR_CACHE_BITS));
  retval.mode = kCacheIdx;
  retval.argb_or_distance = idx;
  retval.len = 1;
  return retval;
}

static WEBP_INLINE PixOrCopy PixOrCopyCreateLiteral(uint32_t argb) {
  PixOrCopy retval;
  retval.mode = kLiteral;
  retval.argb_or_distance = argb;
  retval.len = 1;
  return retval;
}

static WEBP_INLINE int PixOrCopyIsLiteral(const PixOrCopy* const p) {
  return (p->mode == kLiteral);
}

static WEBP_INLINE int PixOrCopyIsCacheIdx(const PixOrCopy* const p) {
  return (p->mode == kCacheIdx);
}

static WEBP_INLINE int PixOrCopyIsCopy(const PixOrCopy* const p) {
  return (p->mode == kCopy);
}

static WEBP_INLINE uint32_t PixOrCopyLiteral(const PixOrCopy* const p,
                                             int component) {
  assert(p->mode == kLiteral);
  return (p->argb_or_distance >> (component * 8)) & 0xff;
}

static WEBP_INLINE uint32_t PixOrCopyLength(const PixOrCopy* const p) {
  return p->len;
}

static WEBP_INLINE uint32_t PixOrCopyArgb(const PixOrCopy* const p) {
  assert(p->mode == kLiteral);
  return p->argb_or_distance;
}

static WEBP_INLINE uint32_t PixOrCopyCacheIdx(const PixOrCopy* const p) {
  assert(p->mode == kCacheIdx);
  assert(p->argb_or_distance < (1U << MAX_COLOR_CACHE_BITS));
  return p->argb_or_distance;
}

static WEBP_INLINE uint32_t PixOrCopyDistance(const PixOrCopy* const p) {
  assert(p->mode == kCopy);
  return p->argb_or_distance;
}

// -----------------------------------------------------------------------------
// VP8LHashChain

#define HASH_BITS 18
#define HASH_SIZE (1 << HASH_BITS)

typedef struct VP8LHashChain VP8LHashChain;
struct VP8LHashChain {
  // Stores the most recently added position with the given hash value.
  int32_t hash_to_first_index_[HASH_SIZE];
  // chain_[pos] stores the previous position with the same hash value
  // for every pixel in the image.
  int32_t* chain_;
  // This is the maximum size of the hash_chain that can be constructed.
  // Typically this is the pixel count (width x height) for a given image.
  int size_;
};

// Must be called first, to set size.
int VP8LHashChainInit(VP8LHashChain* const p, int size);
void VP8LHashChainClear(VP8LHashChain* const p);  // release memory

// -----------------------------------------------------------------------------
// VP8LBackwardRefs (block-based backward-references storage)

// maximum number of reference blocks the image will be segmented into
#define MAX_REFS_BLOCK_PER_IMAGE 16

typedef struct PixOrCopyBlock PixOrCopyBlock;   // forward declaration
typedef struct VP8LBackwardRefs VP8LBackwardRefs;

// Container for blocks chain
struct VP8LBackwardRefs {
  int block_size_;               // common block-size
  int error_;                    // set to true if some memory error occurred
  PixOrCopyBlock* refs_;         // list of currently used blocks
  PixOrCopyBlock** tail_;        // for list recycling
  PixOrCopyBlock* free_blocks_;  // free-list
  PixOrCopyBlock* last_block_;   // used for adding new refs (internal)
};

// Initialize the object. 'block_size' is the common block size to store
// references (typically, width * height / MAX_REFS_BLOCK_PER_IMAGE).
void VP8LBackwardRefsInit(VP8LBackwardRefs* const refs, int block_size);
// Release memory for backward references.
void VP8LBackwardRefsClear(VP8LBackwardRefs* const refs);
// Copies the 'src' backward refs to the 'dst'. Returns 0 in case of error.
int VP8LBackwardRefsCopy(const VP8LBackwardRefs* const src,
                         VP8LBackwardRefs* const dst);

// Cursor for iterating on references content
typedef struct {
  // public:
  PixOrCopy* cur_pos;           // current position
  // private:
  PixOrCopyBlock* cur_block_;   // current block in the refs list
  const PixOrCopy* last_pos_;   // sentinel for switching to next block
} VP8LRefsCursor;

// Returns a cursor positioned at the beginning of the references list.
VP8LRefsCursor VP8LRefsCursorInit(const VP8LBackwardRefs* const refs);
// Returns true if cursor is pointing at a valid position.
static WEBP_INLINE int VP8LRefsCursorOk(const VP8LRefsCursor* const c) {
  return (c->cur_pos != NULL);
}
// Move to next block of references. Internal, not to be called directly.
void VP8LRefsCursorNextBlock(VP8LRefsCursor* const c);
// Move to next position, or NULL. Should not be called if !VP8LRefsCursorOk().
static WEBP_INLINE void VP8LRefsCursorNext(VP8LRefsCursor* const c) {
  assert(c != NULL);
  assert(VP8LRefsCursorOk(c));
  if (++c->cur_pos == c->last_pos_) VP8LRefsCursorNextBlock(c);
}

// -----------------------------------------------------------------------------
// Main entry points

// Evaluates best possible backward references for specified quality.
// Further optimize for 2D locality if use_2d_locality flag is set.
// The return value is the pointer to the best of the two backward refs viz,
// refs[0] or refs[1].
VP8LBackwardRefs* VP8LGetBackwardReferences(
    int width, int height, const uint32_t* const argb, int quality,
    int cache_bits, int use_2d_locality, VP8LHashChain* const hash_chain,
    VP8LBackwardRefs refs[2]);

// Produce an estimate for a good color cache size for the image.
int VP8LCalculateEstimateForCacheSize(const uint32_t* const argb,
                                      int xsize, int ysize, int quality,
                                      VP8LHashChain* const hash_chain,
                                      VP8LBackwardRefs* const ref,
                                      int* const best_cache_bits);

#ifdef __cplusplus
}
#endif

#endif  // WEBP_ENC_BACKWARD_REFERENCES_H_
