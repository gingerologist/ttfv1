/*
 * control.h
 *
 *  Created on: Apr 23, 2024
 *      Author: ma
 */

#ifndef INC_PROFILE_H_
#define INC_PROFILE_H_

#include <stdint.h>
#include <assert.h>

// this board has 4 group of pads
// each group has 9 pads
// both pgcfg_a and pgcfg_b use 18
typedef struct {
	uint32_t pgcfg_a[4];
	uint32_t pgcfg_b[4];
	uint32_t duration_a_sec;
	uint32_t duration_b_sec;
} profile_t;

// suppress the IDE syntax error (yellow mark) by
// Project --> Properties --> C/C++ General --> Preprocessor Include Paths, Macros etc.
// On the Entries tab, select CDT User Setting Entries, then click Add
//
//   _Static_assert(a,b)
//
_Static_assert(sizeof(profile_t) == 40, "profile_t size not 40");

typedef union
{
  uint16_t data;
  struct __attribute__((packed)) {
    unsigned int rsvd:6;
    unsigned int middle:2;
    unsigned int left:2;
    unsigned int bottom:2;
    unsigned int right:2;
    unsigned int top:2;
  };
} pad_star_t;

_Static_assert(sizeof(pad_star_t) == 2, "pad_star_t size not 2");

typedef struct
{
  pad_star_t cfg[3][3];   // cfg [row][col]
} pgcfg_v2_t;

_Static_assert(sizeof(pgcfg_v2_t) == 18, "pad_star_t size not 2");

// in profile_v2_t
typedef struct __attribute__((packed)) {
  pgcfg_v2_t pgcfg_a;
  pgcfg_v2_t pgcfg_b;
  uint32_t duration_a_sec;
  uint32_t duration_b_sec;
  uint32_t level;
} profile_v2_t;

_Static_assert(sizeof(profile_v2_t) == 48, "profile_t size not 40");

void print_profile(int index);
void print_profile_v2(void);

void do_profile_by_key(int profile_index);
void do_profile_blink(void);
profile_t get_profile(int index);
void set_profile(int index,
				uint32_t pgcfg_a[4],
				uint32_t *duration_a_sec,
				uint32_t pgcfg_b[4],
				uint32_t *duration_b_sec);

#endif /* INC_PROFILE_H_ */
