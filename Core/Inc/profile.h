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

#define NUM_OF_PROFILES                 16
#define STOP_PROFILE_INDEX              (NUM_OF_PROFILES)
#define DDBF_PROFILE_INDEX              (STOP_PROFILE_INDEX + 1)
#define CURR_PROFILE_INDEX              (DDBF_PROFILE_INDEX + 1)
#define NEXT_PROFILE_INDEX              (CURR_PROFILE_INDEX + 1)
#define LAST_PROFILE_INDEX              (NEXT_PROFILE_INDEX)
#define NUM_OF_ALL_PROFILES             (LAST_PROFILE_INDEX + 1)

// suppress the IDE syntax error (yellow mark) by
// Project --> Properties --> C/C++ General --> Preprocessor Include Paths, Macros etc.
// On the Entries tab, select CDT User Setting Entries, then click Add
//
//   _Static_assert(a,b)
//

// each rowcfg_t packs 15 pads in total, into one word.
typedef union
{
  uint32_t word;
  struct __attribute__((packed)) {
    unsigned int d____:2;
    unsigned int c_bot:2;
    unsigned int c_rgt:2;
    unsigned int c_mid:2;
    unsigned int c_lft:2;
    unsigned int c_top:2;
    unsigned int b_bot:2;
    unsigned int b_rgt:2;
    unsigned int b_mid:2;
    unsigned int b_lft:2;
    unsigned int b_top:2;
    unsigned int a_bot:2;
    unsigned int a_rgt:2;
    unsigned int a_mid:2;
    unsigned int a_lft:2;
    unsigned int a_top:2;
  };
} rowpads_t;

_Static_assert(sizeof(rowpads_t) == 4, "rowpads_t size not 4");

// three rows of pads in total
typedef struct
{
  rowpads_t row[3];
} allpads_t;

_Static_assert(sizeof(allpads_t) == 12, "allpads_t size not 12");

// single phase contains configuration of all pads, duration and voltage level
typedef struct
{
  allpads_t pads;
  int duration;      // max 3600
  int level;         // max 100
} phase_t;

_Static_assert(sizeof(phase_t) == 20, "phase_t size not 20");

// each profile contains two phases.
typedef union
{
  uint32_t word[sizeof(phase_t) * 2 / sizeof(uint32_t)];
  struct
  {
    phase_t a;
    phase_t b;
  };
} profile_t;

_Static_assert(sizeof(profile_t) == 40, "profile_t size not 40");

void print_profile(int index);

void do_profile(int index);

// profile_t get_profile(int index);

void set_profile_phase(int profile_index, int phase_index, phase_t * phase);


#endif /* INC_PROFILE_H_ */
