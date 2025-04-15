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

#define SIZE_OF_DOUBLEWORD            8
#define SIZE_OF_WORD                  4
#define NUM_OF_PROFILES               16

#define SIZE_OF_PROFILE_IN_WORD       10
#define SIZE_OF_PROFILE_IN_DOUBLEWORD 5
#define SIZE_OF_PROFILES              (sizeof(profile_v2_t) * NUM_OF_PROFILES)

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
} rowcfg_t;

_Static_assert(sizeof(rowcfg_t) == 4, "rowcfg_t size not 4");

// three rows of pads in total
typedef struct
{
  rowcfg_t row[3];
} padscfg_t;

_Static_assert(sizeof(padscfg_t) == 12, "padscfg_t size not 12");

// single phase contains configuration of all pads, duration and voltage level
typedef struct
{
  padscfg_t pads;
  int duration;      // max 3600
  int level;         // max 100
} phase_t;

_Static_assert(sizeof(phase_t) == 20, "phase_t size not 20");

typedef union __attribute__((packed))
{
  uint32_t word[sizeof(phase_t) * 2 / sizeof(uint32_t)];
  struct
  {
    phase_t a;
    phase_t b;
  };
} profile_v2_t;

_Static_assert(sizeof(profile_v2_t) == 40, "profile_t size not 40");

void print_profile(int index);
void print_profile_v2(int index);

void do_profile_by_key(int profile_index);
void do_profile_blink(void);
// profile_t get_profile(int index);

void set_profile_phase(int profile_index, int profile_phase, phase_t * phase);


#endif /* INC_PROFILE_H_ */
