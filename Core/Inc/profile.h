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

// suppress the IDE syntax error (yellow mark) by
// Project --> Properties --> C/C++ General --> Preprocessor Include Paths,
// Macros etc. On the Entries tab, select CDT User Setting Entries, then click
// Add
//
//   _Static_assert(a,b)
//

/**
 * 1. profilt_t的数据格式定义有历史原因，无重构必要性；
 * 2. profile_t在通过消息队列传递时使用值而不是引用，即传递的是profile_t对象；
 * 3. profile_t的profile a的frequency设置为0xFFFFFFFF判定为test
 */
// #define A_FREQ_FOR_TEST (0xF0000000)

#define NUM_OF_PROFILES 16
#define STOP_PROFILE_INDEX (NUM_OF_PROFILES)         // 16
#define DDBF_PROFILE_INDEX (STOP_PROFILE_INDEX + 1)  // 17
#define CURR_PROFILE_INDEX (DDBF_PROFILE_INDEX + 1)  // 18
#define NEXT_PROFILE_INDEX (CURR_PROFILE_INDEX + 1)  // 19
#define LAST_PROFILE_INDEX (NEXT_PROFILE_INDEX)      // 19
#define NUM_OF_ALL_PROFILES (LAST_PROFILE_INDEX + 1) // 20

/**
 * ad9834的输出频率范围
 */
#define DDS_MIN_FREQ 5000
#define DDS_MAX_FREQ 500000

/**
 * 新的pad配置每组只有4个Pad，共4组，所以一个32bit整数即可
 * 表示配置；
 *
 * word方便内部使用；
 * a1/2/3/4-d1/2/3/4方便赋值取值，
 * xbuf用于uart传输；
 */
typedef union {
  uint32_t word;
  struct __attribute__((packed)) {
    unsigned int a1 : 2;
    unsigned int a2 : 2;
    unsigned int a3 : 2;
    unsigned int a4 : 2;
    unsigned int b1 : 2;
    unsigned int b2 : 2;
    unsigned int b3 : 2;
    unsigned int b4 : 2;
    unsigned int c1 : 2;
    unsigned int c2 : 2;
    unsigned int c3 : 2;
    unsigned int c4 : 2;
    unsigned int d1 : 2;
    unsigned int d2 : 2;
    unsigned int d3 : 2;
    unsigned int d4 : 2;
  };
  uint8_t xbuf[4];
} allpads_v2_t;

_Static_assert(sizeof(allpads_v2_t) == 4, "allpads_v2_t size not 4");

/**
 * 实验设计为两个phase来回切换
 * 每个phase有独立的频率，时间，电压
 */
typedef struct {
  allpads_v2_t pads;
  uint32_t freq; // frequency 5000 to 500,000
  int duration;  // max 3600
  int level;     // max 100
} phase_v2_t;

_Static_assert(sizeof(phase_v2_t) == 16, "phase_v2_t size not 20");

typedef union {
  uint32_t word[sizeof(phase_v2_t) * 2 / sizeof(uint32_t)];
  struct {
    phase_v2_t a;
    phase_v2_t b;
  };
} profile_v2_t;

_Static_assert(sizeof(profile_v2_t) == 32, "profile_v2_t size not 32");

/**
 * 打印一个profile，可以是内部profile
 */
void print_profile(int index);

/**
 * 发送一个请求；在profile任务的主循环里会读取该请求；
 * 该函数在command任务种被调用，因为按键处理也在这个任务中；
 *
 * TODO 可以考虑修改名称
 */
void do_profile(int index);

/**
 * 在define命令中设置profile
 */
void set_profile_phase(int profile_index, int phase_index,
                       const phase_v2_t *phase);

#endif /* INC_PROFILE_H_ */
