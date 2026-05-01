/*
 * hv2801.h
 *
 *  Created on: Nov 4, 2025
 *      Author: matia
 */

#ifndef INC_HV2801_H_
#define INC_HV2801_H_

/**
 * This file is not relevant anymore since the analog switch (HV2801)
 * has been moved to another board (b).
 */

void HV2801_Init(void);
void HV2801_CLR(void);
void HV2801_SW(int index);

#endif /* INC_HV2801_H_ */
