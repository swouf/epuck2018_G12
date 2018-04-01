/*
 * tof.h
 *
 *  Created on: 1 avr. 2018
 *      Author: Minh
 */

#ifndef TOF_H_
#define TOF_H_

void tof_init(void);
uint16_t tof_get_distance(void);
void tof_stop(void);



#endif /* TOF_H_ */
