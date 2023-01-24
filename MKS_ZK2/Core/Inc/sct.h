/*
 * sct.h
 *
 *  Created on: Oct 19, 2022
 *      Author: xhlava51
 */

#ifndef SCT_H_
#define SCT_H_

void sct_led(uint32_t value);
void sct_init(void);
void sct_value(uint16_t value, uint8_t led, uint8_t decimal_point);

#endif /* SCT_H_ */
