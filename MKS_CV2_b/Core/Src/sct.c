
/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "main.h"

//#include "stm32f030x8.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif


void sct_led(uint32_t value){
  for(uint8_t i = 0; i<32;i++){
	HAL_GPIO_WritePin(SCT_SDI_GPIO_Port, SCT_SDI_Pin,value & 1);
    value >>= 1;
    HAL_GPIO_WritePin(SCT_CLK_GPIO_Port, SCT_CLK_Pin,1);
    HAL_GPIO_WritePin(SCT_CLK_GPIO_Port, SCT_CLK_Pin,0);
  }

  HAL_GPIO_WritePin(SCT_NLA_GPIO_Port, SCT_NLA_Pin,1);
  HAL_GPIO_WritePin(SCT_NLA_GPIO_Port, SCT_NLA_Pin,0);
}

void sct_init(void){
	HAL_GPIO_WritePin(SCT_NOE_GPIO_Port, SCT_NOE_Pin,0);
	sct_led(0); //nastaveni do registru
}

void sct_value(uint16_t value){
	static const uint32_t reg_values[3][10] = {
	{
		//PCDE--------GFAB @ DIS1
		0b0111000000000111 << 16,
		0b0100000000000001 << 16,
		0b0011000000001011 << 16,
		0b0110000000001011 << 16,
		0b0100000000001101 << 16,
		0b0110000000001110 << 16,
		0b0111000000001110 << 16,
		0b0100000000000011 << 16,
		0b0111000000001111 << 16,
		0b0110000000001111 << 16,
	},
	{
		//----PCDEGFAB---- @ DIS2
		0b0000011101110000 << 0,
		0b0000010000010000 << 0,
		0b0000001110110000 << 0,
		0b0000011010110000 << 0,
		0b0000010011010000 << 0,
		0b0000011011100000 << 0,
		0b0000011111100000 << 0,
		0b0000010000110000 << 0,
		0b0000011111110000 << 0,
		0b0000011011110000 << 0,
	},
	{
		//PCDE--------GFAB @ DIS3
		0b0111000000000111 << 0,
		0b0100000000000001 << 0,
		0b0011000000001011 << 0,
		0b0110000000001011 << 0,
		0b0100000000001101 << 0,
		0b0110000000001110 << 0,
		0b0111000000001110 << 0,
		0b0100000000000011 << 0,
		0b0111000000001111 << 0,
		0b0110000000001111 << 0,
	},
	};
	uint32_t reg =0;

	reg |= reg_values[0][value / 100 % 10];//vydelime stem + modulo 10 a ziskame pouze stovky
	reg |= reg_values[1][value / 10 % 10];//desitky
	reg |= reg_values[2][value / 1 % 10];//jednotky

	sct_led(reg);
}
