/*
 * Serial.h
 *
 *  Created on: Jul 26, 2024
 *      Author: Quan
 */

#ifndef USERCODE_SERIAL_SERIAL_H_
#define USERCODE_SERIAL_SERIAL_H_

#include "../UserCode/user_define.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "usart.h"

typedef enum
{
  NONE = 0,
  SPID,
  VTUN,
  PTUN,
  STOP,
//  RSET,
} PROCESS_t;

void serial_init(void);
//void serial_write_com(char *ucmd, float dvalue);
void serial_write_com(char *scmd, float fvalue, float fkp, float fki, float fkd);

void serial_handle(uint8_t *ubuff);
bool StrCompare(char *pBuff, uint8_t *pSample, uint8_t nSize);

#endif /* USERCODE_SERIAL_SERIAL_H_ */
