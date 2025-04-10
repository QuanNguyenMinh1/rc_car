/*
 * Serial.c
 *
 *  Created on: Jul 26, 2024
 *      Author: Quan
 */
#include "serial.h"
#include "uart_proto.h"

uint8_t urx_buff[MAX_LEN];
uint8_t utx_buff[MAX_LEN];
char scmd[4];
float fkp;
float fki;
float fkd;
float fkp1;
float fki1;
float fkd1;
float fkp2;
float fki2;
float fkd2;
float fset_point;
float fset_point1;
float fset_point2;
uint8_t urx_index = 0;
uint8_t urx = 0;
PROCESS_t tprocess;
extern uint8_t rxB2B;
uint8_t recent = 0;
extern uint8_t fault_flag;

void serial_init(void)
{
//	  HAL_UART_Receive_DMA(&huart3, &rxB2B, 1);
}

//void serial_write_com(char *scmd, float fvalue, float fkp, float fki, float fkd)
//{
////	char sFullCmd[] = "                                ";
////	char sKpid[] = "                                       ";
//	char sFullCmd[] = "                                               ";
//	char sKpid[] = "                                                  ";
//	uint8_t txSrc[50] = {0x7E};
//	uint8_t txBuf[55] = { NULL };
//	uint16_t txBufLen = 0;
//
////	snprintf(sTest, sizeof(sTest), "%s %.1f %.1f %.1f %.1f", scmd, fvalue, fkp, fki, fkd);
//	snprintf(sFullCmd, 5, "%s", scmd);
////	snprintf(sKpid, sizeof(sKpid), " %.1f %.1f %.1f %.1f", fset_point, fkp, fki, fkd);
//	snprintf(sKpid, sizeof(sKpid), " %.1f %.1f %.1f %.1f", fvalue, fkp, fki, fkd);
//	strncat(sFullCmd, sKpid, sizeof(sKpid));
//	memcpy(&txSrc[1], &sFullCmd, strlen(sFullCmd)); //void * memcpy (void * to , const void * from , size_t numBytes );
//	txSrc[strlen(txSrc)] = ' ';
//	txSrc[strlen(txSrc)] = strlen(txSrc) + 1;
//	UART_frame_data(&txSrc, (uint8_t) strlen(txSrc), &txBuf, &txBufLen); //ACK, DATALENGTH sau start thì k cần ESC
////	if (HAL_GetTick() - recent > 2000)
////	{
//		HAL_UART_Transmit(&huart3, txBuf, txBufLen, 2000);	// chi rx duoc vai ki tu. Transmit này hiện tại ok nhất nhma k rx được trong ghi stm transmit ;))
////		recent = HAL_GetTick();
////	}
//
//
//	//	HAL_UART_Transmit_DMA(&huart3, txBuf, txBufLen);	// bên gui rx lỗi
////	HAL_UART_Transmit(&huart3, txBuf, txBufLen, 20000);	// chi rx duoc vai ki tu. Transmit này hiện tại ok nhất nhma k rx được trong ghi stm transmit ;))
////	HAL_UART_Transmit_IT(&huart3, txBuf, txBufLen);		// bên gui rx lỗi
//}

void serial_write_com(char *scmd, float fvalue, float fkp, float fki, float fkd)
{
////	char sFullCmd[] = "                                               ";
////	char sKpid[] = "                                                  ";
//
////	char sFullCmd[55];
////	char sKpid[30];
////
////	uint8_t lenByte = 13;
////	uint8_t txSrc[65] = {0x7E};
	uint8_t txBuf[30] = { NULL };
//
//	char sFullCmd[70];
//	char sKpid[40];
//
//	uint8_t lenByte = 13;
//	uint8_t txSrc[45] = {0x7E};
//	uint8_t txBuf[50] = { NULL };
//	uint16_t txBufLen = 0;
//
////	snprintf(sTest, sizeof(sTest), "%s %.1f %.1f %.1f %.1f", scmd, fvalue, fkp, fki, fkd);
//	snprintf(sFullCmd, 5, "%s", scmd);
//
//
////	snprintf(sKpid, sizeof(sKpid), " %.2f %.2f %.5f %.5f ", fvalue, fkp, fki, fkd);
//
//	fault_flag = 1;
//
//	sprintf(sKpid, " %.2f %.2f %.5f %.8f", fvalue, fkp, fki, fkd);
////	snprintf(sKpid, sizeof(sKpid), " %.1f %.1f %.1f %.1f", fvalue, fkp, fki, fkd);
////	HAL_UART_Transmit(&huart3, sKpid, sizeof(sKpid), 2000);	// chi rx duoc vai ki tu. Transmit này hiện tại ok nhất nhma k rx được trong ghi stm transmit ;))
//	fault_flag = 2;
////	snprintf(sKpid, sizeof(sKpid), " %.1f %.1f %.1f %.1f ", fvalue, fkp, fki, fkd);
//	strncat(sFullCmd, sKpid, sizeof(sKpid));
//	fault_flag = 3;
//
//	txSrc[0] = lenByte;
//	memcpy(&txSrc[2], &sFullCmd, strlen(sFullCmd)); //void * memcpy (void * to , const void * from , size_t numBytes );
////	txSrc[strlen(txSrc)] = ' ';
////	txSrc[strlen(txSrc)] = '  ';
////	txSrc[strlen(txSrc)] = NULL;
//	fault_flag = 4;
//
//	txSrc[1] = (uint8_t)0X7E;
//	txSrc[0] = (uint8_t)(strlen(txSrc));
//	UART_frame_data(&txSrc, (uint8_t) strlen(txSrc), &txBuf, &txBufLen); //ACK, DATALENGTH sau start thì k cần ESC
//	fault_flag = 5;
//
//	HAL_UART_Transmit(&huart3, txBuf, txBufLen, 2000);	// chi rx duoc vai ki tu. Transmit này hiện tại ok nhất nhma k rx được trong ghi stm transmit ;))

	snprintf(txBuf, sizeof(txBuf), "%s %.1f %.1f %.1f %.1f", scmd, fvalue, fkp, fki, fkd);
//	txBuf[strlen(&txBuf)] =
	HAL_UART_Transmit(&huart3, txBuf, strlen(&txBuf) + 1, 2000);	// chi rx duoc vai ki tu. Transmit này hiện tại ok nhất nhma k rx được trong ghi stm transmit ;))

}

void serial_handle(uint8_t *ubuff)
{
	if (ubuff == NULL)
	{
		// Handle the case where ubuff is nullptr
		return;
	}
	char str[MAX_LEN];
	snprintf(str, sizeof(str), "%s", ubuff);
	sscanf(str, "%s %f %f %f %f", scmd, &fkp, &fki, &fkd, &fset_point);
	HAL_UART_Transmit(&UART_COM, ubuff, urx_index, HAL_MAX_DELAY);
	urx_index = 0;
	  if (StrCompare(scmd, (uint8_t*)"SPID", 4))
	    {
	      tprocess = SPID;
	    }
	    else if (StrCompare(scmd, (uint8_t*)"VTUN", 4))
	    {
	      tprocess = VTUN;
	    }
	    else if (StrCompare(scmd, (uint8_t*)"PTUN", 4))
	    {
	      tprocess = PTUN;
	    }
	    else if (StrCompare(scmd, (uint8_t*)"STOP", 4))
	    {
	      tprocess = STOP;
	    }
	    else
	    {
	      tprocess = NONE;
	    }
}

bool StrCompare(char *pBuff, uint8_t *pSample, uint8_t nSize)
{
    for (int i = 0; i < nSize; i++)
    {
        if(pBuff[i] != pSample[i])
        {
            return false;
        }
    }
    return true;
}
