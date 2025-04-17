/*
 * qz_vofa+.h
 *
 *  Created on: 2024��8��7��
 *      Author: 33224
 */

#ifndef CODE_QZ_VOFA__H_
#define CODE_QZ_VOFA__H_


#include "zf_common_typedef.h"

#include "zf_common_headfile.h"



#define USART_REC_LEN           200     //�����������ֽ��� 200

extern uint8 uart_get_data[200];                                                        // ���ڽ������ݻ�����
extern uint8 fifo_get_data[200];
extern uint8_t USART_TX_BUF[200];
extern uint8  USART_TX_BUF[USART_REC_LEN];  //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з�
extern uint8_t rxIndex;

void UART_INIT_car(void);
void wireless_printf(char* fmt, ...);
float Get_Data(void);
void USART_PID_Adjust(void);
extern float ceshi_QQ;
void FireWater_Test(void);	
void uart_rx_interrupt_handler (void);
void wireless_data_handler(void);
#endif /* CODE_QZ_VOFA__H_ */
