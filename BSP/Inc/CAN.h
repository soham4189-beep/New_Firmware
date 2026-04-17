/*
 * CAN.h
 *
 *  Created on: 15-Jul-2025
 *      Author: Rohit
 */
#ifndef INC_CAN_H_
#define INC_CAN_H_
void FlexCANInit(void);
void SendCANData(uint32_t mailbox, uint32_t messageId, uint8_t * data, uint32_t len);
void BMS_Req(uint32_t mailbox, uint32_t messageId, uint8_t * data, uint32_t len);
void CAN_ReceiveConfig1(void);
void CAN_ReceiveConfig2(void);
void CAN_ReceiveConfig3(void);
void CAN_ReceiveConfig4(void);
void CAN_ReceiveConfig5(void);
void CAN_ReceiveConfig6(void);
void CAN_ReceiveConfig7(void);
void CAN_ReceiveConfig8(void);
void CAN_ReceiveConfig9(void);
void CAN_ReceiveConfig10(void);
void CAN_ReceiveConfig11(void);
void CAN_ReceiveConfig12(void);
void FlexCANInit_IC(void);
void SendCANData_IC(uint32_t mailbox, uint32_t messageId, uint8_t * data, uint32_t len);
#define PDLY_TIMEOUT 	(30UL)
//TxMailBoxes
#define TX_MAILBOX_IC_0 (6UL)
#define TX_MAILBOX_IC_1 (7UL)
#define TX_MAILBOX0     (16UL)
#define TX_MAILBOX  	(1UL)
#define TX_MAILBOX2  	(13UL)
#define TX_MAILBOX6 	(18UL)
//#define TX_MAILBOX3  	(14UL)
//#define TX_MAILBOX4 	(15UL)
//#define TX_MAILBOX5 	(17UL)
//RxMailBoxes
#define RX_MAILBOX0  	(0UL)
#define RX_MAILBOX1  	(2UL)
#define RX_MAILBOX2  	(3UL)
#define RX_MAILBOX3  	(4UL)
#define RX_MAILBOX4	    (5UL)
#define RX_MAILBOX5	    (8UL)
#define RX_MAILBOX6	    (9UL)
#define RX_MAILBOX7	    (10UL)
#define RX_MAILBOX8	    (11UL)
#define RX_MAILBOX9	    (12UL)
#define RX_MAILBOX10    (14UL)
#define RX_MAILBOX11    (15UL)
#define RX_MAILBOX12    (1UL)
#define RX_MAILBOX13    (6UL)
extern  flexcan_msgbuff_t recvBuff301,recvBuff302,recvBuff303,recvBuff304,recvBuff305,recvBuff306,recvBuff307,recvBuff308,recvBuff309,recvBuff310,recvBuff311,recvBuff312,recvBuff313,recvBuff314,recvBuff315,recvBuff316;
//#define RX_MAILBOX14    (13UL)
#endif /* INC_CAN_H_ */
//#define TX_MSG_ID   	(1UL)
//#define RX_MSG_ID   	(2UL)
