/*
 * CAN.c
 *
 *  Created on: 15-Jul-2025
 *      Author: Rohit
 */
#include "sdk_project_config.h"
#include "CAN.h"
#define TX_MAILBOX  	(1UL)
#define TX_MSG_ID   	(1UL)
#define RX_MAILBOX0  	(0UL)
#define RX_MAILBOX1  	(2UL)
#define RX_MAILBOX2  	(3UL)
#define RX_MAILBOX3  	(4UL)
#define RX_MAILBOX4		(5UL)
#define TX_MAILBOX_IC  	(6UL)
#define RX_MSG_ID   	(2UL)
flexcan_msgbuff_t recvBuff301,recvBuff302,recvBuff303,recvBuff304,recvBuff305,recvBuff306,recvBuff307,recvBuff308,recvBuff309,recvBuff310,recvBuff311,recvBuff312,recvBuff313,recvBuff314,recvBuff315,recvBuff316;
void CAN_ReceiveConfig6(void)
{
	flexcan_data_info_t dataInfo =
    {
		            .data_length =8U,
		            .msg_id_type = FLEXCAN_MSG_ID_STD,
		            .enable_brs  = false,
		            .fd_enable   = false,
		            .fd_padding  = 0U,
					.is_remote=false
     };
	/* Configure RX message buffer with index RX_MSG_ID and RX_MAILBOX */
    FLEXCAN_DRV_ConfigRxMb(INST_FLEXCAN_CONFIG_1,RX_MAILBOX5,&dataInfo,0x100);
}
void CAN_ReceiveConfig7(void)
{
	flexcan_data_info_t dataInfo =
    {
		            .data_length =8U,
		            .msg_id_type = FLEXCAN_MSG_ID_STD,
		            .enable_brs  = false,
		            .fd_enable   = false,
		            .fd_padding  = 0U,
					.is_remote=false
     };

	/* Configure RX message buffer with index RX_MSG_ID and RX_MAILBOX */
    FLEXCAN_DRV_ConfigRxMb(INST_FLEXCAN_CONFIG_1,RX_MAILBOX6,&dataInfo,0x108);
 //   FLEXCAN_DRV_SetRxMbInterrupt(INST_FLEXCAN_CONFIG_1,RX_MAILBOX6,true);
}
void CAN_ReceiveConfig8(void)
{
	flexcan_data_info_t dataInfo =
    {
		            .data_length =8U,
		            .msg_id_type = FLEXCAN_MSG_ID_STD,
		            .enable_brs  = false,
		            .fd_enable   = false,
		            .fd_padding  = 0U,
					.is_remote=false
     };
	/* Configure RX message buffer with index RX_MSG_ID and RX_MAILBOX */
    FLEXCAN_DRV_ConfigRxMb(INST_FLEXCAN_CONFIG_1,RX_MAILBOX7,&dataInfo,0x107);
}
void CAN_ReceiveConfig9(void)
{
	flexcan_data_info_t dataInfo =
    {
		            .data_length =8U,
		            .msg_id_type = FLEXCAN_MSG_ID_STD,
		            .enable_brs  = false,
		            .fd_enable   = false,
		            .fd_padding  = 0U,
					.is_remote=false
     };
	/* Configure RX message buffer with index RX_MSG_ID and RX_MAILBOX */
    FLEXCAN_DRV_ConfigRxMb(INST_FLEXCAN_CONFIG_1,RX_MAILBOX8,&dataInfo,0x109);
}
void CAN_ReceiveConfig10(void)
{
	flexcan_data_info_t dataInfo =
    {
		            .data_length =8U,
		            .msg_id_type = FLEXCAN_MSG_ID_STD,
		            .enable_brs  = false,
		            .fd_enable   = false,
		            .fd_padding  = 0U,
					.is_remote=false
     };
	/* Configure RX message buffer with index RX_MSG_ID and RX_MAILBOX */
    FLEXCAN_DRV_ConfigRxMb(INST_FLEXCAN_CONFIG_1,RX_MAILBOX9,&dataInfo,0x10A);
}
void CAN_ReceiveConfig11(void)
{
	flexcan_data_info_t dataInfo =
    {
		            .data_length = 8U,
		            .msg_id_type = FLEXCAN_MSG_ID_STD,
		            .enable_brs  = false,
		            .fd_enable   = false,
		            .fd_padding  = 0U,
					.is_remote=false
     };
	/* Configure RX message buffer with index RX_MSG_ID and RX_MAILBOX */
    FLEXCAN_DRV_ConfigRxMb(INST_FLEXCAN_CONFIG_1,RX_MAILBOX0,&dataInfo,0x10B);
}
void CAN_ReceiveConfig12(void)
{
	flexcan_data_info_t dataInfo =
    {
		            .data_length = 8U,
		            .msg_id_type = FLEXCAN_MSG_ID_STD,
		            .enable_brs  = false,
		            .fd_enable   = false,
		            .fd_padding  = 0U,
					.is_remote=false
     };

    FLEXCAN_DRV_ConfigRxMb(INST_FLEXCAN_CONFIG_1,RX_MAILBOX1,&dataInfo,0x10C);
}
void CAN_ReceiveConfig13(void)
{
	flexcan_data_info_t dataInfo =
    {
		            .data_length = 8U,
		            .msg_id_type = FLEXCAN_MSG_ID_STD,
		            .enable_brs  = false,
		            .fd_enable   = false,
		            .fd_padding  = 0U,
					.is_remote=false
     };

    FLEXCAN_DRV_ConfigRxMb(INST_FLEXCAN_CONFIG_1,RX_MAILBOX2,&dataInfo,0x102);
}
void CAN_ReceiveConfig14(void)
{
	flexcan_data_info_t dataInfo =
    {
		            .data_length = 8U,
		            .msg_id_type = FLEXCAN_MSG_ID_STD,
		            .enable_brs  = false,
		            .fd_enable   = false,
		            .fd_padding  = 0U,
					.is_remote=false
     };

    FLEXCAN_DRV_ConfigRxMb(INST_FLEXCAN_CONFIG_1,RX_MAILBOX3,&dataInfo,0x103);
}
void CAN_ReceiveConfig15(void)
{
	flexcan_data_info_t dataInfo =
    {
		            .data_length = 8U,
		            .msg_id_type = FLEXCAN_MSG_ID_STD,
		            .enable_brs  = false,
		            .fd_enable   = false,
		            .fd_padding  = 0U,
					.is_remote=false
     };

    FLEXCAN_DRV_ConfigRxMb(INST_FLEXCAN_CONFIG_1,RX_MAILBOX4,&dataInfo,0x105);
}
void SendCANData(uint32_t mailbox, uint32_t messageId, uint8_t * data, uint32_t len)
{
    /* Set information about the data to be sent
     *  - 1 byte in length
     *  - Standard message ID
     *  - Bit rate switch enabled to use a different bitrate for the data segment
     *  - Flexible data rate enabled
     *  - Use zeros for FD padding
     */
    flexcan_data_info_t dataInfo =
    {
            .data_length = len,
            .msg_id_type = FLEXCAN_MSG_ID_EXT,
            .enable_brs  = false,
            .fd_enable   = false,
            .fd_padding  = 0U
    };

    /* Configure TX message buffer with index TX_MSG_ID and TX_MAILBOX*/
    FLEXCAN_DRV_ConfigTxMb(INST_FLEXCAN_CONFIG_1, mailbox, &dataInfo, messageId);

    /* Execute send non-blocking */
    FLEXCAN_DRV_Send(INST_FLEXCAN_CONFIG_1, mailbox, &dataInfo, messageId, data);
}

void FlexCANInit(void)
{
    /*
     * Initialize FlexCAN driver
     *  - 8 byte payload size
     *  - FD enabled
     *  - Bus clock as peripheral engine clock
     */
    FLEXCAN_DRV_Init(INST_FLEXCAN_CONFIG_1, &flexcanState0, &flexcanInitConfig0);
}

void FlexCANInit_IC()
{
	 FLEXCAN_DRV_Init(INST_FLEXCAN_CONFIG_2, &flexcanState1, &flexcanInitConfig1);
}

void SendCANData_IC(uint32_t mailbox, uint32_t messageId, uint8_t * data, uint32_t len)
{
    /* Set information about the data to be sent
     *  - 1 byte in length
     *  - Standard message ID
     *  - Bit rate switch enabled to use a different bitrate for the data segment
     *  - Flexible data rate enabled
     *  - Use zeros for FD padding
     */
    flexcan_data_info_t dataInfo =
    {
            .data_length = len,
            .msg_id_type = FLEXCAN_MSG_ID_STD,
            .enable_brs  = false,
            .fd_enable   = false,
            .fd_padding  = 0U
    };

    /* Configure TX message buffer with index TX_MSG_ID and TX_MAILBOX*/
    FLEXCAN_DRV_ConfigTxMb(INST_FLEXCAN_CONFIG_2, mailbox, &dataInfo, messageId);

    /* Execute send non-blocking */
    FLEXCAN_DRV_Send(INST_FLEXCAN_CONFIG_2, mailbox, &dataInfo, messageId, data);
}

void SendCANData_BMS(uint32_t mailbox, uint32_t messageId, uint8_t * data, uint32_t len)
{
    /* Set information about the data to be sent
     *  - 1 byte in length
     *  - Standard message ID
     *  - Bit rate switch enabled to use a different bitrate for the data segment
     *  - Flexible data rate enabled
     *  - Use zeros for FD padding
     */
    flexcan_data_info_t dataInfo =
    {
            .data_length = len,
            .msg_id_type = FLEXCAN_MSG_ID_STD,
            .enable_brs  = false,
            .fd_enable   = false,
            .fd_padding  = 0U
    };

    /* Configure TX message buffer with index TX_MSG_ID and TX_MAILBOX*/
    FLEXCAN_DRV_ConfigTxMb(INST_FLEXCAN_CONFIG_1,mailbox,&dataInfo, messageId);

    /* Execute send non-blocking */
    FLEXCAN_DRV_Send(INST_FLEXCAN_CONFIG_1, mailbox, &dataInfo, messageId, data);
}
void BMS_Req(uint32_t mailbox, uint32_t messageId, uint8_t * data, uint32_t len)
{
    /* Set information about the data to be sent
     *  - 1 byte in length
     *  - Standard message ID
     *  - Bit rate switch enabled to use a different bitrate for the data segment
     *  - Flexible data rate enabled
     *  - Use zeros for FD padding
     */
    flexcan_data_info_t dataInfo =
    {
            .data_length = 0U,
            .msg_id_type = FLEXCAN_MSG_ID_STD,
            .enable_brs  = false,
            .fd_enable   = false,
            .fd_padding  = 0U,
		    .is_remote   = true
    };
    FLEXCAN_DRV_ConfigTxMb(INST_FLEXCAN_CONFIG_1,mailbox,&dataInfo, messageId);
    /* Execute send non-blocking */
    FLEXCAN_DRV_Send(INST_FLEXCAN_CONFIG_1,mailbox,&dataInfo,messageId,data);
}
