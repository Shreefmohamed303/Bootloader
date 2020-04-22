/*
 * Bootloader.c
 *
 *  Created on: Apr 19, 2020
 *      Author: LENOVO
 */
#include"Bootloader.h"
uint8_t supported_commands[8] = {
                               BL_GET_VER ,
                               BL_GET_HELP,
                               BL_GET_CID,
                               BL_GET_RDP_STATUS,
                               BL_GO_TO_ADDR,
                               BL_FLASH_ERASE,
                               BL_MEM_WRITE,
							   BL_READ_SECTOR_STATUS} ;
/*****************************FUNCTION DEFINITION**************************************/

void BootLoader_Jump_to_User_APP()
{
	// Pointer to Function to hold the Reset_Handler() of the User Application code
	void (*PF_APP_Reset_handler)(void);

	printmsg("BL_DEBUG_MSG:Bootloader_jump_to_User_Application\r\n");

	/* 1. Configure the MSP_Value of User application
	 *    by Reading the value of the base address of Sector 2
	 */
	uint32_t user_MSP_value=*(volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS;
	printmsg("BL_DEBUG_MSG:User_MSP_Value : %#x\r\n",user_MSP_value);

	// This Function Comes with CEMSIS
	__set_MSP(user_MSP_value);

	/* 2. Fetching The Reset Handler address of The User Application
	 * 	  From The Location (FLASH_SECTOR2_BASE_ADDRESS+4)
	 * 	  as it's the second location of the Flash memory
	 */
	uint32_t resetHandler_address=*(volatile uint32_t*)(FLASH_SECTOR2_BASE_ADDRESS+4);

	PF_APP_Reset_handler=(void*)resetHandler_address;

	printmsg("BL_DEBUG_MSG:App_Reset_Handler_Address : %#x\r\n",PF_APP_Reset_handler);

	/* 3. Jump to Reset Handler of The User_Application */
	PF_APP_Reset_handler();
}


void BootLoader_UART_Read_Host_CMD()
{
	uint8_t BL_rcv_len=0;
	uint8_t BL_cmd_code=0;
	while(1)
	{
		memset(BL_rx_buffer,0,BL_RX_LEN);
		/* Here we will Read and Decode Command coming form the Host */

		// 1.Read only one Byte from the Host, which is "LENGTH_FIELD" of the command Packet
		HAL_UART_Receive(C_UART,BL_rx_buffer,1,HAL_MAX_DELAY);
		BL_rcv_len=BL_rx_buffer[0];

		// 2.Read The Rest of the command packet according to Length i got from the user
		HAL_UART_Receive(C_UART,&BL_rx_buffer[1],BL_rcv_len,HAL_MAX_DELAY);

		// 3.The second Byte of The Packet is the "COMMAND_CODE" of the command packet
		BL_cmd_code=BL_rx_buffer[1];

		// 4.Use Switch case to Decode The Command Code and Give The Right Response according to CMD
		switch(BL_cmd_code)
		{
			case BL_GET_VER:
				BL_hanlde_getVersion_cmd(BL_rx_buffer);
				break;
			case BL_GET_HELP:
				BL_hanlde_getHelp_cmd(BL_rx_buffer);
				break;
			case BL_GET_CID:
				BL_hanlde_getCID_cmd(BL_rx_buffer);
				break;
			case BL_GET_RDP_STATUS:
				BL_hanlde_getRDP_cmd(BL_rx_buffer);
				break;
			case BL_GO_TO_ADDR:
				BL_hanlde_goAddress_cmd(BL_rx_buffer);
				break;
			case BL_FLASH_ERASE:
				BL_hanlde_flashErase_cmd(BL_rx_buffer);
				break;
			case BL_MEM_WRITE:
				BL_hanlde_memWrite_cmd(BL_rx_buffer);
				break;
			case BL_MEM_READ:
				BL_hanlde_memRead_cmd(BL_rx_buffer);
				break;
			case BL_READ_SECTOR_STATUS:
				BL_handle_ReadSectorStatus_cmd(BL_rx_buffer);
				break;
			default:
				printmsg("BL_DEBUG_MSG: INVALID COMMAND CODE RECEIVED FROM THE HOST\r\n");
		}
	}
}

/* This Function send ACK if CRC Matches with length of bytes of the reply */
void BootLoader_Send_ACK(uint8_t command_code , uint8_t length_of_reply)
{
	/* Here We Send Two Bytes
	 * 1st Byte is ACK
	 * 2nd Byte is The Length of The Coming Reply
	 */
	uint8_t ACK_buffer[2];
	ACK_buffer[0]= BL_ACK;
	ACK_buffer[1]= length_of_reply;
	HAL_UART_Transmit(C_UART,ACK_buffer,2,HAL_MAX_DELAY);
}

/* This Function sends NACK if CRC doesn't Matches */
void BootLoader_Send_NACK(void)
{
	uint8_t NACK_buffer=BL_NACK;
	HAL_UART_Transmit(C_UART,&NACK_buffer,1,HAL_MAX_DELAY);
}

uint8_t BootLoader_Verify_CRC(uint8_t *pData ,uint32_t length ,uint32_t CRC_Host)
{
	uint32_t uwCRCValue=0xFF;

	for(uint32_t i=0 ; i<length ; i++)
	{
		// Convert Data from uint8_t to uint32_t
		uint32_t iData=pData[i];
		//Accumulate The CRC for each iteration of pData bytes
		uwCRCValue=HAL_CRC_Accumulate(&hcrc,&iData,1);
	}

	if(uwCRCValue==CRC_Host)
	{
		return VERIFY_CRC_SUCCESS;
	}
	else
	{
		return VERIFY_CRC_FAIL;
	}
}

uint8_t BootLoader_get_Version(void)
{
	return (uint8_t)BL_VERSION;
}

void BootLoader_UART_Write_Data(uint8_t *pbuffer, uint32_t length)
{
	HAL_UART_Transmit(C_UART,pbuffer,length,HAL_MAX_DELAY);
}

uint16_t get_MCU_ID_Code(void)
{
	uint16_t chip_id;
	// we need to get from bit 0 till bit 11 ,so we need to make masking
	chip_id = (uint16_t)(DBGMCU->IDCODE) & 0x0FFF;
	return  chip_id;

}
/***************Implementation of Bootloader Command Handle Function*******************/
void BL_hanlde_getVersion_cmd(uint8_t* pbuffer)
{
	uint8_t BL_Version;
	uint8_t BL_length_to_follow = pbuffer[0]; // length Segment in The command Packet
	uint8_t BL_command_code = pbuffer[1];

	printmsg("BL_DEBUG_MSG: BL_hanlde_getVersion_cmd\r\n");

	// 1.Verify The CRC

	// Total Length of The Command Packet
	uint32_t command_Packet_length = BL_length_to_follow + BL_LENGTH_TO_FOLLOW_SEG_SIZE ;

	// Extract CRC_32 word sent by The Host

	//CRC_Host Contains The Data of The beginning of BL_CRC_SEG till the end of The command Packet
	uint32_t CRC_Host = * ((uint32_t*) (pbuffer+command_Packet_length-BL_CRC_SEG_SIZE) );

	if(! BootLoader_Verify_CRC(&pbuffer[0],command_Packet_length-BL_CRC_SEG_SIZE,CRC_Host))
	{
		// Checksum is Correct
		printmsg("BL_DEBUG_MSG: Checksum Success !!\r\n");
		BootLoader_Send_ACK(BL_command_code,BL_VERSION_REPLY_SIZE);
		// 2. Get The BootLoader Version
		BL_Version = BootLoader_get_Version();
		printmsg("BL_DEBUG_MSG: BL_Version: %d	%#x\r\n",BL_Version,BL_Version);
		// 3. Send The BootLooder Version via C_UART
		BootLoader_UART_Write_Data(&BL_Version,1);
	}
	else
	{
		// 2. Checksum is Wrong , Send NACK
		printmsg("BL_DEBUG_MSG: Checksum FAIL !!\r\n");
		BootLoader_Send_NACK();
	}
}
void BL_hanlde_getHelp_cmd(uint8_t* pbuffer)
{
	uint8_t BL_length_to_follow = pbuffer[0]; // length Segment in The command Packet
	uint8_t BL_command_code = pbuffer[1];

	printmsg("BL_DEBUG_MSG: BL_hanlde_getHelp_cmd\r\n");

	// 1.Verify The CRC

	// Total Length of The Command Packet
	uint32_t command_Packet_length = BL_length_to_follow + BL_LENGTH_TO_FOLLOW_SEG_SIZE ;

	// Extract CRC_32 word sent by The Host

	//CRC_Host Contains The Data of The beginning of BL_CRC_SEG till the end of The command Packet
	uint32_t CRC_Host = * ((volatile uint32_t*) (pbuffer+command_Packet_length-BL_CRC_SEG_SIZE) );

	if(! BootLoader_Verify_CRC(&pbuffer[0],command_Packet_length-BL_CRC_SEG_SIZE,CRC_Host))
	{
		// Checksum is Correct
		printmsg("BL_DEBUG_MSG: Checksum Success !!\r\n");
		BootLoader_Send_ACK(BL_command_code,sizeof(supported_commands));
		BootLoader_UART_Write_Data(supported_commands,sizeof(supported_commands) );
	}
	else
	{
		// 2. Checksum is Wrong , Send NACK
		printmsg("BL_DEBUG_MSG: Checksum FAIL !!\r\n");
		BootLoader_Send_NACK();
	}
}
void BL_hanlde_getCID_cmd(uint8_t* pbuffer)
{
	uint16_t BL_chipID=0;
	uint8_t BL_length_to_follow = pbuffer[0]; // length Segment in The command Packet
	uint8_t BL_command_code = pbuffer[1];

	printmsg("BL_DEBUG_MSG: BL_hanlde_getCID_cmd\r\n");

	// 1.Verify The CRC

	// Total Length of The Command Packet
	uint32_t command_Packet_length = BL_length_to_follow + BL_LENGTH_TO_FOLLOW_SEG_SIZE ;

	// Extract CRC_32 word sent by The Host

	//CRC_Host Contains The Data of The beginning of BL_CRC_SEG till the end of The command Packet
	uint32_t CRC_Host = * ((uint32_t*) (pbuffer+command_Packet_length-BL_CRC_SEG_SIZE) );

	if(! BootLoader_Verify_CRC(&pbuffer[0],command_Packet_length-BL_CRC_SEG_SIZE,CRC_Host))
	{
		// Checksum is Correct
		printmsg("BL_DEBUG_MSG: Checksum Success !!\r\n");
		BootLoader_Send_ACK(BL_command_code,BL_CID_REPLY_SIZE);
		// 2. Get chip ID
		BL_chipID = get_MCU_ID_Code();
		printmsg("BL_DEBUG_MSG: BL_chipID: %d	%#x\r\n",BL_chipID,BL_chipID);
		// 3. Send The Chip ID via C_UART
		BootLoader_UART_Write_Data( (uint8_t*) &BL_chipID,BL_CID_REPLY_SIZE);
	}
	else
	{
		// 2. Checksum is Wrong , Send NACK
		printmsg("BL_DEBUG_MSG: Checksum FAIL !!\r\n");
		BootLoader_Send_NACK();
	}

}
void BL_hanlde_getRDP_cmd(uint8_t* pbuffer)
{

}
void BL_hanlde_goAddress_cmd(uint8_t* pbuffer)
{

}
void BL_hanlde_flashErase_cmd(uint8_t* pbuffer)
{

}
void BL_hanlde_memWrite_cmd(uint8_t* pbuffer)
{

}
void BL_hanlde_memRead_cmd(uint8_t* pbuffer)
{

}
void BL_handle_ReadSectorStatus_cmd(uint8_t* pbuffer)
{

}
