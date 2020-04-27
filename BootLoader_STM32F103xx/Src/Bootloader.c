/*
 * Bootloader.c
 *
 *  Created on: Apr 19, 2020
 *      Author: LENOVO
 */
#include"Bootloader.h"

uint8_t supported_commands[8] = {
                               BL_GET_VER,
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

uint8_t BL_verify_address(uint32_t go_address)
{
	/*
	 * what is The Valid Addresses which we can jump ?
	 *
	 * - Can we jump to System Memory ? Yes
	 * - Can we jump to SRAM ? Yes
	 * - Can we jump to Peripheral Memory ? it's Possible but Not Recommended ,so NO
	 */
	if( (go_address >= SRAM_BASE) && (go_address <=SRAM_END) )
	{
		return ADDRESS_VALID;
	}
	else if( (go_address >= FLASH_BASE) && (go_address <=FLASH_END) )
	{
		return ADDRESS_VALID;
	}
	else
	{
		return ADDRESS_INVALID ;
	}

}

uint8_t BL_Execute_Flash_Erase(uint8_t BL_page_number, uint8_t BL_number_of_pages)
{
	FLASH_EraseInitTypeDef hflash;

	if(BL_page_number>128)
	{
		return FLASH_ERASE_FAIL ;
	}
	else if(BL_page_number<128  || BL_page_number==0xFF )
	{
		if(BL_page_number==0xFF)
		{
			//Mass Erase
			hflash.TypeErase=FLASH_TYPEERASE_MASSERASE;
		}
		else
		{
			//Pages Erase
			/* Calculation of how Many Pages needs to be Erased */
			uint8_t remaining_pages = 128-BL_number_of_pages;
			if(BL_number_of_pages > remaining_pages)
			{
				BL_number_of_pages=remaining_pages ;
			}

			hflash.TypeErase=FLASH_TYPEERASE_PAGES;
			hflash.NbPages=BL_number_of_pages;
			hflash.PageAddress=0x08008000;
		}
		hflash.Banks=FLASH_BANK_1;
		uint32_t pageError ;
		HAL_FLASH_Unlock();
		if(HAL_FLASHEx_Erase(&hflash,&pageError)==HAL_OK)
		{
			HAL_FLASH_Lock();
			return FLASH_ERASE_SUCCESS ;
		}
		else
		{
			return FLASH_ERASE_FAIL ;
		}
	}

	return FLASH_ERASE_FAIL ;

}

uint8_t execute_Mem_Write(uint8_t *pbuffer,uint32_t Mem_Address,uint8_t payload_length)
{
	uint8_t flashing_status=0 ;
	// We Have To unlock The Flash Module To get control of The Registers
	HAL_FLASH_Unlock();

	for(uint32_t i=0 ; i<payload_length ; i+=2)
	{
		uint16_t data= pbuffer[i] | (pbuffer[i+1] <<8);
		flashing_status=HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD , Mem_Address+i,data);
	}
	HAL_FLASH_Lock();
	return flashing_status ;
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
	uint8_t BL_length_to_follow = pbuffer[0]; // length Segment in The command Packet
	uint8_t BL_command_code = pbuffer[1];

	// Total Length of The Command Packet
	uint32_t command_Packet_length = BL_length_to_follow + BL_LENGTH_TO_FOLLOW_SEG_SIZE ;

	printmsg("BL_DEBUG_MSG: BL_hanlde_goAddress_cmd\r\n");

	// 1.Verify The CRC

	//extract The CRC Segment form The Host
	uint32_t CRC_Host = *(uint32_t *) (&pbuffer+command_Packet_length-BL_CRC_SEG_SIZE);

	if(1)//BootLoader_Verify_CRC(&pbuffer[0],command_Packet_length-BL_CRC_SEG_SIZE,CRC_Host))
	{
		// 2.Checksum is Success
		printmsg("BL_DEBUG_MSG: Checksum Success !!\r\n");

		// 3.Send ACK
		BootLoader_Send_ACK(BL_command_code,BL_GO_TO_ADDR_REPLY_SIZE);

		// 4.Extract The Go Address
		uint32_t go_address = (*(uint32_t*) &pbuffer[2] );
		printmsg("BL_DEBUG_MSG: GO ADDRESS: %#x\r\n",go_address);

		if(BL_verify_address(go_address)==ADDRESS_VALID)
		{
			// Tell Host That Address is Fine
			printmsg("BL_DEBUG_MSG: GO ADDRESS: %#x is VALID \r\n",go_address);
			HAL_UART_Transmit(C_UART,ADDRESS_VALID,1,HAL_MAX_DELAY);

			go_address +=1 ; // To Make it Thump instruction

			void (*lets_jump) (void) = (void*)go_address;
			printmsg("BL_DEBUG_MSG: Jumping To GO ADDRESS: %#x\r\n",go_address);

			lets_jump();
		}
		else
		{
			// Tell Host That Address is Not Valid
			printmsg("BL_DEBUG_MSG: GO ADDRESS: %#x is INVALID \r\n",go_address);
			BootLoader_UART_Write_Data((uint8_t*)ADDRESS_INVALID,1);

		}
	}
	else
	{
		// 2.Checksum is Wrong , Send NACK
		printmsg("BL_DEBUG_MSG: Checksum FAIL !!\r\n");
		BootLoader_Send_NACK();
	}
}
void BL_hanlde_flashErase_cmd(uint8_t* pbuffer)
{
	uint8_t BL_length_to_follow = pbuffer[0]; // length Segment in The command Packet
	uint8_t BL_command_code = pbuffer[1];
	uint8_t BL_page_number = pbuffer[2];
	uint8_t BL_number_of_pages = pbuffer[3];
	uint8_t BL_Erase_Status=0x00;

	// Total Length of The Command Packet
	uint32_t command_Packet_length = BL_length_to_follow + BL_LENGTH_TO_FOLLOW_SEG_SIZE ;

	printmsg("BL_DEBUG_MSG: BL_hanlde_flashErase_cmd\r\n");

	// 1.Verify The CRC

	//extract The CRC Segment form The Host
	uint32_t CRC_Host = *(uint32_t *) (&pbuffer+command_Packet_length-BL_CRC_SEG_SIZE);

//	if(! BootLoader_Verify_CRC(&pbuffer[0],command_Packet_length-BL_CRC_SEG_SIZE,CRC_Host))
//	{
		// 2.Checksum is Success
		printmsg("BL_DEBUG_MSG: Checksum Success !!\r\n");

		// 3.Send ACK
		BootLoader_Send_ACK(BL_command_code,BL_FLASH_ERASE_REPLY_SIZE);

		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		BL_Erase_Status=BL_Execute_Flash_Erase(BL_page_number,BL_number_of_pages);
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);

		if(BL_Erase_Status==FLASH_ERASE_SUCCESS)
		{
			printmsg("BL_DEBUG_MSG: FLASH_Erase_SUCCESS !!\r\n");
		}
		else
		{
			printmsg("BL_DEBUG_MSG: FLASH_Erase_FAIL !!\r\n");
		}


//	}
//	else
	{
		// 2.Checksum is Wrong , Send NACK
		printmsg("BL_DEBUG_MSG: Checksum FAIL !!\r\n");
		BootLoader_Send_NACK();
	}

}
void BL_hanlde_memWrite_cmd(uint8_t* pbuffer)
{
	uint8_t BL_length_to_follow = pbuffer[0]; // length Segment in The command Packet
	uint8_t BL_command_code = pbuffer[1];
	uint8_t payload_length= pbuffer[6];
	uint32_t Base_Mem_Address = *(uint32_t*)pbuffer[2];

	// Total Length of The Command Packet
	uint32_t command_Packet_length = BL_length_to_follow + BL_LENGTH_TO_FOLLOW_SEG_SIZE ;

	printmsg("BL_DEBUG_MSG: BL_hanlde_memWrite_cmd\r\n");

	// 1.Verify The CRC

	// Extract CRC_32 word sent by The Host
	//CRC_Host Contains The Data of The beginning of BL_CRC_SEG till the end of The command Packet
	uint32_t CRC_Host = * ((uint32_t*) (pbuffer+command_Packet_length-BL_CRC_SEG_SIZE) );

	if(!BootLoader_Verify_CRC(&pbuffer[0],command_Packet_length-BL_CRC_SEG_SIZE,CRC_Host))
	{
		printmsg("BL_DEBUG_MSG: Checksum Success !!\r\n");

		// 3.Send ACK
		BootLoader_Send_ACK(BL_command_code,BL_MEM_WRITE_REPLY_SIZE);
		printmsg("BL_DEBUG_MSG: Base Mem Write Address: %#x\r\n");

		if(BL_verify_address(Base_Mem_Address)==ADDRESS_VALID)
		{
			printmsg("BL_DEBUG_MSG: Valid Base Mem Write Address \r\n");

			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);

			uint8_t status=execute_Mem_Write(&pbuffer[7],Base_Mem_Address,payload_length);

			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		}
		else
		{
			printmsg("BL_DEBUG_MSG: Invalid Base Mem Write Address \r\n");

		}
	}
	else
	{
		// 2.Checksum is Wrong , Send NACK
		printmsg("BL_DEBUG_MSG: Checksum FAIL !!\r\n");
		BootLoader_Send_NACK();
	}

}
void BL_hanlde_memRead_cmd(uint8_t* pbuffer)
{

}
void BL_handle_ReadSectorStatus_cmd(uint8_t* pbuffer)
{
}
