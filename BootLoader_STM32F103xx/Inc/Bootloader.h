
#ifndef BOOTLOADER_H_
#define BOOTLOADER_H_

#include <string.h>

#include "stm32f1xx_hal.h"

/*****************************BOOTLOADER_MACROS*****************************************/
#define D_UART							&huart1
#define C_UART							&huart1
#define FLASH_SECTOR2_BASE_ADDRESS 		0x08008000
#define BL_RX_LEN						200
#define BL_VERSION 						0x10 // version 1.0

//Bootloader Commands
#define BL_GET_VER						0x51 // BootLoader Get Version of The Micro_controller
#define BL_GET_HELP						0x52 // Bootloader Get The Supported Commands
#define BL_GET_CID						0x53 // Bootloader Get chip ID
#define BL_GET_RDP_STATUS				0x54 // Bootloader Reed Flash Protection Status
#define BL_GO_TO_ADDR					0x55 // Bootloader Go To Specific address
#define BL_FLASH_ERASE					0x56 // Bootloader mass Erase or Sector Erase
#define BL_MEM_WRITE					0x57 // Bootloader memory Write
#define BL_EN_R_W_PROTECT				0x58 // BL Enable R/W Protection on different Sectors on Flash
#define BL_MEM_READ						0x59 // BL Read Data from Different Memories of MC
#define BL_READ_SECTOR_STATUS			0x5A // BL Read all the sector protection status
#define BL_DIS_R_W_PROTECT				0x5C // BL Disable R/W Protection on different Sectors on Flash

//Bootloader ACK and NACK Bytes
#define BL_ACK							0xA5
#define BL_NACK							0x7F

// CRC Verification
#define VERIFY_CRC_FAIL 				1
#define VERIFY_CRC_SUCCESS 				0

//Packet Segments
#define BL_LENGTH_TO_FOLLOW_SEG_SIZE	1
#define BL_COMMAND_CODE_SEG_SIZE		1
#define BL_CRC_SEG_SIZE					4

// BootLoader Replies Size
#define BL_VERSION_REPLY_SIZE			1
#define BL_HELP_REPLY_SIZE				10
#define BL_CID_REPLY_SIZE				2
#define BL_RDP_STATUS_REPLY_SIZE		1
#define BL_GO_TO_ADDR_REPLY_SIZE		1
#define BL_FLASH_ERASE_REPLY_SIZE		1
#define BL_MEM_WRITE_REPLY_SIZE			1
#define BL_EN_R_W_PROTECT_REPLY_SIZE	1
#define BL_SECTOR_STATUS_REPLY_SIZE		2
#define BL_DIS_R_W_PROTECT_REPLY_SIZE	1

// Go Address cmd Defines
#define ADDRESS_VALID					0x00
#define ADDRESS_INVALID					0x01
#define SRAM_SIZE 						(20*1024) //STM32F103C8 has 20KB SRAM Memory
#define SRAM_END						(SRAM_BASE+SRAM_SIZE)
#define FLASH_SIZE 						(64*1024) //STM32F103C8 has 64KB Flash Memory
#define FLASH_END						(FLASH_BASE+FLASH_SIZE)

// Flash Erase cmd Defines
#define FLASH_ERASE_SUCCESS 			0x00
#define FLASH_ERASE_FAIL 				0x01
#define FLASH_INVALID_SECTOR			0x02
/*****************************EXTERN VARIABLES******************************************/
extern UART_HandleTypeDef huart1;
extern CRC_HandleTypeDef hcrc;
/*****************************GLOBAL VARIABLES******************************************/
uint8_t BL_rx_buffer[BL_RX_LEN];

/*****************************FUNCTION DECLARATION**************************************/
extern void printmsg(char *format,...);
void BootLoader_Jump_to_User_APP();
void BootLoader_UART_Read_Host_CMD();
void BootLoader_Send_ACK(uint8_t command_code , uint8_t length_of_reply);
void BootLoader_Send_NACK(void);
uint8_t BootLoader_Verify_CRC(uint8_t *pData ,uint32_t length ,uint32_t CRC_Host);
uint8_t BootLoader_get_Version(void);
void BootLoader_UART_Write_Data(uint8_t *pbuffer, uint32_t length);
uint16_t get_MCU_ID_Code(void);
uint8_t BL_verify_address(uint32_t go_address);
uint8_t BL_Execute_Flash_Erase(uint8_t BL_page_address, uint8_t BL_number_of_pages);
uint8_t execute_Mem_Write(uint8_t *pbuffer,uint32_t Base_Mem_Address,uint8_t payload_length);

//Bootloader Handle Commands Function Prototype
void BL_hanlde_getVersion_cmd(uint8_t* pbuffer);
void BL_hanlde_getHelp_cmd(uint8_t* pbuffer);
void BL_hanlde_getCID_cmd(uint8_t* pbuffer);
void BL_hanlde_getRDP_cmd(uint8_t* pbuffer);
void BL_hanlde_goAddress_cmd(uint8_t* pbuffer);
void BL_hanlde_flashErase_cmd(uint8_t* pbuffer);
void BL_hanlde_memWrite_cmd(uint8_t* pbuffer);
void BL_hanlde_memRead_cmd(uint8_t* pbuffer);
void BL_handle_ReadSectorStatus_cmd(uint8_t* pbuffer);



#endif /* BOOTLOADER_H_ */
