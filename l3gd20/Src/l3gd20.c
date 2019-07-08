#include "l3gd20.h"

uint8_t dataRecieved	= 0;
uint8_t dataTransmitted	= 0;

void L3GD20_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);

void L3GD20_Init(L3GD20_InitTypeDef *L3GD20_InitStruct)
{  
  uint8_t ctrl1 = 0x00, ctrl4 = 0x00;
  
  /* Configure MEMS: data rate, power mode, full scale and axes */
  ctrl1 |= (uint8_t) (L3GD20_InitStruct->Power_Mode | L3GD20_InitStruct->Output_DataRate | \
                    L3GD20_InitStruct->Axes_Enable | L3GD20_InitStruct->Band_Width);
  
  ctrl4 |= (uint8_t) (L3GD20_InitStruct->BlockData_Update | L3GD20_InitStruct->Endianness | \
                    L3GD20_InitStruct->Full_Scale);
                    
  /* Write value to MEMS CTRL_REG1 regsister */
  L3GD20_Write(&ctrl1, L3GD20_CTRL_REG1_ADDR, 1);
  
  /* Write value to MEMS CTRL_REG4 regsister */
  L3GD20_Write(&ctrl4, L3GD20_CTRL_REG4_ADDR, 1);
}

void L3GD20_RebootCmd(void)
{
  uint8_t tmpreg;
  
  /* Read CTRL_REG5 register */
  L3GD20_Read(&tmpreg, L3GD20_CTRL_REG5_ADDR, 1);
  
  /* Enable or Disable the reboot memory */
  tmpreg |= L3GD20_BOOT_REBOOTMEMORY;
  
  /* Write value to MEMS CTRL_REG5 regsister */
  L3GD20_Write(&tmpreg, L3GD20_CTRL_REG5_ADDR, 1);
}

void L3GD20_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
	if (NumByteToRead > 0x01)
	{
		ReadAddr |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
	}
	else
	{
		ReadAddr |= (uint8_t)READWRITE_CMD;
	}
	// Set Chip Select Low at the start of transmission
	L3GD20_CS_ENABLE();

	// Set the adress of the register to read
	L3GD20_SendByte(ReadAddr);

	// Recieve the data that will be read from the device
	while (NumByteToRead > 0x00)
	{
		//Send Dummy Byte to generate the SPI clock to L3GD20
		*pBuffer = L3GD20_SendByte(DUMMY_BYTE);
		NumByteToRead--;
		pBuffer++;
	}

	// Set Chip Select High at the end of transmission
	L3GD20_CS_DISABLE();
}

void L3GD20_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
  /* Configure the MS bit: 
       - When 0, the address will remain unchanged in multiple read/write commands.
       - When 1, the address will be auto incremented in multiple read/write commands.
  */
  if(NumByteToWrite > 0x01)
  {
    WriteAddr |= (uint8_t)MULTIPLEBYTE_CMD;
  }
  /* Set chip select Low at the start of the transmission */
  L3GD20_CS_ENABLE();
  
  /* Send the Address of the indexed register */
  L3GD20_SendByte(WriteAddr);
  /* Send the data that will be written into the device (MSB First) */
  while(NumByteToWrite >= 0x01)
  {
    L3GD20_SendByte(*pBuffer);
    NumByteToWrite--;
    pBuffer++;
  }
  
  /* Set chip select High at the end of the transmission */ 
  L3GD20_CS_DISABLE();
}

static uint8_t L3GD20_SendByte(uint8_t byte)
{
    //HAL_SPI_Transmit_IT(&hspi5, &byte, 1);
    uint8_t recByte = 0;

    // Write byte to Data Register of SPI5
    SPI5->DR = byte;

    // Wait while TXE flag is 0
    while (!dataTransmitted);
    dataTransmitted = 0;
    //HAL_SPI_Recieve_IT(&hspi5, &recByte, 1);

    // Wait while data comes to Data Register of SPI5
    while (!dataRecieved);
    dataRecieved = 0;
    recByte = SPI5->DR;

    return recByte;
}

