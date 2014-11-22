/*****************************************************************************/
// @file 	cc112x.c  
//    
// @brief 	Implementation file for basic and neccessary functions
//          to communicate with CC112X over SPI
//				 
//  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
/****************************************************************************/


/******************************************************************************
 * INCLUDES
 */
#include "cc112x.h"
#include "board.h"
#include "stm32f10x_spi.h"

/******************************************************************************
 * FUNCTIONS
 */
/*******************************************************************************
* @brief  Configure the SPI interface for CC1120
* @param  void
* @retval void
*******************************************************************************/
void CC1120_SPI_Configuration(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
    SPI_InitTypeDef     SPI_InitStructure;

    /* Enable SPI SCK, MOSI, MISO and NSS GPIO PB12 PB13 PB14 PB15 clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    
    /* Enable the SPI peripheral */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    
    /* Enable the AFIO peripheral */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    /*  SPI pin config */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

    /* SPI SCK pin configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_CC1120_SCK;
    GPIO_Init(GPIO_Port_CC1120_SPI, &GPIO_InitStructure);

    /* SPI MOSI pin configuration */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_CC1120_MOSI;
    GPIO_Init(GPIO_Port_CC1120_SPI, &GPIO_InitStructure);

    /* SPI MISO pin configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_CC1120_MISO;
    GPIO_Init(GPIO_Port_CC1120_SPI, &GPIO_InitStructure);
    
    /* SPI NSS pin configuration */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIO_Port_CC1120_NSS, &GPIO_InitStructure);
	GPIO_SetBits(GPIO_Port_CC1120_NSS,GPIO_Pin_4);

     /* SPI configuration -------------------------------------------------------*/
    SPI_I2S_DeInit(SPI1);
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;   
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; // (8M/32 =250KHz)
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStructure);

    /* Enable the SPI peripheral */
    SPI_Cmd(SPI1, DISABLE);
}

/*******************************************************************************
* @brief  Sends a byte through the SPI interface and return the byte
* @param  SPIx: To select the SPIx/I2Sx peripheral
* @param  temp: Data to be transmitted.
* @retval The value of the received data.
*******************************************************************************/
uint8_t CC1120_SPIReadWriteByte(SPI_TypeDef* SPIx,uint8_t temp)
{
	/* Loop while DR register in not emplty */
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET);
	
    /* Send byte through the SPIx peripheral */
    SPI_I2S_SendData(SPIx,temp);
	
    /* Wait to receive a byte */
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET);
    
    /* Return the byte read from the SPI bus */
    return (SPI_I2S_ReceiveData(SPIx));
}

/*******************************************************************************
 * @fn          trxReadWriteBurstSingle
 * @brief       When the address byte is sent to the SPI slave, the next byte
 *              communicated is the data to be written or read. The address
 *              byte that holds information about read/write -and single/
 *              burst-access is provided to this function.
 *              Depending on these two bits this function will write len bytes to
 *              the radio in burst mode or read len bytes from the radio in burst
 *              mode if the burst bit is set. If the burst bit is not set, only
 *              one data byte is communicated.
 *              NOTE: This function is used in the following way:
 *
 *              TRXEM_SPI_BEGIN();
 *              while(TRXEM_PORT_IN & TRXEM_SPI_MISO_PIN);
 *              ...[Depending on type of register access]
 *              trxReadWriteBurstSingle(uint8 addr,uint8 *pData,uint16 len);
 *              TRXEM_SPI_END();
 * input parameters
 * @param       none
 * output parameters
 * @return      void
 */
static void trxReadWriteBurstSingle(uint8_t addr,uint8_t *pData,uint16_t len)
{
    uint16_t i;
    
    /* Communicate len number of bytes: if RX - the procedure sends 0x00 to push bytes from slave*/
    if(addr&RADIO_READ_ACCESS)
    {
        if(addr&RADIO_BURST_ACCESS)
        {
            for (i = 0; i < len; i++)
            {   
                /* Possible to combining read and write as one access type */
                *pData = CC1120_SPIReadWriteByte(SPI1,0); 
                pData++;
            }
        }
        else
        {
            *pData = CC1120_SPIReadWriteByte(SPI1,0); 
        }
    }
    else
    {
        if(addr&RADIO_BURST_ACCESS)
        {
            /* Communicate len number of bytes: if TX - the procedure doesn't overwrite pData */
            for (i = 0; i < len; i++)
            {
                CC1120_SPIReadWriteByte(SPI1,*pData); 
                pData++;
            }
        }
        else
        {
            CC1120_SPIReadWriteByte(SPI1,*pData); 
        }
    }
    
    return;
}

/*******************************************************************************
 * @fn          trx8BitRegAccess
 * @brief       This function performs a read or write from/to a 8bit register
 *              address space. The function handles burst and single read/write
 *              as specfied in addrByte. Function assumes that chip is ready.
 * input parameters
 * @param       accessType - Specifies if this is a read or write and if it's
 *                           a single or burst access. Bitmask made up of
 *                           RADIO_BURST_ACCESS/RADIO_SINGLE_ACCESS/
 *                           RADIO_WRITE_ACCESS/RADIO_READ_ACCESS.
 * @param       addrByte - address byte of register.
 * @param       pData    - data array
 * @param       len      - Length of array to be read(TX)/written(RX)
 * output parameters
 * @return      chip status
 */
rfStatus_t trx8BitRegAccess(uint8_t accessType, uint8_t addrByte, uint8_t *pData, uint8_t len)
{
    uint8_t readValue;
    
    SPI_Cmd(SPI1, ENABLE);
    
    /* Pull CS_N low and wait for SO to go low before communication starts */
    GPIO_ResetBits(GPIO_Port_CC1120_NSS,GPIO_Pin_4);       // CS = 0
    while((GPIO_ReadInputDataBit(GPIO_Port_CC1120_SPI, GPIO_Pin_CC1120_MISO) & Bit_SET));

    /* send register address byte */
    readValue = CC1120_SPIReadWriteByte(SPI1,accessType|addrByte);

    trxReadWriteBurstSingle(accessType|addrByte,pData,len);
    
    GPIO_SetBits(GPIO_Port_CC1120_NSS,GPIO_Pin_4);       // CS = 0
    
    SPI_Cmd(SPI1, DISABLE);
    
    /* return the status byte value */
    return(readValue);
}

/******************************************************************************
 * @fn          trx16BitRegAccess
 * @brief       This function performs a read or write in the extended adress
 *              space of CC112X.
 * input parameters
 * @param       accessType - Specifies if this is a read or write and if it's
 *                           a single or burst access. Bitmask made up of
 *                           RADIO_BURST_ACCESS/RADIO_SINGLE_ACCESS/
 *                           RADIO_WRITE_ACCESS/RADIO_READ_ACCESS.
 * @param       extAddr - Extended register space address = 0x2F.
 * @param       regAddr - Register address in the extended address space.
 * @param       *pData  - Pointer to data array for communication
 * @param       len     - Length of bytes to be read/written from/to radio
 * output parameters
 * @return      rfStatus_t
 */
rfStatus_t trx16BitRegAccess(uint8_t accessType, uint8_t extAddr, uint8_t regAddr, uint8_t *pData, uint8_t len)
{
    uint8_t readValue;

    SPI_Cmd(SPI1, ENABLE);

    /* Pull CS_N low and wait for SO to go low before communication starts */
    GPIO_ResetBits(GPIO_Port_CC1120_NSS,GPIO_Pin_4);       // CS = 0
    while((GPIO_ReadInputDataBit(GPIO_Port_CC1120_SPI, GPIO_Pin_CC1120_MISO) & Bit_SET));

    /* send extended address byte with access type bits set */
    readValue = CC1120_SPIReadWriteByte(SPI1,accessType|extAddr);  

    CC1120_SPIReadWriteByte(SPI1,regAddr); 
    /* Communicate len number of bytes */
    trxReadWriteBurstSingle(accessType|extAddr,pData,len);

    GPIO_SetBits(GPIO_Port_CC1120_NSS,GPIO_Pin_4);       // CS = 0

    SPI_Cmd(SPI1, DISABLE);

    /* return the status byte value */
    return(readValue);
}

/*******************************************************************************
 * @fn          trxSpiCmdStrobe
 * @brief       Send command strobe to the radio. Returns status byte read
 *              during transfer of command strobe. Validation of provided
 *              is not done. Function assumes chip is ready.
 * input parameters
 * @param       cmd - command strobe
 * output parameters
 * @return      status byte
 */
rfStatus_t trxSpiCmdStrobe(uint8_t cmd)
{
    uint8_t rc;
    
    SPI_Cmd(SPI1, ENABLE);

    /* Pull CS_N low and wait for SO to go low before communication starts */
    GPIO_ResetBits(GPIO_Port_CC1120_NSS,GPIO_Pin_4);       // CS = 0
    while((GPIO_ReadInputDataBit(GPIO_Port_CC1120_SPI, GPIO_Pin_CC1120_MISO) & Bit_SET));
    
    /* send extended address byte with access type bits set */
    rc = CC1120_SPIReadWriteByte(SPI1,cmd);  

    GPIO_SetBits(GPIO_Port_CC1120_NSS,GPIO_Pin_4);       // CS = 0

    SPI_Cmd(SPI1, DISABLE);

    return(rc);
}

/******************************************************************************
 * @fn          cc112xSpiReadReg
 * @brief       Read value(s) from config/status/extended radio register(s).
 *              If len  = 1: Reads a single register
 *              if len != 1: Reads len register values in burst mode 
 * input parameters
 * @param       addr   - address of first register to read
 * @param       *pData - pointer to data array where read bytes are saved
 * @param       len   - number of bytes to read
 * output parameters
 * @return      rfStatus_t
 */
rfStatus_t cc112xSpiReadReg(uint16_t addr, uint8_t *pData, uint8_t len)
{
    uint8_t tempExt  = (uint8_t)(addr>>8);
    uint8_t tempAddr = (uint8_t)(addr & 0x00FF);
    uint8_t rc;

    /* Checking if this is a FIFO access -> returns chip not ready  */
    if((CC112X_SINGLE_TXFIFO<=tempAddr)&&(tempExt==0)) return STATUS_CHIP_RDYn_BM;

    /* Decide what register space is accessed */
    if(!tempExt)
    {
        rc = trx8BitRegAccess((RADIO_BURST_ACCESS|RADIO_READ_ACCESS),tempAddr,pData,len);
    }
    else if (tempExt == 0x2F)
    {
        rc = trx16BitRegAccess((RADIO_BURST_ACCESS|RADIO_READ_ACCESS),tempExt,tempAddr,pData,len);
    }
    
    return (rc);
}

/******************************************************************************
 * @fn          cc112xSpiWriteReg
 * @brief       Write value(s) to config/status/extended radio register(s).
 *              If len  = 1: Writes a single register
 *              if len  > 1: Writes len register values in burst mode 
 * input parameters
 * @param       addr   - address of first register to write
 * @param       *pData - pointer to data array that holds bytes to be written
 * @param       len    - number of bytes to write
 * output parameters
 * @return      rfStatus_t
 */
rfStatus_t cc112xSpiWriteReg(uint16_t addr, uint8_t *pData, uint8_t len)
{
    uint8_t tempExt  = (uint8_t)(addr>>8);
    uint8_t tempAddr = (uint8_t)(addr & 0x00FF);
    uint8_t rc;

    /* Checking if this is a FIFO access - returns chip not ready */
    if((CC112X_SINGLE_TXFIFO<=tempAddr)&&(tempExt==0)) return STATUS_CHIP_RDYn_BM;

    /* Decide what register space is accessed */  
    if(!tempExt)
    {
        rc = trx8BitRegAccess((RADIO_BURST_ACCESS|RADIO_WRITE_ACCESS),tempAddr,pData,len);
    }
    else if (tempExt == 0x2F)
    {
        rc = trx16BitRegAccess((RADIO_BURST_ACCESS|RADIO_WRITE_ACCESS),tempExt,tempAddr,pData,len);
    }
    
    return (rc);
}

/*******************************************************************************
 * @fn          cc112xSpiWriteTxFifo
 * @brief       Write pData to radio transmit FIFO.
 * input parameters
 * @param       *pData - pointer to data array that is written to TX FIFO
 * @param       len    - Length of data array to be written
 * output parameters
 * @return      rfStatus_t
 */
rfStatus_t cc112xSpiWriteTxFifo(uint8_t *pData, uint8_t len)
{
    uint8_t rc;
    rc = trx8BitRegAccess(0x00,CC112X_BURST_TXFIFO, pData, len);
    return (rc);
}

/*******************************************************************************
 * @fn          cc112xSpiReadRxFifo
 * @brief       Reads RX FIFO values to pData array
 * input parameters
 * @param       *pData - pointer to data array where RX FIFO bytes are saved
 * @param       len    - number of bytes to read from the RX FIFO
 * output parameters
 * @return      rfStatus_t
 */
rfStatus_t cc112xSpiReadRxFifo(uint8_t * pData, uint8_t len)
{
    uint8_t rc;
    rc = trx8BitRegAccess(0x00,CC112X_BURST_RXFIFO, pData, len);
    return (rc);
}

/******************************************************************************
 * @fn      cc112xGetTxStatus(void)       
 * @brief   This function transmits a No Operation Strobe (SNOP) to get the 
 *          status of the radio and the number of free bytes in the TX FIFO.
 *          Status byte:
 *          ---------------------------------------------------------------------------
 *          |          |            |                                                 |
 *          | CHIP_RDY | STATE[2:0] | FIFO_BYTES_AVAILABLE (free bytes in the TX FIFO |
 *          |          |            |                                                 |
 *          ---------------------------------------------------------------------------
 * input parameters
 * @param   none
 * output parameters    
 * @return  rfStatus_t 
 */
rfStatus_t cc112xGetTxStatus(void)
{
    return(trxSpiCmdStrobe(CC112X_SNOP));
}

/******************************************************************************
 *  @fn       cc112xGetRxStatus(void)
 *  @brief   
 *            This function transmits a No Operation Strobe (SNOP) with the 
 *            read bit set to get the status of the radio and the number of 
 *            available bytes in the RXFIFO.   
 *            Status byte:
 *            
 *            --------------------------------------------------------------------------------
 *            |          |            |                                                      |
 *            | CHIP_RDY | STATE[2:0] | FIFO_BYTES_AVAILABLE (available bytes in the RX FIFO |
 *            |          |            |                                                      |
 *            --------------------------------------------------------------------------------
 * input parameters
 * @param     none
 * output parameters    
 * @return    rfStatus_t 
 */
rfStatus_t cc112xGetRxStatus(void)
{
    return(trxSpiCmdStrobe(CC112X_SNOP | RADIO_READ_ACCESS));
}
