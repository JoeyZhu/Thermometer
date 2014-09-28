/**************************************************************************************************'
  Filename:       hal_temp112.c
  Revised:        $Date: 2014-09-19 07:58:08 -0700 (Mon, 25 Mar 2013) $
  Revision:       $Revision: 000 $

  Description:    Driver for the temp112 sensor

**************************************************************************************************/

/* ------------------------------------------------------------------------------------------------
*                                          Includes
* ------------------------------------------------------------------------------------------------
*/
#include "hal_temp112.h"
//#include "hal_sensor.h"
#include "hal_i2c.h"

/* ------------------------------------------------------------------------------------------------
*                                           Constants
* ------------------------------------------------------------------------------------------------
*/

// Sensor I2C address
#define HAL_TEMP112_I2C_ADDRESS      0x48       //rd/wr not included

#define TEMP112_TEMP_READ_R                  0x00
#define TEMP112_CONFIG_ADDR             0x01

// Internal commands
#define SHT21_CMD_TEMP_T_H         0xE3 // command trig. temp meas. hold master
#define SHT21_CMD_HUMI_T_H         0xE5 // command trig. humidity meas. hold master
#define SHT21_CMD_TEMP_T_NH        0xF3 // command trig. temp meas. no hold master
#define SHT21_CMD_HUMI_T_NH        0xF5 // command trig. humidity meas. no hold master
#define SHT21_CMD_WRITE_U_R        0xE6 // command write user register
#define SHT21_CMD_READ_U_R         0xE7 // command read user register
#define SHT21_CMD_SOFT_RST         0xFE // command soft reset

#define HUMIDITY				           0x00
#define TEMPERATURE			           0x01

#define USR_REG_MASK               0x38  // Mask off reserved bits (3,4,5)
#define USR_REG_DEFAULT            0x02  // Disable OTP reload
#define USR_REG_RES_MASK           0x7E  // Only change bits 0 and 7 (meas. res.)
#define USR_REG_11BITRES           0x81  // 11-bit resolution

/* ------------------------------------------------------------------------------------------------
*                                           Type Definitions
* ------------------------------------------------------------------------------------------------
*/

/* ------------------------------------------------------------------------------------------------
*                                           Local Functions
* ------------------------------------------------------------------------------------------------
*/
bool HalTemp112ReadReg(uint8 addr, uint8 *pBuf, uint8 nBytes);
bool HalTemp112WriteReg(uint8 addr, uint8 *pBuf, uint8 nBytes);
/* ------------------------------------------------------------------------------------------------
*                                           Local Variables
* ------------------------------------------------------------------------------------------------
*/
static uint8 buf[6];                      // Data buffer
static bool  success;
static uint8 buffer[24];

/**************************************************************************************************
* @fn          HalHumiInit
*
* @brief       Initialise the humidity sensor driver
*
* @return      none
**************************************************************************************************/
void HalTemp112Init(void)
{
  uint16 config;
  
  HalI2CInit(HAL_TEMP112_I2C_ADDRESS,i2cClock_267KHZ);
  //OS = 1, R0R1 read only, F1F0 don't care, POL don't care, TM don't care, SD = 1, so BYTE1 is 0xFF
  //CR1CR0 don't care, AL read only, EM = 0, so BYTE2 is 0x00
  config = 0x00FF;      //or 0xFF00
  //HalTemp112ReadReg(TEMP112_TEMP_READ_R,(uint8 *)&val_test,2);
  HalTemp112WriteReg(TEMP112_CONFIG_ADDR,(uint8 *)&config,2);
  
  //should read back to verify to indicate tmp112 chip ok?
  //HalTemp112ReadReg(TEMP112_TEMP_READ_R,(uint8 *)&val_test,2); 
  
  //begin one shot is same as init
  //read temp first
  //result_test = BUILD_UINT16(HI_UINT16(val_test), LO_UINT16(val_test));
  //temp_test = (uint16)(((uint32)(result_test>>4) * 25) >> 2);  
  //config = 0x00FF;      //or 0xFF00
  //HalTemp112WriteReg(TEMP112_CONFIG_ADDR,(uint8 *)&config,2);
}


uint16 HalTemp112ReadTemp(void){  
  uint16 val;
  HalTemp112Init();
  //HalI2CInit(HAL_TEMP112_I2C_ADDRESS,i2cClock_267KHZ);
  HalTemp112ReadReg(TEMP112_TEMP_READ_R,(uint8 *)&val,2);
 // config = 0x00FF;      //or 0xFF00
 // HalTemp112WriteReg(TEMP112_CONFIG_ADDR,(uint8 *)&config,2);  
    //test
    //HalTemp112ReadReg(TEMP112_TEMP_READ_R,(uint8 *)&temp_high_after,2);  
    //temp_test = temp_high_after;
   // HalTemp112WriteReg(2,(uint8 *)&config,2);
  return (uint16)(((uint32)((BUILD_UINT16(HI_UINT16(val), LO_UINT16(val)))>>4) * 25) >> 2);;
}  
/**************************************************************************************************
 * @fn          HalSensorReadReg
 *
 * @brief       This function implements the I2C protocol to read from a sensor. The sensor must
 *              be selected before this routine is called.
 *
 * @param       addr - which register to read
 * @param       pBuf - pointer to buffer to place data
 * @param       nBytes - numbver of bytes to read
 *
 * @return      TRUE if the required number of bytes are reveived
 **************************************************************************************************/
bool HalTemp112ReadReg(uint8 addr, uint8 *pBuf, uint8 nBytes){
  uint8 i = 0;

  /* Send address we're reading from */
  if (HalI2CWrite(1,&addr) == 1)
  {
    /* Now read data */
    i = HalI2CRead(nBytes,pBuf);
  }

  return i == nBytes;
}
/**************************************************************************************************
* @fn          HalSensorWriteReg
* @brief       This function implements the I2C protocol to write to a sensor. he sensor must
*              be selected before this routine is called.
*
* @param       addr - which register to write
* @param       pBuf - pointer to buffer containing data to be written
* @param       nBytes - number of bytes to write
*
* @return      TRUE if successful write
*/
bool HalTemp112WriteReg(uint8 addr, uint8 *pBuf, uint8 nBytes){
  uint8 i;
  uint8 *p = buffer;

  /* Copy address and data to local buffer for burst write */
  *p++ = addr;
  for (i = 0; i < nBytes; i++)
  {
    *p++ = *pBuf++;
  }
  nBytes++;

  /* Send address and data */
  i = HalI2CWrite(nBytes, buffer);
  if ( i!= nBytes){
    i = i;
  }
    //write error indicate

  return (i == nBytes);
}  
/**************************************************************************************************
* @fn          HalHumiExecMeasurementStep
*
* @brief       Execute measurement step
*
* @return      none
*/
bool HalHumiExecMeasurementStep(uint8 state)
{
 /* HalHumiSelect();

  switch (state)
  {
    case 0:
      // Turn on DC-DC control
      HalDcDcControl(ST_HUMID,true);

      // Start temperature read
      success = HalHumiWriteCmd(SHT21_CMD_TEMP_T_NH);
      break;

    case 1:
      // Read and store temperature value
      if (success)
      {
        success = HalHumiReadData(buf, DATA_LEN);

        // Start for humidity read
        if (success)
        {
          success = HalHumiWriteCmd(SHT21_CMD_HUMI_T_NH);
        }
      }
      break;

    case 2:
      // Read and store humidity value
      if (success)
      {
        success = HalHumiReadData(buf+DATA_LEN, DATA_LEN);
      }

      // Turn of DC-DC control
      HalDcDcControl(ST_HUMID,false);
      break;
  }
*/
  return success;
}


/**************************************************************************************************
* @fn          HalHumiReadMeasurement
*
* @brief       Get humidity sensor data
*
* @return      none
*/
bool HalHumiReadMeasurement(uint8 *pBuf)
{
  // Store temperature
  pBuf[0] = buf[1];
  pBuf[1] = buf[0];

  // Store humidity
  pBuf[2] = buf[4];
  pBuf[3] = buf[3];

  return success;
}



/**************************************************************************************************
* @fn          HalHumiTest
*
* @brief       Humidity sensor self test
*
* @return      none
**************************************************************************************************/
bool HalHumiTest(void)
{
/*  uint8 val;

  HalHumiSelect();

  // Verify default value
  ST_ASSERT(HalSensorReadReg(SHT21_CMD_READ_U_R,&val,1));
  ST_ASSERT(val==usr);
*/
  return TRUE;
}


/* ------------------------------------------------------------------------------------------------
*                                           Private functions
* -------------------------------------------------------------------------------------------------
*/


/*********************************************************************
*********************************************************************/

