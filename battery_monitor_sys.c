/*
 ******************************************************************************
 * File : battery_monitor.c
 * Version : Initial Draft (0.0)
 * Date : 02-11-2021
 * Brief Description : Battery Monitoring System (BMS)
 * Author: Rutvij Trivedi (rutvij.trivedi@sisignals.com)
 * bq27421-g1, Legacy - standard peripherals library
 ******************************************************************************
 */
/* -------------------------REVISIONS ----------------------------------------- 
 *  Date         Name           Problem    Description
 *                              Number  
 * ----------------------------------------------------------------------------
 * 02/11/2021     -      N/A       Initial Draft, File Created
 ******************************************************************************
 */
#include "battery_monitor_sys.h"

#ifdef ENABLE_PRINT
#include "dbg_uart.h"
#endif

#include <stdio.h>
#include <string.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* BSP */
#include "board.h"
#include "i2c1.h"
#include "i2c2.h"


#define DM_REGS_COUNT    7

static uint8_t batteryPercentage, batteryStatus;
static uint8_t prev_batteryPercentage = 0, prev_batteryStatus = 0;

volatile uint8_t BatteryStatus_Changed = 0;
  
//as per data sheet - bq27421-g1
enum bq27xxx_reg_index {
	BQ27XXX_REG_CTRL = 0,
	BQ27XXX_REG_TEMP,
	BQ27XXX_REG_INT_TEMP,
	BQ27XXX_REG_VOLT,
	BQ27XXX_REG_AI,
	BQ27XXX_REG_FLAGS,
	BQ27XXX_REG_TTE,
	BQ27XXX_REG_TTF,
	BQ27XXX_REG_TTES,
	BQ27XXX_REG_TTECP,
	BQ27XXX_REG_NAC,
	BQ27XXX_REG_FCC,
	BQ27XXX_REG_CYCT,
	BQ27XXX_REG_AE,
	BQ27XXX_REG_SOC,
	BQ27XXX_REG_DCAP,
	BQ27XXX_POWER_AVG,
	NUM_REGS
};

static const uint8_t bq274xx_regs[NUM_REGS] = 
{
    0x00,	/* CONTROL	*/
    0x02,	/* TEMP		*/
    0x1e,	/* INT TEMP	*/
    0x04,	/* VOLT		*/
    0x10,	/* AVG CURR	*/
    0x06,	/* FLAGS	*/
    0xFF,	/* TTE - NA	*/
    0xFF,	/* TTF - NA	*/
    0xFF,	/* TTES - NA	*/
    0xFF,	/* TTECP - NA	*/
    0x08,	/* NAC		*/
    0x0E,	/* FCC		*/
    0xFF,	/* CYCT - NA	*/
    0xFF,	/* AE - NA	*/
    0x1C,	/* SOC		*/
    0x3C,	/* DCAP - NA	*/
    0x18,	/* AP		*/  
};

struct dm_reg {
	uint8_t subclass;
	uint8_t offset;
	uint8_t len;
	uint32_t data;
};

/* Update DM_REGS_COUNT Macro above when add/remove array entry */
/* See Quick Start Guide Page 3 */
static const struct dm_reg bq274xx_dm_regs[] = {
#ifndef BAT_DET_HW
        {64, 0, 2, 0x05F8},     /* OpConfig BIE = 0 to rely on BAT_INSERT Command */
#else
        {64, 0, 2, 0x25F8},     /* OpConfig BIE = 0 to rely on BAT_INSERT Command, changed to 1, GPOUT Active Low */
#endif
	{82, 0, 2, 16384},	/* Qmax */
	{82, 5, 1, 0x81},	/* Load Select */
	//{82, 10, 2, 500},	/* Design Capacity = 500mAh */
        {82, 10, 2, 450},	/* Design Capacity = 450mAh */
	{82, 12, 2, 1900},	/* Design Energy = 3.7 * Design Capacity for G1A For G1B = 3.8 * Design Capacity*/
	{82, 16, 2, 3100},	/* Terminate Voltage - Just Taking for now as 3V as per battery_datasheet.pdf dead voltage Page 1 */
	//{82, 27, 2, 250},	/* Taper rate = Design Capacity/ (0.1 * Taper Current) , Taper Current at which full charge capacity reached and charging is now about to stop nearly 20mA Taking = 500/(0.1 * 20) = 250 */
        {82, 27, 2, 115},	/* Taper rate = Design Capacity/ (0.1 * Taper Current) , Taper Current at which full charge capacity reached and charging is now about to stop nearly 20mA Taking = 500/(0.1 * 8) = 625 */
};

int value, is_sealed, is_cfgupdate_mode, is_Battery_Inserted;

#ifdef ENABLE_PRINT
static char bat_log[50];
#endif


/*
 *      Standard Commands
 */
void I2C2_battery_write_register(uint8_t SlaveAddr,uint8_t RegisterAddr, uint8_t RegisterValue)
{
    xSemaphoreTake( i2c2_lock, ( TickType_t ) portMAX_DELAY );

    I2C_start(I2C2, SlaveAddr, I2C_Direction_Transmitter);
    I2C_write(I2C2, RegisterAddr);
    I2C_write(I2C2, RegisterValue);
    I2C_stop(I2C2);
    
    xSemaphoreGive(i2c2_lock);
}

uint8_t I2C2_battery_read_register(uint8_t SlaveAddr,uint8_t RegisterAddr)
{
    uint8_t val;

    xSemaphoreTake( i2c2_lock, ( TickType_t ) portMAX_DELAY );
  
    I2C_start(I2C2, SlaveAddr, I2C_Direction_Transmitter);
    I2C_write(I2C2, RegisterAddr);
    
    /* Regenerate a start condition */
    I2C_start(I2C2, SlaveAddr, I2C_Direction_Receiver);
    val = I2C_read_nack(I2C2);
    
    xSemaphoreGive(i2c2_lock);
    
    return val;
}

uint8_t I2C2_battery_read_multi_register(uint8_t SlaveAddr,uint8_t RegisterAddr)
{
    uint8_t val1, val2;

    xSemaphoreTake( i2c2_lock, ( TickType_t ) portMAX_DELAY );
  
    I2C_start(I2C2, SlaveAddr, I2C_Direction_Transmitter);
    I2C_write(I2C2, RegisterAddr);
    
    /* Regenerate a start condition */
    I2C_start(I2C2, SlaveAddr, I2C_Direction_Receiver);
    val1 = I2C_read_ack(I2C2);
    val2 = I2C_read_nack(I2C2);
    
    xSemaphoreGive(i2c2_lock);
    
    return ((uint16_t) val1 | ((uint16_t)val2 << 8));
}

/**
  * @brief  Get Battery Voltage
  * @param  None
  * @retval Battery Voltage
  */   
static int bqGetVoltage(void)
{
    int val;  
    
    val = I2C2_battery_read_register(FUELGAUGE_SLAVE_ADDR, bq274xx_regs[BQ27XXX_REG_VOLT]); 

    return (val * 1000);
}

/**
  * @brief  Get Flag Status
  * @param  None
  * @retval Flag Status
  */
static uint16_t bqGetFlagsStatus(void)
{
    return I2C2_battery_read_register(FUELGAUGE_SLAVE_ADDR, bq274xx_regs[BQ27XXX_REG_FLAGS]);     		
}


/**
  * @brief  Get Average Current
  * @param  None
  * @retval Average Current
  */
static int bqGetAverageCurrent(void)
{
    int val;  
    
    val = I2C2_battery_read_register(FUELGAUGE_SLAVE_ADDR, bq274xx_regs[BQ27XXX_REG_AI]); 

    val = (int) ((int16_t) val) * 1000;
    
    return val;	
}

/**
  * @brief  Get State of Charge
  * @param  None
  * @retval State of Charge
  */
static int bqGetStateOfCharge(void)
{
    int val;  
    
    val = I2C2_battery_read_register(FUELGAUGE_SLAVE_ADDR, bq274xx_regs[BQ27XXX_REG_SOC]); 
    
    return val;	    		
}

/**
  * @brief  Writes Control Command 
  * @param  cmd command to write
  *         n byte number
  * @retval None
  */
static void control_cmd_wr(uint16_t cmd, uint8_t n)
{  
    uint8_t val;
    
    if(!n)
    { 
        val = (uint8_t)cmd;
        I2C2_battery_write_register(FUELGAUGE_SLAVE_ADDR, 0x00, val);      
    }    
    else
    {
        val = (uint8_t)(cmd >> 8);
        I2C2_battery_write_register(FUELGAUGE_SLAVE_ADDR, 0x01, val);      
    }
}

/**
  * @brief  Reads Control Status
  * @param  None
  * @retval control status
  */
static int control_cmd_read(void)
{
    int val;  
    
    val = I2C2_battery_read_register(FUELGAUGE_SLAVE_ADDR, bq274xx_regs[BQ27XXX_REG_CTRL]); 
    
    return val;	     
}

/**
  * @brief  Checks Fuel Gauge Sealed 
  * @param  None
  * @retval  -1 = Wait For Response , 0 or 1 Status
  */
static int sealed(void)
{
    static int sealState = 0;
    portTickType xbatteryLastWakeTime;
    
    // Take Current Ticks
    xbatteryLastWakeTime = xTaskGetTickCount();
  
    switch(sealState)
    {
        case 0:  
            control_cmd_wr(CONTROL_STATUS_SUBCMD, 0);
            
            // Sleep For 10 ms
            vTaskDelayUntil(&xbatteryLastWakeTime, 10);
            sealState = 1;
            break;  
        
        case 1:
            control_cmd_wr(CONTROL_STATUS_SUBCMD, 1);
            
            // Sleep For 10 ms
            vTaskDelayUntil(&xbatteryLastWakeTime, 10);
            sealState = 2;
        break;
        
        case 2:
            sealState = 0;
            if(control_cmd_read() & CONTROL_STATUS_SEALED_MASK)
              return 1;
            else
              return 0;
        break;  
        
    }
    
    return -1;
}

/**
  * @brief  unseals Fuel Gauge
  * @param  None
  * @retval -1 = Wait For Response , 0 or 1 Status
  */
static int unseal(void)
{
    static int SetunsealState = 0;  
    portTickType xbatteryLastWakeTime;
    
    // Take Current Ticks
    xbatteryLastWakeTime = xTaskGetTickCount();
    
    switch(SetunsealState)
    {
        case 0:    
            control_cmd_wr(BQ274XX_UNSEAL_KEY_0, 0);
                
            // Sleep For 10 ms
            vTaskDelayUntil(&xbatteryLastWakeTime, 10);
            SetunsealState = 1;
        break;  
        
        case 1:
            control_cmd_wr(BQ274XX_UNSEAL_KEY_0, 1);
                
            // Sleep For 10 ms
            vTaskDelayUntil(&xbatteryLastWakeTime, 10);
            SetunsealState = 2;
        break;
        
        case 2:
            control_cmd_wr(BQ274XX_UNSEAL_KEY_1, 0);
                
            // Sleep For 10 ms
            vTaskDelayUntil(&xbatteryLastWakeTime, 10);
            SetunsealState = 3;
        break;
        
        case 3:
            control_cmd_wr(BQ274XX_UNSEAL_KEY_1, 1);
                
            // Sleep For 10 ms
            vTaskDelayUntil(&xbatteryLastWakeTime, 10);
            SetunsealState = 4;
        break;
         
        case 4:
            SetunsealState = 0;
            return 0;      
        break;  
    }
  
    return -1;  
}

/**
  * @brief  Enter Config update mode
  * @param  None
  * @retval -1 = Wait For Response , 0 or 1 Status
  */
static int enter_cfg_update_mode(void)
{
    static int enterConfigModeState = 0;
    portTickType xbatteryLastWakeTime;
    
    // Take Current Ticks
    xbatteryLastWakeTime = xTaskGetTickCount();
    
    switch(enterConfigModeState)
    {
        case 0:    
            control_cmd_wr(SET_CFGUPDATE_SUBCMD, 0);
            
            // Sleep For 10 ms
            vTaskDelayUntil(&xbatteryLastWakeTime, 10);
            enterConfigModeState = 1;
        break;  
        
        case 1:
            control_cmd_wr(SET_CFGUPDATE_SUBCMD, 1);
            
            // Sleep For 1 second
            vTaskDelayUntil(&xbatteryLastWakeTime, 1000);
            enterConfigModeState = 2;
        break;
        
        case 2:
            enterConfigModeState = 0;
            if( bqGetFlagsStatus() & FLAGS_CFGUPMODE_MASK)
                return 1;
            else
                return 0;
        break;
    }  
    return -1;
}
  
/**
  * @brief  Exit Config update mode
  * @param  None
  * @retval  -1 = Wait For Response , 0 or 1 Status
  */
static int exit_cfg_update_mode(void)
{
    static int exitConfigModeState = 0;
    portTickType xbatteryLastWakeTime;
    
    // Take Current Ticks
    xbatteryLastWakeTime = xTaskGetTickCount();
    
    switch(exitConfigModeState)
    {
        case 0:    
            control_cmd_wr(EXIT_CFGUPDATE, 0);
                        
            // Sleep For 1 second
            vTaskDelayUntil(&xbatteryLastWakeTime, 10);
            exitConfigModeState = 1;
        break;  
        
        case 1:
            control_cmd_wr(EXIT_CFGUPDATE, 1);
                        
            // Sleep For 1 second
            vTaskDelayUntil(&xbatteryLastWakeTime, 1000);
            exitConfigModeState = 2;
        break;
        
        case 2:
            exitConfigModeState = 0;
            if( bqGetFlagsStatus() & FLAGS_CFGUPMODE_MASK)
                return 1;
            else
                return 0;
        break;
    }
  
    return -1;  
}

/**
  * @brief  Update Data memory Block
  * @param  subclass = subclass
  *         offset - offset in block
  *         len - length
  * @retval 1 = Success , 0 Failure
  */
static int update_dm_block(uint8_t subclass, uint8_t offset, uint8_t len, uint8_t *data)
{
    uint8_t rd_buf[4];
    uint8_t Old_cksum = 0, New_cksum = 0;
    uint8_t i = 0;
    uint8_t blk_offset = offset >> 5;
    uint8_t tempOld = 0, tempNew = 0, temp = 0;
    
    portTickType xbatteryLastWakeTime;
    
    // Take Current Ticks
    xbatteryLastWakeTime = xTaskGetTickCount();
    
        
    I2C2_battery_write_register(FUELGAUGE_SLAVE_ADDR, BLOCK_DATA_CONTROL, 0x00);   
         
    
    // Sleep For 10 ms
    vTaskDelayUntil(&xbatteryLastWakeTime, 10);

    I2C2_battery_write_register(FUELGAUGE_SLAVE_ADDR, BLOCK_DATA_CLASS, subclass);   
    
    // Sleep For 10 ms
    vTaskDelayUntil(&xbatteryLastWakeTime, 10);

    I2C2_battery_write_register(FUELGAUGE_SLAVE_ADDR, DATA_BLOCK, blk_offset);   
    
    // Sleep For 10 ms
    vTaskDelayUntil(&xbatteryLastWakeTime, 10);
	
    Old_cksum = I2C2_battery_read_register(FUELGAUGE_SLAVE_ADDR, BLOCK_DATA_CHECKSUM);
    
    // Sleep For 10 ms
    vTaskDelayUntil(&xbatteryLastWakeTime, 10);

    if( (len == 1) ||  (len == 2) ||  (len == 4) )
    {
        for(i=0; i < len; i++)
        {
            rd_buf[i] = I2C2_battery_read_register(FUELGAUGE_SLAVE_ADDR, (BLOCK_DATA + offset + i) );    

            // Sleep For 10 ms
            vTaskDelayUntil(&xbatteryLastWakeTime, 10);
        }
    }
    else
    {
        return 0;    
    }
        
    for(i=0; i < len; i++)
    {
        I2C2_battery_write_register(FUELGAUGE_SLAVE_ADDR, (BLOCK_DATA + offset + i) , data[len - i - 1] );   
            
        // Sleep For 10 ms
        vTaskDelayUntil(&xbatteryLastWakeTime, 10);
    }        
        
    for(i=0; i < len; i++)
    {
        tempOld +=  rd_buf[i]; 
        tempNew +=  data[i];
    }
        
    tempOld += Old_cksum;

    temp = (uint8_t) (((uint8_t)0xFF) - tempOld); 
    temp += tempNew;

    New_cksum = (uint8_t) (((uint8_t)0xFF) - temp); 

    I2C2_battery_write_register(FUELGAUGE_SLAVE_ADDR, BLOCK_DATA_CHECKSUM, New_cksum);

        
    // Sleep For 10 ms
    vTaskDelayUntil(&xbatteryLastWakeTime, 10);

    I2C2_battery_write_register(FUELGAUGE_SLAVE_ADDR, BLOCK_DATA_CONTROL, 0x00);   
    
    // Sleep For 10 ms
    vTaskDelayUntil(&xbatteryLastWakeTime, 10);
	        
    I2C2_battery_write_register(FUELGAUGE_SLAVE_ADDR, BLOCK_DATA_CLASS, subclass);   

    // Sleep For 10 ms
    vTaskDelayUntil(&xbatteryLastWakeTime, 10);

    I2C2_battery_write_register(FUELGAUGE_SLAVE_ADDR, DATA_BLOCK, blk_offset);         
    
    // Sleep For 10 ms
    vTaskDelayUntil(&xbatteryLastWakeTime, 10);
        
    for(i=0; i < len; i++)
    {
        rd_buf[len - i - 1] = I2C2_battery_read_register(FUELGAUGE_SLAVE_ADDR, (BLOCK_DATA + offset + i) );    
            
        // Sleep For 10 ms
        vTaskDelayUntil(&xbatteryLastWakeTime, 10);
    }          
         
    if (memcmp(data, rd_buf, len)) 
    {
        return 0;
    } 
    else 
    {
        return 1;
    }
}

/**
  * @brief  Read Data memory Block
  * @param  subclass - subclass
  *         offset - offset in block
  *         len - length
  * @retval parameter value being read
  */
static int read_dm_block(uint8_t subclass, uint8_t offset, uint8_t len)
{
    int value = 0;
    uint8_t blk_offset = offset >> 5; 
    uint8_t rd_buf;
    uint8_t i;
    
    portTickType xbatteryLastWakeTime;
    
    // Take Current Ticks
    xbatteryLastWakeTime = xTaskGetTickCount();
  
    I2C2_battery_write_register(FUELGAUGE_SLAVE_ADDR, BLOCK_DATA_CONTROL, 0x00);   
    
    // Sleep For 10 ms
    vTaskDelayUntil(&xbatteryLastWakeTime, 10);
	
    I2C2_battery_write_register(FUELGAUGE_SLAVE_ADDR, BLOCK_DATA_CLASS, subclass);   
    
    // Sleep For 10 ms
    vTaskDelayUntil(&xbatteryLastWakeTime, 10);
        
    I2C2_battery_write_register(FUELGAUGE_SLAVE_ADDR, DATA_BLOCK, blk_offset);   
    
    // Sleep For 10 ms
    vTaskDelayUntil(&xbatteryLastWakeTime, 10);
    
    if( (len == 1) ||  (len == 2) ||  (len == 4) )
    {
            for(i=0; i < len; i++)
            {
                rd_buf = I2C2_battery_read_register(FUELGAUGE_SLAVE_ADDR, (BLOCK_DATA + offset + i) );
                value |= ( (((int)rd_buf) & 0xFF) << (8 * (len - 1 - i)) );
                
                // Sleep For 10 ms
                vTaskDelayUntil(&xbatteryLastWakeTime, 10);
            }
     }
     else
     {
         return 0;    
     }
    
     return value;     
}

/**
  * @brief  Initialize Fuel Gauge Data Memory
  * @param  None
  * @retval None
  */
static void rom_mode_gauge_dm_init(void)
{
    int i;
#ifdef ENABLE_PRINT
    int val; 
#endif   
    
    for (i = 0; i < DM_REGS_COUNT; i++) 
    {        
        update_dm_block(bq274xx_dm_regs[i].subclass, bq274xx_dm_regs[i].offset, bq274xx_dm_regs[i].len, (uint8_t *)(&bq274xx_dm_regs[i].data) );       
    }

#ifdef ENABLE_PRINT
    for (i = 0; i < DM_REGS_COUNT; i++) 
    {
        val = read_dm_block(bq274xx_dm_regs[i].subclass, bq274xx_dm_regs[i].offset, bq274xx_dm_regs[i].len);
        sprintf(bat_log, "[%d] = %d\r\n", i, val);
        print(bat_log, strlen(bat_log)); 
    }
#endif
    
}

/**
  * @brief  Configure Battery
  * @param  None
  * @retval None
  */
void battery_configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;    
    NVIC_InitTypeDef NVIC_InitStructure;
    
    portTickType batteryLastWakeTime;

    RCC_AHB1PeriphClockCmd(BQ_BIN_GPIO_CLK | BQ_GPOUT_GPIO_CLK, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BQ_GPOUT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_Init(BQ_GPOUT_GPIO_PORT, &GPIO_InitStructure);

    SYSCFG_EXTILineConfig(BQ_GPOUT_EXTI_PORT_SOURCE, BQ_GPOUT_EXTI_PIN_SOURCE);

    EXTI_InitStructure.EXTI_Line = BQ_GPOUT_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  //Active Low 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    // Enable Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = BQ_GPOUT_EXTI_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = BATTERY_GPOUT_IRQ_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);
   
  
    while(1)
    {
        is_sealed = sealed();

        if( is_sealed >= 0)
        {
            if(is_sealed)
            {
#ifdef ENABLE_PRINT
                sprintf(bat_log, "Sealed\r\n");
                print( bat_log , strlen( bat_log));  
#endif
                break;
            } 
            else
            {
#ifdef ENABLE_PRINT
                sprintf(bat_log, "Unsealed\r\n");
                print( bat_log , strlen( bat_log));
#endif               
                break;
            }
        }
    }
    
    if(is_sealed)
    {
        // Take Current Ticks
        batteryLastWakeTime = xTaskGetTickCount();
        // Sleep For 500 ms
        vTaskDelayUntil(&batteryLastWakeTime, 500);  

        while( unseal() != 0)
        {
#ifdef ENABLE_PRINT          
            sprintf(bat_log, "Unsealing done, checking ...\r\n");
            print( bat_log , strlen( bat_log));  
#endif
            break;
        }
    }
    
    // Take Current Ticks
    batteryLastWakeTime = xTaskGetTickCount();
    // Sleep For 500 ms
    vTaskDelayUntil(&batteryLastWakeTime, 500);  
    
    while(1)
    {
        is_cfgupdate_mode = enter_cfg_update_mode();
        if( is_cfgupdate_mode >= 0)
        {
            if(!is_cfgupdate_mode)
            {
#ifdef ENABLE_PRINT
                sprintf(bat_log, "Not in CfgUpdate Mode\r\n");
                print( bat_log , strlen( bat_log));  
#endif
            }
            else
            { 
#ifdef ENABLE_PRINT
                sprintf(bat_log, "In CfgUpdate Mode\r\n");
                print( bat_log , strlen( bat_log)); 
#endif
                break;
            }
            
            // Take Current Ticks
            batteryLastWakeTime = xTaskGetTickCount();
            // Sleep For 1000 ms
            vTaskDelayUntil(&batteryLastWakeTime, 1000);      
        }
    }     
    
    rom_mode_gauge_dm_init(); 
    
    for(int i = 0; i < 0xFFFF; i++);
   
    SET_BAT_INSERTED();

    // Take Current Ticks
    batteryLastWakeTime = xTaskGetTickCount();
    // Sleep For 1000 ms
    vTaskDelayUntil(&batteryLastWakeTime, 1000);           
    
    while(1)
    {
        is_cfgupdate_mode = exit_cfg_update_mode();
        if( is_cfgupdate_mode >= 0)
        {
            if(!is_cfgupdate_mode)
            {
#ifdef ENABLE_PRINT
                sprintf(bat_log, "Out of CfgUpdate Mode\r\n");
                print( bat_log , strlen( bat_log));  
#endif
                break;
            }
            else
            {
#ifdef ENABLE_PRINT
                sprintf(bat_log, "In CfgUpdate Mode\r\n");
                print( bat_log , strlen( bat_log)); 
#endif
            }
            
            // Take Current Ticks
            batteryLastWakeTime = xTaskGetTickCount();
            // Sleep For 1000 ms
            vTaskDelayUntil(&batteryLastWakeTime, 1000);    
        }
    }    
}// End of Battery Configuration


//from external world main entry point for all query
/**
  * @brief  Battery State Machine
  * @param  Command for Battery State Machine   
  * @retval None
  */
void battery_statemachine(uint8_t BatCmd)
{
    switch(BatCmd)
    {
        case BATTERY_IDLE:  
        break;  
        
        case BATTERY_QUERY_STATUS:
        if(bqGetFlagsStatus() & FLAGS_DISCHARGING_MASK)
        {
            batteryStatus = BAT_STATUS_DISCHARGING;
#ifdef ENABLE_PRINT            
              sprintf(bat_log, "Discharging\r\n");
              print( bat_log , strlen( bat_log)); //uart print wrapper 
#endif
        }
        else
        {
            batteryStatus = BAT_STATUS_CHARGING;
#ifdef ENABLE_PRINT
              sprintf(bat_log, "Charging\r\n");
              print( bat_log , strlen( bat_log));
#endif
        }
        break;
        
        case BATTERY_QUERY_SOC:
        batteryPercentage = bqGetStateOfCharge();
        if(batteryPercentage > 100)
        {
            batteryPercentage = 100;          
        }
#ifdef ENABLE_PRINT
          sprintf(bat_log, "Capacity = %hhu%\r\n",batteryPercentage);
          print( bat_log , strlen( bat_log));
#endif
        break;  
          
        case BATTERY_QUERY_VOLTAGE:
        value = bqGetVoltage();  
#ifdef ENABLE_PRINT
          sprintf(bat_log, "Voltage = %d uV\r\n",value);
          print( bat_log , strlen( bat_log));
#endif
        break;
        
        case BATTERY_QUERY_CURRENT:
        value = bqGetAverageCurrent();  
#ifdef ENABLE_PRINT
          sprintf(bat_log, "Current = %d uA\r\n",value);
          print( bat_log , strlen( bat_log));
#endif
        break;
        
        case BATTERY_BUS_FREE_WAITING_TIME:
        break;            
    }
  
}

/**
  * @brief  Get Battery Percentage Remaining
  * @param  None
  * @retval Battery Percentage Remaining
  */
uint8_t getBatteryPercentage(void)
{
    return batteryPercentage;     
}

/**
  * @brief  Get Battery Charging/Discharging Status
  * @param  None
  * @retval Battery Charging/Discharging Status
  */
uint8_t getBatteryStatus(void)
{
    if(batteryStatus)
    {
        /* Battery Charging = BAT_STATUS_CHARGING = 1 */  
        return BAT_CHARGING;
    }
    else
    {
        uint8_t b;
        /* Battery Discharging = BAT_STATUS_DISCHARGING  = 0 */
        b = (batteryPercentage / 20);
        if(b < 5)
        {
            /* See Macros : BAT_LEVEL_0 - BAT_LEVEL_4 */
            return b;  
        }
        else
        {
            /* Battery Percentage 100% */
            return BAT_LEVEL_3;  
        }
    }
}

/**
  * @brief  Get Battery Charging Or Discharging indication
  * @param  None
  * @retval Battery Charging Or Discharging
  */
uint8_t IsBatteryCharging(void)
{
   if(batteryStatus)
    {
        /* Battery Charging = BAT_STATUS_CHARGING = 1 */  
        return BAT_STATUS_CHARGING;
    }
    else
    {
        return BAT_STATUS_DISCHARGING;
    }
}

/**
  * @brief  Is Battery Status changed or not 
  * @param  None
  * @retval Battery Status changed or not 
  */
uint8_t IsBatteryStatusChanged(void)
{
    if(batteryPercentage != prev_batteryPercentage)
    {
        prev_batteryPercentage = batteryPercentage;
        prev_batteryStatus = batteryStatus;
        return 1;
    }
    else if( batteryStatus != prev_batteryStatus )
    {
        prev_batteryPercentage = batteryPercentage;
        prev_batteryStatus = batteryStatus;
        return 1;  
    }
    else
    {
        return 0;
    }
}

//addded by RUTVIJ 
// Quick check, not for production
//In boot mode check BMS diagnostic and raise signal
#ifdef BAT_DIAG_ENABLE 
void battery_diagnostic_test(void)
{
    uint8_t batTemp1, batTemp2;
    
	// As per all the test formates
    print("========================= BATTERY FUEL GAUGE TEST CASE ================================\r\n");
    print("Test case: Check Whether Battery is in Charging or not Charging..\r\n");
    print("Test case: Checking By Inserting/Removing USB Cable ..\r\n\r\n");
             
    battery_statemachine(BATTERY_QUERY_SOC);
    
    battery_statemachine(BATTERY_QUERY_STATUS);
    
    for(int i = 0; i < 0xFFFF; i++); // baremetal - non-rtos context, HAL_Delay ();
    
    batTemp1 = getBatteryPercentage(); //TODO - delay
    
    for(int i = 0; i < 0xFFFF; i++); //TODO - delay
    
    batTemp2 = getBatteryStatus();
    if(batTemp2 == BAT_CHARGING)
    {
        print("\t\tBattery in Charging..\r\n"); 
    }
    else
    {
        print("\t\tBattery in Discharging..\r\n");
    }
    
    print("\r\n========================= BATTERY FUEL GAUGE TEST CASE ================================\r\n");
    
}
#endif
