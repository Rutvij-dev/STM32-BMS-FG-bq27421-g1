#ifndef _BATTERY_MONITOR_SYS_H_
#define _BATTERY_MONITOR_SYS_H_

#include "stm32f4xx.h"


#define FUELGAUGE_SLAVE_ADDR		((uint8_t)0xAA)  /* I2C address of fuel gauge bq27421-g1 IC */

/* Battery Inserted Signal Indication Pin */
#define BQ_BIN_PIN                      GPIO_Pin_14
#define BQ_BIN_GPIO_PORT                GPIOB
#define BQ_BIN_GPIO_CLK                 RCC_AHB1Periph_GPIOB

/*  BQ_GPOUT  Pin */
#define BQ_GPOUT_PIN                    GPIO_Pin_15
#define BQ_GPOUT_GPIO_PORT              GPIOE
#define BQ_GPOUT_GPIO_CLK               RCC_AHB1Periph_GPIOE

#define BQ_GPOUT_EXTI_LINE              EXTI_Line15
#define BQ_GPOUT_EXTI_PORT_SOURCE       EXTI_PortSourceGPIOE
#define BQ_GPOUT_EXTI_PIN_SOURCE        EXTI_PinSource15
#define BQ_GPOUT_EXTI_IRQn              EXTI15_10_IRQn 


#define SET_BAT_INSERTED()              GPIO_ResetBits(BQ_BIN_GPIO_PORT, BQ_BIN_PIN)
#define SET_BAT_REMOVED()               GPIO_SetBits(BQ_BIN_GPIO_PORT, BQ_BIN_PIN)
/**
  *	Commmands for interfacing With fuel gauge bq27421-g1 IC
  *
  **/
/* Subcommands of Control() */
#define CONTROL_STATUS_SUBCMD		((uint16_t)0x0000)
#define DEV_TYPE_SUBCMD			((uint16_t)0x0001)
#define FW_VER_SUBCMD			((uint16_t)0x0002)
#define DF_VER_SUBCMD			((uint16_t)0x001F)
#define RESET_SUBCMD			((uint16_t)0x0041)
#define SET_CFGUPDATE_SUBCMD		((uint16_t)0x0013)
#define SEAL_SUBCMD			((uint16_t)0x0020)
#define EXIT_CFGUPDATE		        ((uint16_t)0x0043)      
#define SET_BAT_INSERT                  ((uint16_t)0x000C)  
#define SET_BAT_REMOVE                  ((uint16_t)0x000D)  
    
#define BLOCK_DATA_CLASS		0x3E
#define DATA_BLOCK			0x3F
#define BLOCK_DATA			0x40
#define BLOCK_DATA_CHECKSUM		0x60
#define BLOCK_DATA_CONTROL		0x61    
    
    
#define BQ274XX_UNSEAL_KEY_0		((uint16_t)0x8000)    
#define BQ274XX_UNSEAL_KEY_1		((uint16_t)0x8000)    

    
#define CONTROL_STATUS_SEALED_MASK     ((uint16_t) 0x2000)   

#define FLAGS_CFGUPMODE_MASK           ((uint16_t) 0x0010)      
#define FLAGS_BAT_DET_MASK             ((uint16_t) 0x0008)         
#define FLAGS_DISCHARGING_MASK         ((uint16_t) 0x0001)         
    
#define  BAT_STATUS_DISCHARGING        ((uint8_t) 0x00) 
#define  BAT_STATUS_CHARGING           ((uint8_t) 0x01)  
    
#define  BATTERY_IDLE                          ((uint8_t) 0x00)
#define  BATTERY_QUERY_STATUS                  ((uint8_t) 0x01)
#define  BATTERY_QUERY_SOC                     ((uint8_t) 0x02)
#define  BATTERY_QUERY_VOLTAGE                 ((uint8_t) 0x03)
#define  BATTERY_QUERY_CURRENT                 ((uint8_t) 0x04)
#define  BATTERY_QUERY_SEALED                  ((uint8_t) 0x05)
#define  BATTERY_WAIT_QUERY_SEALED_RESPONSE    ((uint8_t) 0x06)
#define  BATTERY_SEAL                          ((uint8_t) 0x07)
#define  BATTERY_UNSEAL                        ((uint8_t) 0x08)
#define  BATTERY_ENTER_CONFIG_UPDATE_MODE      ((uint8_t) 0x09)
#define  BATTERY_EXIT_CONFIG_UPDATE_MODE       ((uint8_t) 0x0A)    
#define  BATTERY_CONTROL_SUBCMD1               ((uint8_t) 0x0B)
#define  BATTERY_CONTROL_SUBCMD2               ((uint8_t) 0x0C)
#define  BATTERY_CONTROL_READ                  ((uint8_t) 0x0D)
#define  BATTERY_QUERY                         ((uint8_t) 0x0E)
#define  BATTERY_BUS_FREE_WAITING_TIME         ((uint8_t) 0x0F)
#define  BATTERY_LEARNING_DEBUG                ((uint8_t) 0x10)
    
/* CHKPOINT1(10%) > CHKPOINT2(5%) > CHKPOINT3(2%) > CHKPOINT4(1%) */
// HOLD the boot if bat <= CHKPOINT2
#define BAT_LOW_CHKPOINT1                      ((uint8_t)10)   
#define BAT_LOW_CHKPOINT2                      ((uint8_t)5) 
#define BAT_LOW_CHKPOINT3                      ((uint8_t)2)  
#define BAT_LOW_CHKPOINT4                      ((uint8_t)1) 

enum
{
    BAT_IDLE = 0,
    BAT_QUERY_STATUS,
    BAT_QUERY_SOC,
    BAT_CURRENT
};

extern uint8_t batteryPercentage, batteryStatus;
extern volatile uint8_t BatteryStatus_Changed;

// Battery Status for Battery Task
extern uint8_t bat_status;

void battery_configure(void);


void battery_statemachine(uint8_t BatCmd);

uint8_t getBatteryStatus(void);
uint8_t getBatteryPercentage(void);
uint8_t IsBatteryCharging(void);
uint8_t IsBatteryStatusChanged(void);

void battery_diagnostic_test(void);

#endif //_BATTERY_MONITOR_SYS_H_



