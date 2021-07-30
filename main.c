#include "stm8l15x.h"

/*!<Integer types
signed char     int8_t    s8;
signed short    int16_t   s16;
signed long     int32_t   s32;  
unsigned char   uint8_t   u8;
unsigned short  uint16_t  u16;
unsigned long   uint32_t  u32;*/

//Pullups on PC1 and PC2
//Desolder jumper SB17 as it places 100nf on PC1


void delay_ms(unsigned short n_ms);
void I2C_setup(void);
void I2C_write1(u8 I2C_Slave_Adress, u8 b1);
void I2C_write2(u8 I2C_Slave_Adress, u8 b1, u8 b2);
s16 I2C_read2(u8 I2C_Slave_Adress, u8 reg);
void s16_to_str( s16 value, char* buffer );


void main(void)
{
  GPIO_DeInit(GPIOC);
  GPIO_DeInit(GPIOE);
  /* Initialize LED mounted on STM8L152-Discovery */
  GPIO_Init(GPIOC, GPIO_Pin_7, GPIO_Mode_Out_PP_Low_Fast);
  GPIO_Init(GPIOE, GPIO_Pin_7, GPIO_Mode_Out_PP_Low_Fast);
  /* Initialize LCD mounted on STM8L152-Discovery */
  LCD_GLASS_Init();
  char data_str[] = "Foob";  //string to hold the data for the LCD functions

  /* Initialize I2C */
  I2C_setup();
  u8 mpu = (0x68) << 1;  //shift the address on one bit to work with I2C_Send7bitAddress
  s16 data = 100;        //variable to hold sensor data, initial value is arbitrary
  
  //Sensor specific initializacion procedure
  //Here presented for the MPU6050
  //Write to the PWR_MGMT_1 register (6B hex) Reset the register bits to activate MPU.
  I2C_write2(mpu, 0x6B, 0x00);
  //Write to the ACCEL_CONFIG register (1A hex) Set the register bits as 00001000 (+/- 4g full scale range)
  I2C_write2(mpu, 0x1C, 0x08);
  //Write to the CONFIG register (1A hex) Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  I2C_write2(mpu, 0x1A, 0x03);
      
  while (1)
  {    
    data=I2C_read2(mpu, 0x3B);    //Read 2 bytes from the 0x3B register (x accelerometer)
    s16_to_str( data, data_str ); //store data as a string for LCD functions
  
    GPIO_ToggleBits(GPIOC, GPIO_Pin_7);//toggle LED on C7
    GPIO_WriteBit(GPIOE, GPIO_Pin_7,(BitAction) 0);//toggle LED on E7 using different approach
    delay_ms(500);
    
    GPIO_ToggleBits(GPIOC, GPIO_Pin_7);//toggle LED on C7
    GPIO_WriteBit(GPIOE, GPIO_Pin_7,(BitAction) 1);//toggle LED on E7 using different approach
    delay_ms(500);
    
    LCD_GLASS_Clear();                 //Clear LCD display
    LCD_GLASS_DisplayString(data_str); //Display data to LCD    
   }
}


/**
  * @brief  waits for the specified number of miliseconds.
  * @param  n_ms : time in ms.
  * @retval none.
  */
void delay_ms(u16 n_ms)
{
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, ENABLE);//Init TIMER 4
  TIM4->PSCR = 3;//prescaler: / (2^3) = /8
  TIM4->ARR = 250;//HSI div by 1 --> Auto-Reload value: 2M / 8 = 1/4M, 1/4M / 1k = 250  
  TIM4->CNTR = 2;//Counter value: 2, to compensate the initialization of TIMER
  TIM4->SR1 &= ~TIM4_SR1_UIF;//clear update flag
  TIM4->CR1 |= TIM4_CR1_CEN;//Enable Counter

  while(n_ms--)//wait until time match
  {
    while((TIM4->SR1 & TIM4_SR1_UIF) == 0);
    TIM4->SR1 &= ~TIM4_SR1_UIF;
  }

/* Disable Counter */
  TIM4->CR1 &= ~TIM4_CR1_CEN;
}

/**
  * @brief  initial setup of the I2C interface.
  * @param  none.
  * @retval none.
  */
void I2C_setup(void)
{  
  //Define outputs as Open Drain, this happens automaticly within I2C_Init
  //but I kept the explicit declarations so it wont go overlooked
   GPIO_Init(GPIOC, GPIO_Pin_0, GPIO_Mode_Out_OD_HiZ_Fast);
   GPIO_Init(GPIOC, GPIO_Pin_1, GPIO_Mode_Out_OD_HiZ_Fast);  
   //Start the I2C clock
   CLK_PeripheralClockConfig(CLK_Peripheral_I2C1,ENABLE);
   //Initialize the I2C interface at 100KHz with its own adress as 0xA0
   I2C_DeInit(I2C1); 
   I2C_Init(I2C1,100000, 0xA0, I2C_Mode_I2C, I2C_DutyCycle_2,
            I2C_Ack_Enable, I2C_AcknowledgedAddress_7bit); 
   I2C_Cmd(I2C1,ENABLE);
}


/**
  * @brief  write a single byte to slave device.
  * @param  I2C_Slave_Adress: Adress of the slave.
  * @param  b1: byte to write.
  * @retval none.
  */
void I2C_write1(u8 I2C_Slave_Adress, u8 b1)
{   
   I2C_GenerateSTART(I2C1,ENABLE); //Start I2C communication  
   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));//Check corresponding event (EV5)
      
   I2C_Send7bitAddress( I2C1, I2C_Slave_Adress, I2C_Direction_Transmitter);//Identify with the slave as master transmitter 
   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//Check corresponding event (EV6) 
   
   I2C_SendData(I2C1,b1); //Send one byte of data
   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));//Check corresponding event (EV8_2)
   
   I2C_GenerateSTOP(I2C1, ENABLE); //Stop  
}


/**
  * @brief  write 2 bytes to slave device.
  * @param  I2C_Slave_Adress: Adress of the slave.
  * @param  b1: first byte to write.
  * @param  b2: second byte to write.
  * @retval none.
  */
void I2C_write2(u8 I2C_Slave_Adress, u8 b1, u8 b2)
{   
   I2C_GenerateSTART(I2C1,ENABLE); //Start I2C communication  
   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));//Check corresponding event (EV5)
      
   I2C_Send7bitAddress( I2C1, I2C_Slave_Adress, I2C_Direction_Transmitter);//Identify with the slave as master transmitter 
   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//Check corresponding event (EV6) 
   
   I2C_SendData(I2C1,b1); //Send fisrt byte of data
   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));//Check corresponding event (EV8_2)
   
   I2C_SendData(I2C1,b2); //Send second byte of data
   while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));//Check corresponding event (EV8_2)
   
   I2C_GenerateSTOP(I2C1, ENABLE); //Stop   
}

/**
  * @brief  Read 2 bytes from slave device.
  * @param  I2C_Slave_Adress: Adress of the slave.
  * @param  reg: target register to read.
  * @retval single 16-bit signed integer with both bytes of data.
  */
s16 I2C_read2(u8 I2C_Slave_Adress, u8 reg)
{
   s16 RegValue = 0;
 
   // Enable I2C1 acknowledgement if it is already disabled by other function
   I2C_AcknowledgeConfig(I2C1, ENABLE);
 
   //---------------------------------- Transmission Phase ----------------
   I2C_GenerateSTART(I2C1, ENABLE);//Start
   while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));//Check corresponding event (EV5)

   I2C_Send7bitAddress(I2C1, I2C_Slave_Adress, I2C_Direction_Transmitter);//Identify with the slave as master transmitter  
   while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//Check corresponding event (EV6)
 
   I2C_SendData(I2C1, reg);//Send register direction to start read operation 
   while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));//Check corresponding event (EV8_2)
   
   /*-------------------------------- Reception Phase --------------------------*/
   I2C_GenerateSTART(I2C1, ENABLE);//Send Re-START condition 
   while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));//Check corresponding event (EV5)
 
   I2C_Send7bitAddress(I2C1, I2C_Slave_Adress, I2C_Direction_Receiver);//Identify with the slave as master receiver  
   while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));//Check corresponding event (EV6)

   while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));//Check corresponding event (EV7)
   RegValue = I2C_ReceiveData(I2C1) << 8;//Store first byte on the MSB portion
   
   while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));//Check corresponding event (EV7)
   RegValue |= I2C_ReceiveData(I2C1);//Store second byte on the MSB portion
   
   I2C_AcknowledgeConfig(I2C1, DISABLE);//Disable I2C1 acknowledgement
   I2C_GenerateSTOP(I2C1, ENABLE);//Stop (second byte already on input register) 
 
   return RegValue;//return value
}


/**
  * @brief  Transform signed 16bit integer to string.
  * @param  value: integer to transform.
  * @param  buffer: pointer to start of char array to store value.
  * @retval none.
  */
void s16_to_str( s16 value, char* buffer )
{
  unsigned char p=0;
  if ( value == 0 )
  {
    buffer[0]='0';
    buffer[1]='\0';
    return;
  }
  else if ( value < 0 )
  {
    buffer[p++] = '-';
    value = (value<0)?-value:value;
  }
  for ( int s=10000; s>0; s/=10 )
  {
    //p++
    if ( value >= s )
    {
      buffer[p++] = (unsigned char)(value/s) + '0';
      value %= s;
    }
    else if ( p>0 )
    {
      buffer[p++] = '0';
    }
  }
  buffer[p] = '\0';
}
