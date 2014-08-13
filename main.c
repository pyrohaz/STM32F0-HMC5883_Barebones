#include <stm32f0xx_gpio.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_i2c.h>

//HMC5883 declarations
#define HMC_SDA GPIO_Pin_11
#define HMC_SCL GPIO_Pin_10
#define HMC_GPIO GPIOB

#define HMC_SDA_PS GPIO_PinSource11
#define HMC_SCL_PS GPIO_PinSource10
#define HMC_PIN_AF GPIO_AF_1

#define HMC_I2C I2C2

//HMC5883 I2C address, don't ask my about the shift
//as it doesn't work without it!
#define HMCAddr (0x1E<<1)

//HMC5883 internal registers
#define R_Config1 0x00
#define R_Config2 0x01
#define R_Mode 0x02
#define R_Status 0x09
#define R_XRegister 0x03

//Time keeping variable!
volatile uint32_t MSec;

GPIO_InitTypeDef GP;
I2C_InitTypeDef IT;

void I2C_WrReg(uint8_t Reg, uint8_t Val){

	//Wait until I2C isn't busy
	while(I2C_GetFlagStatus(HMC_I2C, I2C_FLAG_BUSY) == SET);

	//"Handle" a transfer - The STM32F0 series has a shocking I2C interface...
	//...Regardless! Send the address of the HMC sensor down the I2C Bus and generate
	//a start saying we're going to write one byte. I'll be completely honest,
	//the I2C peripheral doesn't make too much sense to me and a lot of the code is
	//from the Std peripheral library
	I2C_TransferHandling(HMC_I2C, HMCAddr, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);

	//Ensure the transmit interrupted flag is set
	while(I2C_GetFlagStatus(HMC_I2C, I2C_FLAG_TXIS) == RESET);

	//Send the address of the register we wish to write to
	I2C_SendData(HMC_I2C, Reg);

	//Ensure that the transfer complete reload flag is Set, essentially a standard
	//TC flag
	while(I2C_GetFlagStatus(HMC_I2C, I2C_FLAG_TCR) == RESET);

	//Now that the HMC5883L knows which register we want to write to, send the address
	//again and ensure the I2C peripheral doesn't add any start or stop conditions
	I2C_TransferHandling(HMC_I2C, HMCAddr, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);

	//Again, wait until the transmit interrupted flag is set
	while(I2C_GetFlagStatus(HMC_I2C, I2C_FLAG_TXIS) == RESET);

	//Send the value you wish you write to the register
	I2C_SendData(HMC_I2C, Val);

	//Wait for the stop flag to be set indicating a stop condition has been sent
	while(I2C_GetFlagStatus(HMC_I2C, I2C_FLAG_STOPF) == RESET);

	//Clear the stop flag for the next potential transfer
	I2C_ClearFlag(HMC_I2C, I2C_FLAG_STOPF);
}


//We're really fortunate with the HMC5883L as it automatically increments the internal register
//counter with every read so we need to set the internal register pointer to the first data
//register (the X value register) and just read the next 6 piece of data, X1, X2, Z1, Z2
//Y1, Y2 and voila! We have the compass values!
uint8_t I2C_RdReg(int8_t Reg, int8_t *Data, uint8_t DCnt){
	int8_t Cnt, SingleData = 0;

	//As per, ensure the I2C peripheral isn't busy!
	while(I2C_GetFlagStatus(HMC_I2C, I2C_FLAG_BUSY) == SET);

	//Again, start another tranfer using the "transfer handling" function, the end bit
	//being set in software this time round, generate a start condition and indicate
	//you will be writing data to the HMC device.
	I2C_TransferHandling(HMC_I2C, HMCAddr, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

	//Wait until the transmit interrupt status is set
	while(I2C_GetFlagStatus(HMC_I2C, I2C_FLAG_TXIS) == RESET);

	//Send the address of the register you wish to read
	I2C_SendData(HMC_I2C, (uint8_t)Reg);

	//Wait until transfer is complete!
	while(I2C_GetFlagStatus(HMC_I2C, I2C_FLAG_TC) == RESET);

	//As per, start another transfer, we want to read DCnt amount of bytes. Generate
	//a start condition and indicate that we want to read.
	I2C_TransferHandling(HMC_I2C, HMCAddr, DCnt, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

	//Read in DCnt pieces of data
	for(Cnt = 0; Cnt<DCnt; Cnt++){
		//Wait until the RX register is full of luscious data!
		while(I2C_GetFlagStatus(HMC_I2C, I2C_FLAG_RXNE) == RESET);

		//If we're only reading one byte, place that data direct into the
		//SingleData variable. If we're reading more than 1 piece of data
		//store in the array "Data" (a pointer from main)
		if(DCnt > 1) Data[Cnt] = I2C_ReceiveData(HMC_I2C);
		else SingleData = I2C_ReceiveData(HMC_I2C);
	}

	//Wait for the stop condition to be sent
	while(I2C_GetFlagStatus(HMC_I2C, I2C_FLAG_STOPF) == RESET);

	//Clear the stop flag for next transfers
	I2C_ClearFlag(HMC_I2C, I2C_FLAG_STOPF);

	//Return a single piece of data if DCnt was less than 1, otherwise 0 will be returned.
	return SingleData;
}

//Standard systick interrupt handler incrementing a variable named
//MSec (Milliseconds)
void SysTick_Handler(void){
	MSec++;
}

//Standard delay function as described in one of my previous tutorials!
//All it does is operate a nop instruction until "Time" amount of
//milliseconds has passed.
void Delay(uint32_t Time){
	volatile uint32_t MSStart = MSec;
	while((MSec-MSStart)<Time) asm volatile("nop");
}

//The main!
int main(void)
{
	//Setup a systick interrupt every 1ms (1/1000 seconds)
	SysTick_Config(SystemCoreClock/1000);
	//Enable GPIOB clock, required for the I2C output
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	//Enable the I2C peripheral clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

	//Set the pins HMC_SDA and HMC_SCL as alternate function GPIO pins
	GP.GPIO_Pin = HMC_SDA | HMC_SCL;
	GP.GPIO_Mode = GPIO_Mode_AF;
	GP.GPIO_OType = GPIO_OType_OD;
	GP.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_Init(HMC_GPIO, &GP);

	//Configure the pins to the I2C AF
	GPIO_PinAFConfig(HMC_GPIO, HMC_SDA_PS, HMC_PIN_AF);
	GPIO_PinAFConfig(HMC_GPIO, HMC_SCL_PS, HMC_PIN_AF);

	//Setup the I2C struct. The timing variable is acquired from the
	//STM32F0 I2C timing calculator sheet. Pretty standard stuff really,
	//its using the Analog filter to clean up any noisy edges (not really
	//required though if you wish to disable it, you will need a different
	//I2C_Timing value).
	IT.I2C_Ack = I2C_Ack_Enable;
	IT.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
	IT.I2C_DigitalFilter = 0;
	IT.I2C_Mode = I2C_Mode_I2C;
	IT.I2C_OwnAddress1 = 0xAB;
	IT.I2C_Timing = 0x10805E89;
	IT.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(HMC_I2C, &IT);
	I2C_Cmd(HMC_I2C, ENABLE);

	//Write the configuration registers for the HMC5883L compass.
	const uint8_t SmplAverage = 0b11; //Average 8 samples
	const uint8_t OutputRate = 0b110; //Output at 75Hz (Why not!)
	const uint8_t MeasurementCfg = 0b00; //We don't want any bias on the sensors
	const uint8_t SnsrGain = 0b100; //A gain of 440LSb/Gauss, standard.
	const uint8_t HighSpeedI2C = 0; //We want standard speed I2C.
	const uint8_t SnsrMode = 0b00; //Continuous measurement mode

	//Config 1 register
	I2C_WrReg(R_Config1, (SmplAverage<<5)|(OutputRate<<3)|MeasurementCfg);
	//Config 2 register
	I2C_WrReg(R_Config2, (SnsrGain<<5));
	//Mode register
	I2C_WrReg(R_Mode, (HighSpeedI2C<<7)|SnsrMode);

	//Give the sensor a breather before reading data!
	Delay(10);

	int8_t RecData[6] = {0,0,0,0,0,0};
	int32_t XVal = 0, YVal = 0, ZVal = 0;

	while(1)
	{
		//Read status register, waiting for the data ready flag
		while(!(I2C_RdReg(R_Status, 0, 1) & 1));

		//Receive 6 bytes of data from the value registers and store in
		//RecData variable
		I2C_RdReg(R_XRegister, &RecData[0], 6);

		//Construct the words containing the value data!
		//The HMC5883 is a little odd as it arranges its registers
		//in the order X,Z,Y as opposed to what I suppose you'd
		//expect from the normal order X,Y,Z.
		XVal = (RecData[0]<<8) | RecData[1];
		ZVal = (RecData[2]<<8) | RecData[3];
		YVal = (RecData[4]<<8) | RecData[5];
	}
}
