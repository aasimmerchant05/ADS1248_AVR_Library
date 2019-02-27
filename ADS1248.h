#ifndef ADS1248
#define ADS1248

/*
 * @file ADS1248.h
 *
 *@author Written by Mohammed Asim Merchant
 *		 Created on 27/02/2019
 *
 *@Note	PIN CONFIGURATION
 *		ADS1248 DIN		---		SPI-MOSI = PINB2
 *		ADS1248 DOUT	---		SPI-MISO = PINB3
 *		ADS1248 SCK		---		SPI-MOSI = PINB1
 *		ADS1248 CS		---		any I/O = PINB0
 *		ADS1248 DRDY	---		any I/O = PINE7
 *		ADS1248 START	---		any I/O = PINE6
 *		ADS1248 RESET	---		any I/O = PINB4
 *
 * @License MIT License
 *
 * Copyright (c) 2019 Mohammed Asim Merchant
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.

 *
 */

#include <avr/io.h>
#include <util/delay.h>
#include "SPILib.h"

//////////////////////////////////////////////////////////////////////////		MACROS		//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////	PINMAP

#define ADS1248_RESET_DDR						DDRB					/**< ADS1248 RESET Pin Data Direction Register **/
#define ADS1248_RESET_PORT						PORTB					/**< ADS1248 RESET Pin PORT Register **/
#define ADS1248_RESET							4						/**< ADS1248 RESET Pin No **/ 

#define ADS1248_START_DDR						DDRE					/**< ADS1248 START Pin Data Direction Register **/
#define ADS1248_START_PORT						PORTE					/**< ADS1248 START Pin PORT Register **/
#define ADS1248_START							6						/**< ADS1248 START Pin No **/

#define ADS1248_DRDY_DDR						DDRE					/**< ADS1248 DRDY Pin Data Direction Register **/
#define ADS1248_DRDY_PORT						PORTE					/**< ADS1248 DRDY Pin PORT Register **/
#define ADS1248_DRDY_PINP						PINE					/**< ADS1248 DRDY Pin PIN Register **/
#define ADS1248_DRDY							7						/**< ADS1248 DRDY Pin No **/

#define ADS1248_SS_DDR							SPI_PORT_DDR			/**< ADS1248 SS(Slave Select) Pin Data Direction Register **/
#define ADS1248_SS_PORT							SPI_PORT				/**< ADS1248 SS(Slave Select) Pin PORT Register **/
#define ADS1248_SS								SPI_SS					/**< ADS1248 SS(Slave Select) Pin No **/

// Commands
#define ADS1248_CMD_WAKE						0x00					/** ADS1248 COMMAND : Exit power-down mode **/
#define ADS1248_CMD_SLEEP     					0x02					/** ADS1248 COMMAND : Enter power-down mode **/
#define ADS1248_CMD_SYNC      					0x04					/** ADS1248 COMMAND : Synchronize ADC conversions	//has second command byte also, check datasheet **/
#define ADS1248_CMD_RESET     					0x06					/** ADS1248 COMMAND : Reset to default values **/
#define ADS1248_CMD_RDATA     					0x12					/** ADS1248 COMMAND : Read data once **/
#define ADS1248_CMD_RDATAC    					0x14					/** ADS1248 COMMAND : Read data continuous mode **/
#define ADS1248_CMD_SDATAC    					0x16					/** ADS1248 COMMAND : Stop data read continuous mode **/
#define ADS1248_CMD_RREG      					0x20					/** ADS1248 COMMAND : Read register  **/
#define ADS1248_CMD_WREG      					0x40					/** ADS1248 COMMAND : Write register **/
#define ADS1248_CMD_SYSOCAL   					0x60					/** ADS1248 COMMAND : System offset calibration **/
#define ADS1248_CMD_SYSGCAL   					0x61					/** ADS1248 COMMAND : System gain calibration **/
#define ADS1248_CMD_SELFOCAL  					0x62					/** ADS1248 COMMAND : Self offset calibration **/
#define ADS1248_CMD_NOP	      					0xFF					/** ADS1248 COMMAND : No operation **/

// Registers
#define ADS1248_MUX0          					0x00					/** ADS1248 REGISTER : Multiplexer Control Register 0 **/
#define ADS1248_VBIAS         					0x01					/** ADS1248 REGISTER : Bias Voltage Register **/
#define ADS1248_MUX1          					0x02					/** ADS1248 REGISTER : Multiplexer Control Register 1 **/
#define ADS1248_SYS0          					0x03					/** ADS1248 REGISTER : System Control Register 0 **/
#define ADS1248_OFC0          					0x04					/** ADS1248 REGISTER : Offset Calibration Coefficient Register 0 **/
#define ADS1248_OFC1          					0x05					/** ADS1248 REGISTER : Offset Calibration Coefficient Register 1 **/
#define ADS1248_OFC2          					0x06					/** ADS1248 REGISTER : Offset Calibration Coefficient Register 2 **/
#define ADS1248_FSC0          					0x07					/** ADS1248 REGISTER : Full-Scale Calibration Coefficient Register 0 **/
#define ADS1248_FSC1          					0x08					/** ADS1248 REGISTER : Full-Scale Calibration Coefficient Register 1 **/
#define ADS1248_FSC2          					0x09					/** ADS1248 REGISTER : Full-Scale Calibration Coefficient Register 2 **/
#define ADS1248_IDAC0         					0x0A					/** ADS1248 REGISTER : IDAC Control Register 0 **/
#define ADS1248_IDAC1         					0x0B					/** ADS1248 REGISTER : IDAC Control Register 1 **/
#define ADS1248_GPIOCFG       					0x0C					/** ADS1248 REGISTER : GPIO Configuration Register **/
#define ADS1248_GPIODIR       					0x0D					/** ADS1248 REGISTER : GPIO Direction Register **/
#define ADS1248_GPIODAT       					0x0E					/** ADS1248 REGISTER : GPIO Data Register **/

// Register Bit Masks
#define ADS1248_CH_MASK       					0x07			
#define ADS1248_PGA_MASK      					0x70
#define ADS1248_SPS_MASK      					0x0F
#define ADS1248_REFSEL_MASK   					(0x03 << 3)
#define ADS1248_VREF_MASK     					(0x03 << 5)
#define ADS1248_MUXCAL_MASK   					0x07

#define ADS1248_MAX_VAL       					0x7FFFFF
#define ADS1248_MIN_VAL       					0x800000
#define ADS1248_RANGE         					0xFFFFFF
#define ADS1248_INT_REF_MV    					2048

#define ADS1248_TEST_REG      					ADS1248_MUX1

#define ADS1248_DELAY         					_delay_us(10)

// Setting defines for ADS1248
#define ADC_5_SPS             					0x00
#define ADC_10_SPS            					0x01
#define ADC_20_SPS            					0x02
#define ADC_40_SPS            					0x03
#define ADC_80_SPS            					0x04
#define ADC_160_SPS           					0x05
#define ADC_320_SPS           					0x06
#define ADC_640_SPS           					0x07
#define ADC_1000_SPS          					0x08
#define ADC_2000_SPS          					0x09

#define ADC_PGA_1             					(0x00 << 4)
#define ADC_PGA_2             					(0x01 << 4)
#define ADC_PGA_4             					(0x02 << 4)
#define ADC_PGA_8             					(0x03 << 4)
#define ADC_PGA_16            					(0x04 << 4)
#define ADC_PGA_32            					(0x05 << 4)
#define ADC_PGA_64            					(0x06 << 4)
#define ADC_PGA_128           					(0x07 << 4)

#define ADC_IDAC_0uA          					0x00
#define ADC_IDAC_50uA         					0x01
#define ADC_IDAC_100uA        					0x02
#define ADC_IDAC_250uA        					0x03
#define ADC_IDAC_500uA        					0x04
#define ADC_IDAC_750uA        					0x05
#define ADC_IDAC_1000uA       					0x06
#define ADC_IDAC_1500uA       					0x07
#define ADC_IDAC1_IEXT1       					(0x80 << 4)
#define ADC_IDAC1_IEXT2       					(0x81 << 4)
#define ADC_IDAC1_DISCONNECTED 					(0xC0 << 4)
#define ADC_IDAC2_IEXT1       					0x80
#define ADC_IDAC2_IEXT2       					0x81
#define ADC_IDAC2_DISCONNECTED 					0xC0

#define ADC_REF0              					(0x00 << 3)
#define ADC_REF1              					(0x01 << 3)
#define ADC_INTREF            					(0x02 << 3)
#define ADC_INTREF_REF0       					(0x03 << 3)
#define ADC_VREF_OFF          					(0x00 << 5)
#define ADC_VREF_ON           					(0x01 << 5)
#define ADC_VREF_ON_LP        					(0x02 << 5)

#define ADC_MUXCAL_NORM       					0x00
#define ADC_MUXCAL_OFFSET     					0x01
#define ADC_MUXCAL_GAIN       					0x02
#define ADC_MUXCAL_TEMP       					0x03
#define ADC_MUXCAL_REF1       					0x04
#define ADC_MUXCAL_REF0       					0x05
#define ADC_MUXCAL_AVDD       					0x06
#define ADC_MUXCAL_DVDD       					0x07

//////////////////////////////////////////////////////////////////////////		PROTOTYPES		//////////////////////////////////////////////////////////////////////////

uint8_t ADS1248_begin(void);
void ADS1248_end(void);

void ADS1248_reset(void);

void ADS1248_wake(void);
void ADS1248_sleep(void);
void ADS1248_stopContinuous(void);

void ADS1248_startSingle(void);
int32_t ADS1248_sample_raw(uint8_t ch_pos, uint8_t ch_neg, uint8_t gain, uint8_t sampleRate, uint8_t ref);
//float sample(uint8_t muxcal, uint8_t sampleRate);

void ADS1248_setPGA(uint8_t gain);
void ADS1248_setSampleRate(uint8_t rate);
void ADS1248_enableIntRef(void);
void ADS1248_selectRef(uint8_t ref_mux);
void ADS1248_selectMuxCal(uint8_t muxcal);
void ADS1248_selfOffsetCal(void);
void ADS1248_dumpRegs(void);

uint8_t ADS1248_readReg(uint8_t addr);
int32_t ADS1248_readData(void);
void ADS1248_writeReg(uint8_t addr, uint8_t data);

//////////////////////////////////////////////////////////////////////////		FUNCTIONS		//////////////////////////////////////////////////////////////////////////

/**
* @brief This function will Initialize ADS1248 and check connection
* @brief It will return 1 if ADS1248 is connected properly else it will return 0
* @author Mohammed Asim Merchant
* @date 27/02/2019
* @param void.
* @return uint8_t.
*/
uint8_t ADS1248_begin(void)
{
	uint8_t temp;

	// Set pin directions
	spi_init_master(8,0,1);
	ADS1248_SS_DDR |= (1 << ADS1248_SS);
	ADS1248_RESET_DDR |= (1 << ADS1248_RESET);
	ADS1248_START_DDR |= (1 << ADS1248_START);
	ADS1248_DRDY_DDR  &= ~(1 << ADS1248_DRDY);

	// Set pin idle states
	ADS1248_SS_PORT |= (1 << ADS1248_SS);
	ADS1248_DRDY_PORT |= (1<<ADS1248_DRDY);
	ADS1248_RESET_PORT |= (1 << ADS1248_RESET);

	ADS1248_reset();
	
	ADS1248_START_PORT |= (1 << ADS1248_START);
	
	temp = ADS1248_readReg(ADS1248_TEST_REG);
	ADS1248_writeReg(ADS1248_TEST_REG, 0x30);
	if(ADS1248_readReg(ADS1248_TEST_REG) == 0x30)
	{
		ADS1248_writeReg(ADS1248_TEST_REG,temp); 		//Return to reset state
		return 1;
	}
	return 0;
}

/**
* @brief This function will reset ADS1248 by toggling the RESET pin
* @author Mohammed Asim Merchant
* @date 27/02/2019
* @param void.
* @return void.
*/
void ADS1248_reset(void)
{
	ADS1248_RESET_PORT &= ~(1 << ADS1248_RESET);
	_delay_ms(1);			// Min 4*t_osc
	ADS1248_RESET_PORT |= (1 << ADS1248_RESET);
	_delay_ms(1);			// min .6ms
}

/**
* @brief This function will read any ADS1248 register by passing the register address
* @author Mohammed Asim Merchant
* @date 27/02/2019
* @param void.
* @return uint8_t.
*/
uint8_t ADS1248_readReg(uint8_t addr)
{
	uint8_t ret;
	
	ADS1248_SS_PORT &= ~(1 << ADS1248_SS);	// Pull CS low
	
	spi_transceiver((addr & 0x0F) | ADS1248_CMD_RREG);
	spi_transceiver(0x00);
	ret = spi_transceiver(ADS1248_CMD_NOP);
	
	_delay_us(2);
	ADS1248_SS_PORT |= (1 << ADS1248_SS);	// Pull CS high
	_delay_us(1);
	return ret;
}

/**
* @brief This function will write ADS1248 register by passing register address and the data to be written
* @author Mohammed Asim Merchant
* @date 27/02/2019
* @param void.
* @return void.
*/
void ADS1248_writeReg(uint8_t addr, uint8_t data)
{
	ADS1248_SS_PORT &= ~(1 << ADS1248_SS);	// Pull CS low
	
	spi_transceiver((addr & 0x0F) | ADS1248_CMD_WREG);
	spi_transceiver(0x00);
	spi_transceiver(data);
	
	_delay_us(2);
	ADS1248_SS_PORT |= (1 << ADS1248_SS);	// Pull CS high
	_delay_us(1);
}

/**
* @brief This function will return raw reading of channels by passing +ve channe, --ve channel, gain setting, samplerate and reference voltage
* @author Mohammed Asim Merchant
* @date 27/02/2019
* @param void.
* @return int32_t.
*/
int32_t ADS1248_sample_raw(uint8_t ch_pos, uint8_t ch_neg, uint8_t gain, uint8_t sampleRate, uint8_t ref)
{
	int32_t res;
	
	// Enable START to allow writing to registers
	ADS1248_START_PORT |= (1 << ADS1248_START);
	
	// Check if reference on.  If not, turn on and wait...
	if(ADS1248_readReg(ADS1248_MUX1) & ADC_VREF_ON)
	ADS1248_writeReg(ADS1248_MUX1, ADC_VREF_ON | (ADS1248_REFSEL_MASK & ref));
	else
	{
		ADS1248_writeReg(ADS1248_MUX1, ADC_VREF_ON | (ADS1248_REFSEL_MASK & ref));
		// Delay 3ms to allow reference to settle
		_delay_ms(3);
	}
	// Setup gain and sample rate
	ADS1248_writeReg(ADS1248_SYS0, (ADS1248_PGA_MASK & gain) | (ADS1248_SPS_MASK & sampleRate));
	// Choose channels
	ADS1248_writeReg(ADS1248_MUX0, (ADS1248_CH_MASK & ch_neg) | ((ADS1248_CH_MASK & ch_pos) << 3));
	//Bias negative input to (AVDD-AVSS/2) for common mode input requirement
	ADS1248_writeReg(ADS1248_VBIAS, 1 << ch_neg);
	// Disable START to reset sample
	ADS1248_START_PORT &= ~(1 << ADS1248_START);
	_delay_us(1);
	// Start conversion
	ADS1248_startSingle();
	// Wait for completion DRDY timeout occurs after no response for 250ms
	while(bit_is_set(ADS1248_DRDY_PINP, ADS1248_DRDY)){};
	res = ADS1248_readData();
	return res;
}

/**
* @brief This function will toggle START pin of ADS1248 to begin conversion
* @author Mohammed Asim Merchant
* @date 27/02/2019
* @param void.
* @return void.
*/
void ADS1248_startSingle(void)
{
	ADS1248_START_PORT |= (1 << ADS1248_START);
	_delay_us(2);				// Datasheet specifies minimum 3*t_osc which would really be < 1us
	ADS1248_START_PORT &= ~(1 << ADS1248_START);
}

/**
* @brief This function will read adc reading by sending NOP instruction in exchange and return the raw data
* @author Mohammed Asim Merchant
* @date 27/02/2019
* @param void.
* @return int32_t.
*/
int32_t ADS1248_readData(void)
{
	uint32_t ret;
	uint8_t byte;
	ADS1248_SS_PORT &= ~(1 << ADS1248_SS);	// Pull CS low
	
	byte = spi_transceiver(ADS1248_CMD_NOP);
	ret = (((uint32_t)byte) << 16) | ((byte & 0x80) ? 0xFF000000:0x00000000);
	ret |= (((uint32_t)spi_transceiver(ADS1248_CMD_NOP)) << 8);
	ret |= (uint32_t)spi_transceiver(ADS1248_CMD_NOP);
	
	_delay_us(2);
	ADS1248_SS_PORT |= (1 << ADS1248_SS);	// Pull CS high
	_delay_us(1);
	
	return ret;
}

/**
* @brief This function will send ADS1248 the Self Offset Calibration command and will wait till calibration is complete
* @author Mohammed Asim Merchant
* @date 27/02/2019
* @param void.
* @return void.
*/
void ADS1248_selfOffsetCal(void)
{
	uint8_t MUX1_old = ADS1248_readReg(ADS1248_MUX1);
	ADS1248_START_PORT |= (1 << ADS1248_START);
	ADS1248_enableIntRef();
	ADS1248_selectRef(ADC_INTREF);
	ADS1248_START_PORT &= ~(1 << ADS1248_START);
	
	_delay_ms(1);			// Wait for ref to settle
	
	ADS1248_SS_PORT &= ~(1 << ADS1248_SS);
	
	spi_transceiver(ADS1248_CMD_SELFOCAL);
	
	_delay_us(2);
	ADS1248_SS_PORT |= (1 << ADS1248_SS);	// Pull CS high
	_delay_us(1);
	
	// TODO add timeout here
	while(bit_is_set(ADS1248_DRDY_PINP, ADS1248_DRDY)){};
	ADS1248_START_PORT |= (1 << ADS1248_START);
	
	ADS1248_writeReg(ADS1248_MUX1,MUX1_old);
	ADS1248_START_PORT &= ~(1 << ADS1248_START);
}

/**
* @brief This function will enable the internal reference voltage of ADS1248
* @author Mohammed Asim Merchant
* @date 27/02/2019
* @param void.
* @return void.
*/
void ADS1248_enableIntRef(void)
{
	ADS1248_writeReg(ADS1248_MUX1,ADS1248_readReg(ADS1248_MUX1) | ADC_VREF_ON);
}

/**
* @brief This function will write ADS1248 register MUX1 and the reference that is to be selected will be passed in the parameter
* @author Mohammed Asim Merchant
* @date 27/02/2019
* @param void.
* @return void.
*/
void ADS1248_selectRef(uint8_t ref_mux)
{
	ADS1248_writeReg(ADS1248_MUX1,(ADS1248_readReg(ADS1248_MUX1) & ~ADS1248_REFSEL_MASK) | (ref_mux & ADS1248_REFSEL_MASK));
}

/**
* @brief This function will write muxcal data to lower three bits of MUX1 register
* @brief muxcal register can be used to read the internal temperature of the IC as well as other paramters like supply
* @author Mohammed Asim Merchant
* @date 27/02/2019
* @param void.
* @return void.
*/
void ADS1248_end(void)
{
	SPCR &= ~(1<<SPE);
}

void ADS1248_selectMuxCal(uint8_t muxcal)
{
	ADS1248_writeReg(ADS1248_MUX1,(ADS1248_readReg(ADS1248_MUX1) & ~ADS1248_MUXCAL_MASK) | (muxcal & ADS1248_MUXCAL_MASK));
}

/**
* @brief This function will write ADS1248 SYS0 register and will set the PGA only
* @author Mohammed Asim Merchant
* @date 27/02/2019
* @param void.
* @return void.
*/
void ADS1248_setPGA(uint8_t gain)
{
	ADS1248_writeReg(ADS1248_SYS0,(ADS1248_readReg(ADS1248_SYS0) & ADS1248_SPS_MASK) | (gain & ADS1248_PGA_MASK));
}

/**
* @brief This function will write ADS1248 SYS0 register and will set the sample rate only
* @author Mohammed Asim Merchant
* @date 27/02/2019
* @param void.
* @return void.
*/
void ADS1248_setSampleRate(uint8_t rate)
{
	ADS1248_writeReg(ADS1248_SYS0,(ADS1248_readReg(ADS1248_SYS0) & ADS1248_PGA_MASK) | (rate & ADS1248_SPS_MASK));
}

/**
* @brief This function will write ADS1248 stopContinuous data output command (SDATAC)
* @author Mohammed Asim Merchant
* @date 27/02/2019
* @param void.
* @return void.
*/
void ADS1248_stopContinuous(void)
{
	ADS1248_SS_PORT &= ~(1 << ADS1248_SS);  // Pull CS low
	spi_transceiver(ADS1248_CMD_SDATAC);
	_delay_us(2);
	ADS1248_SS_PORT |= (1 << ADS1248_SS);	// Pull CS high
	_delay_us(1);
}

/**
* @brief This function will send ADS1248 WAKEUP command
* @author Mohammed Asim Merchant
* @date 27/02/2019
* @param void.
* @return void.
*/
void ADS1248_wake(void)
{
	ADS1248_SS_PORT &= ~(1 << ADS1248_SS);  // Pull CS low
	spi_transceiver(ADS1248_CMD_WAKE);
	_delay_us(2);
	ADS1248_SS_PORT |= (1 << ADS1248_SS);	// Pull CS high
	_delay_us(1);
}

/**
* @brief This function will send ADS1248 SLEEP command
* @author Mohammed Asim Merchant
* @date 27/02/2019
* @param void.
* @return void.
*/
void ADS1248_sleep(void)
{
	ADS1248_SS_PORT &= ~(1 << ADS1248_SS);  // Pull CS low
	spi_transceiver(ADS1248_CMD_SLEEP);
	_delay_us(2);
	ADS1248_SS_PORT |= (1 << ADS1248_SS);	// Pull CS high
	_delay_us(1); 
}

/**
* @brief This function will read ADS1248 registers sequentially from Register 0
* @author Mohammed Asim Merchant
* @date 27/02/2019
* @param void.
* @return void.
*/
void ADS1248_dumpRegs(void)
{
	ADS1248_START_PORT |= (1 << ADS1248_START);
	for(uint8_t i = 0;i<0x0F;i++)
	{
		//Serial.print("Reg 0x");
		//Serial.print(i,16);
		//Serial.print(" : 0x");
		//Serial.print(ADS1248_readReg(i),16);
		//Serial.print("\n");
	}
	ADS1248_START_PORT &= ~(1 << ADS1248_START);
}

/* Example implementation
	int main(void)
	{
		ADS1248_begin();
		
		while (1)
		{
			int32_t raw_reading = ADS1248_sample_raw(3,2,ADC_PGA_128,ADC_5_SPS,ADC_INTREF);
			_delay_ms(500);
		}
	}
*/

#endif