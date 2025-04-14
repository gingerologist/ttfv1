/*
 *******************************************************************************
 *                   Driver for frequency synthesizer AD9834
 *******************************************************************************
 *
 *  AD9834 - digital synthesizer, capable of outputting high-precision
 *  sinusoidal and triangular signals. It has a built-in comparator,
 *  allowing generation of square waves for clock generation.
 *  The possibility of phase and frequency modulation is provided.
 *  The frequency registers are 28 bits, with a clock frequency of 75 MHz,
 *  a resolution of 0.28 Hz can be achieved.
 *  Frequency and phase modulation is set by loading registers through
 *  a serial interface and switching registers using software
 *  or with a selection pin and a selection pin, respectively.
 *  AD9834 is programmed using a 3-wire serial interface.
 *  This serial interface operates at a clock frequency of up to 40 MHz and
 *  is compatible with DSP and microcontroller standards.
 *  AD9834 has a power-down pin (standby mode), which allows
 *  external control of the power-down mode.
 *  Unused parts of the device can be turned off to minimize
 *  current consumption.
 *
 *******************************************************************************
*/
#if 0
// #include "spi.h"
#include "AD9834.h"
#define _AD9834_USE_FREERTOS    1                                               // =1 -> operation in a multitasking environment
#if (_AD9834_USE_FREERTOS == 1)
    #include "cmsis_os.h"
    #define _AD9834_DELAY(x)    osDelay(x)
#else
    #define _AD9834_DELAY(x)    HAL_Delay(x)
#endif
#define _75MHZ                  1171875                                         // Divider for calculations with 75 MHz crystal
#define _50MHZ                  781250                                          // Divider for calculations with 50 MHz crystal


// Basic functions for working with the frequency synthesizer
void AD9834_SetRegisterValue(uint16_t regValue)                                 // Function for setting registers
{
    _AD9834_DELAY               (10);
    HAL_SPI_Transmit            (&hspi3, (uint8_t*) &regValue, 1, 5000);        // Sending 16 bits via SPI3 (to RF board)
    _AD9834_DELAY               (10);
}

void AD9834_SetFrequency        (uint16_t reg, uint32_t val)                    // Function for setting the specified frequency
{
    uint16_t freqHi             = reg;
    uint16_t freqLo             = reg;
    freqHi                      |= (val & 0x3FFFC000) >> 14 ;
    freqLo                      |= (val & 0x3FFF);
    AD9834_SetRegisterValue     (AD9834_B28);
    AD9834_SetRegisterValue     (freqLo);
    AD9834_SetRegisterValue     (freqHi);
}

void AD9834_Init                (unsigned long long int f)                      // Function for initializing ad9834 at the specified frequency
{
    unsigned long freq          = 0;
    AD9834_SetRegisterValue     (AD9834_REG_CMD | AD9834_RESET | AD9834_CMD_SW);
    freq                        = (unsigned long)  ((f * 4194304) / _50MHZ);    // Calculation of the final frequency taking into account the external clocking of the IC (from 50 MHz)
    AD9834_SetFrequency         (AD9834_REG_FREQ0,  freq);
    AD9834_SetFrequency         (AD9834_REG_FREQ1,  freq);
    AD9834_SetFrequency         (AD9834_REG_PHASE0, 0);
    AD9834_SetFrequency         (AD9834_REG_PHASE1, 0);
    AD9834_SetRegisterValue     (AD9834_RESET_CLEAR | AD9834_FSEL1 | AD9834_PSEL1 | AD9834_CMD_SW);
}

#endif

