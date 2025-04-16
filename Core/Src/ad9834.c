
#include "stm32f4xx_hal.h"

#include "main.h"
#include "ad9834.h"

/*
 * This file is not used, yet.
 */

// Pin definitions
#define AD9834_RESET_PIN  GPIO_PIN_6
#define AD9834_RESET_PORT GPIOC         // Changed to PC6
#define AD9834_PSEL_PIN   GPIO_PIN_9    // Changed to match your specifications
#define AD9834_PSEL_PORT  GPIOA
#define AD9834_FSEL_PIN   GPIO_PIN_10   // Changed to match your specifications
#define AD9834_FSEL_PORT  GPIOA

// External SPI handler declared in main.c
extern SPI_HandleTypeDef hspi2;

// Control register bits
#define AD9834_B28        (1 << 13)
#define AD9834_HLB        (1 << 12)
#define AD9834_FSELECT    (1 << 11)
#define AD9834_PSELECT    (1 << 10)
#define AD9834_RESET      (1 << 8)
#define AD9834_SLEEP1     (1 << 7)
#define AD9834_SLEEP12    (1 << 6)
#define AD9834_OPBITEN    (1 << 5)
#define AD9834_MODE       (1 << 1)
#define AD9834_DIV2       (1 << 3)

// Frequency register addresses
#define AD9834_FREQ0_REG  (0x4000)
#define AD9834_FREQ1_REG  (0x8000)
#define AD9834_PHASE0_REG (0xC000)
#define AD9834_PHASE1_REG (0xE000)

// MCLK frequency (Hz)
#define AD9834_MCLK       12500000  // Updated to match your actual MCO frequency of 12.5MHz

// Initialize the AD9834
void AD9834_Init(void) {
    // Configure GPIO pins for RESET, FSELECT, PSELECT
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIO clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();  // Added for RESET pin on GPIOC

    // Configure FSELECT, PSELECT pins on GPIOA
    GPIO_InitStruct.Pin = AD9834_FSEL_PIN | AD9834_PSEL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure RESET pin on GPIOC
    GPIO_InitStruct.Pin = AD9834_RESET_PIN;
    HAL_GPIO_Init(AD9834_RESET_PORT, &GPIO_InitStruct);

    // Set initial pin states
    HAL_GPIO_WritePin(AD9834_RESET_PORT, AD9834_RESET_PIN, GPIO_PIN_SET);  // Put in reset initially

    // Configure MCLK
    AD9834_ConfigMCLK();

    // Reset sequence
    HAL_Delay(1);  // Short delay
    HAL_GPIO_WritePin(AD9834_RESET_PORT, AD9834_RESET_PIN, GPIO_PIN_RESET);  // Take out of reset
    HAL_Delay(1);  // Short delay

    // Initialize with default settings - B28 enabled for full 28-bit frequency writes
    // AD9834_WriteRegister(AD9834_B28 | AD9834_RESET);
}

extern UART_HandleTypeDef huart2;

// Write a 16-bit value to the AD9834 using hardware SPI
void AD9834_WriteRegister(uint16_t data) {
    HAL_SPI_Transmit(&hspi2, (uint8_t*)&data, 1, HAL_MAX_DELAY);
}




// Set frequency (in Hz) to one of the frequency registers (0 or 1)
void AD9834_SetFrequency(uint8_t reg, uint32_t freq) {
    // Calculate frequency word
    // freq_word = (freq * 2^28) / MCLK
    uint32_t freq_word = (uint32_t)(((uint64_t)freq << 28) / AD9834_MCLK);

    // uart_printf(&huart2, "freq_word: %d\r\n", freq_word);

    uint16_t freq_reg_addr = (reg == 0) ? AD9834_FREQ0_REG : AD9834_FREQ1_REG;

    // Write lower 14 bits
    uint16_t freq_lsb = (freq_word & 0x3FFF) | freq_reg_addr;

    // Write upper 14 bits
    uint16_t freq_msb = ((freq_word >> 14) & 0x3FFF) | freq_reg_addr;


    // Write to the device - B28 mode requires writing both words
    AD9834_WriteRegister(freq_lsb);  // LSB first
    AD9834_WriteRegister(freq_msb);  // Then MSB
}

// Set phase value to one of the phase registers (0 or 1)
void AD9834_SetPhase(uint8_t reg, uint16_t phase) {
    uint16_t phase_reg_addr = (reg == 0) ? AD9834_PHASE0_REG : AD9834_PHASE1_REG;

    // Only use 12 bits of phase
    uint16_t phase_data = (phase & 0x0FFF) | phase_reg_addr;

    // Write to the device
    AD9834_WriteRegister(phase_data);
}

void AD9834_ConfigMCLK(void) {
    // Configure MCO1 pin (PA8) to output 20MHz
    // HSE is typically 8MHz for STM32F405, use PLL to get 20MHz

    // Configure MCO1 to output 20MHz from PLL
    RCC->CFGR &= ~RCC_CFGR_MCO1;       // Clear MCO1 bits
    RCC->CFGR |= RCC_CFGR_MCO1_1;      // Select PLL as source
    RCC->CFGR &= ~RCC_CFGR_MCO1PRE;    // Clear MCO1PRE bits
    RCC->CFGR |= RCC_CFGR_MCO1PRE_2;   // Divide by 4 (assuming 80MHz PLL output)

    // Configure PA8 as MCO output
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// Configure MCLK output on STM32F405
//void AD9834_ConfigMCLK(void) {
//    // With 25MHz HSE, we can use MCO1 with divider settings
//    // Option 1: Keep current 12.5MHz by dividing HSE by 2
//    RCC->CFGR &= ~RCC_CFGR_MCO1;       // Clear MCO1 bits
//    RCC->CFGR |= RCC_CFGR_MCO1_0;      // Select HSE as source (value 01)
//    RCC->CFGR &= ~RCC_CFGR_MCO1PRE;    // Clear MCO1PRE bits
//    RCC->CFGR |= RCC_CFGR_MCO1PRE_1;   // Divide by 2 (25MHz/2 = 12.5MHz)
//
//    /*
//    // Option 2: Try to get closer to 20MHz using PLL
//    // Not recommended as it would change system clocks
//    // Only uncomment if you want to reconfigure the entire system clock
//
//    // Configure MCO1 to output PLL/4 which could be ~21MHz depending on your PLL settings
//    RCC->CFGR &= ~RCC_CFGR_MCO1;       // Clear MCO1 bits
//    RCC->CFGR |= RCC_CFGR_MCO1_1;      // Select PLL as source
//    RCC->CFGR &= ~RCC_CFGR_MCO1PRE;    // Clear MCO1PRE bits
//    RCC->CFGR |= RCC_CFGR_MCO1PRE_2;   // Divide by 4
//    */
//
//    // Configure PA8 as MCO output
//    GPIO_InitTypeDef GPIO_InitStruct = {0};
//    GPIO_InitStruct.Pin = GPIO_PIN_8;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//}

// Main function to set up AD9834 for 200kHz sine wave
void AD9834_Setup200kHzSineWave(void) {
    // Initialize the AD9834
    AD9834_Init();

    // AD9834_WriteRegister(0x2100);
    // AD9834_WriteRegister(0x4937);
    // AD9834_WriteRegister(0x4106);
    // AD9834_WriteRegister(0xC000);
    // Set frequency for 200kHz sine wave in FREQ0 register
    // AD9834_SetFrequency(0, 200000);

    // Set default phase (0 degrees)
    // AD9834_SetPhase(0, 0);


    AD9834_WriteRegister(0x2100);
    AD9834_WriteRegister(0x50C7);
    AD9834_WriteRegister(0x4000);
    AD9834_WriteRegister(0xC000);

    // Select FREQ0 and PHASE0 registers (low on FSELECT and PSELECT pins)
    // HAL_GPIO_WritePin(AD9834_FSEL_PORT, AD9834_FSEL_PIN, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(AD9834_PSEL_PORT, AD9834_PSEL_PIN, GPIO_PIN_RESET);

    // Final control register configuration - enable sine wave output
    AD9834_WriteRegister(0x2000); // Just B28 bit set, RESET cleared, MODE=0 for sine wave

}

/*
// IMPORTANT: Your current SPI2 initialization requires modification
// Change this in your main.c or wherever MX_SPI2_Init is defined:

void MX_SPI2_Init(void)
{
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;    // CPOL=0
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;        // CPHA=0
  hspi2.Init.NSS = SPI_NSS_SOFT;                // Must change to software CS management
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2; // Your fast setting
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;       // MSB first
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
}
*/

// Usage in main function
//int main(void) {
//    HAL_Init();
//    // Initialize your system clock, SPI, and other peripherals here
//
//    // Setup the AD9834 for 200kHz sine wave generation
//    AD9834_Setup200kHzSineWave();
//
//    while (1) {
//        // Main loop
//    }
//}


