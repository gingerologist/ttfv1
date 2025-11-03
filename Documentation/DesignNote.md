# Interfaces

## MCU

| Pin Name                                             | Function        | IOC      |
| ---------------------------------------------------- | --------------- | -------- |
| OSC_IN, OSC_OUT                                      | 25MHz           | Y        |
| OSC32_IN, OSC32_OUT                                  | 32KHz           | N        |
| DDS_RESET, DDS_FSEL, DDS_SEL<sup>1</sup>             | DDS             | as input |
| SPI2_NSS, SPI2_SCK, SPI2_MOSI                        | DDS             | Y        |
| TIM1_MCO                                             | DDS (CLK)       | Y        |
| DAC1_OUT                                             | DDS (Amplitude) | Y        |
| MUX1_CLR, MUX1_LE_N, SPI1_SCK, SPI1_MOSI<sup>2</sup> | MUX             | Y        |
| ADC1_IN0<sup>3</sup>                                 | HVSIG           | Y        |
| SW1,SW2,SW3,SW4,SW5,LEDC<sup>4</sup>                 | USER            |          |
| USART2_TX, USART2_RX                                 | USER/DEBUG      | Y        |



1. All pulled down , which means out-of-reset, select FREQ0 and PHASE0 register.
2. MUX GPIO default output high, SPI Mode 3 according to diagram, but not verified. 8 Bits. MSB First. 5.25Mbps as starting point.
3. Via a T-shape network to translate +/- high voltage to 0-3.3V
4. All SWs are pulled-up by GPIO configuration. No external pull-up on boards. LEDC output is active high and defaults to low.