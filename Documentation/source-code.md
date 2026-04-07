# Boot up

- [x] 用Programmer连接目标

- [x] 检查所有引脚配置
  - [x] 时钟25MHz Bypass
  - [x] ADC0 IN0, IN1
  - [x] DAC Out1 (PA4, Pin 20)
  - [x] SW1-SW5 input, LEDC output
  - [x] SPI2 NSS, SCK, MOSI, to DSS
  - [x] USART1
  - [x] SPI1 SCK, MOSI, MUX_LE_N, MUX_CLR, to HV analog switch
  - [ ] Clock Tree（pending）
- [x] 检查任务
  - [x] profileTask；block在载入profiles之后，不初始化DAC；
  - [x] uxTask；不用修改但需要保持开关全部off；
  - [x] 测试串口命令；
  - [x] 按键测试bug
  - [x] 启动DAC
    - [x] 设置输出百分比50，测量输出电压（profileTask），pass，680-720mV
  - [ ] DAC输出10%，DDS输出100KHz
    - [ ] 测量DDS输出的两路波形





# 文件说明

# 结构

## FreeRTOS

### 任务

`StartUxTask`（command.c）负责用户交互；

`StartProfileTask` （profile.c）负责执行Profile；

1. 载入Profile；
2. 启动DAC，设置`DDS输出电压百分比`为0；
3. 





# TroubleShooting

## 按键测试失败

描述：未修改代码，按键测试不通过。具体体现为1检测为ON，2/3/4/5为OFF，波动开关2和5均工作，3/4未测。

分析：检查原理图发现，开关电路上没有上拉，卡关闭合时100ohm下拉到地，打开时引脚为floating

尝试：修改5个引脚到上拉；

结果：问题修复；

