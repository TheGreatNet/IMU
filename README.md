## 2019.10.16

串口调通，传感器的数据可以通过USART2进入stm32。数据在IIR滤波时出现问题，待解决。

### 2019.10.29

代码基本完成，收完一轮数据(2秒)后，程序继续运行出现instruction error，一次出现在bus，一次出现在memory。