## 2019.23.4

功能完成，IMU数据进入CMU，计算位置后从USART1输出给上位机。在教研室外转了一圈，效果较好。

![09BA8591521C64B344522A1092395D07](/Users/chenjunting/Library/Containers/com.tencent.qq/Data/Library/Caches/Images/09BA8591521C64B344522A1092395D07.png)

下一步需要将蓝牙模块加入，给上位机传输数据。

## 2019.12.5

蓝牙模块加入，但是算法效果从测试看有所下降，怀疑是蓝牙模块的收发影响了IMU数据的接收，下一步试试提高蓝牙波特率。