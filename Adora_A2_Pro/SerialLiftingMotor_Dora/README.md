# Adora A2 Pro

## 1. Dora 节点

driver_dora.yaml 存放了丝杠的测试yaml文件，该文件描述了两个dora节点，节点A以10S的间隔时间发送位置0 和 位置32768*200 测试丝杠电机的行程。

启动方法

```
dora up
dora start driver_dora.yaml --name test_motor
```

此时 丝杠会周期性的上下运动
