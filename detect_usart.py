import serial
import serial.tools.list_ports

# 打印可用串口列表

port_list = list(serial.tools.list_ports.comports())
print(port_list)
if len(port_list) == 0:
    print("无可用串口")
else:
    for i in range(0, len(port_list)):
        print(port_list[i])
