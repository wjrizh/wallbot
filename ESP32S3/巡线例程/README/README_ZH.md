# README_ZH

- ## 开发板选择注意事项

  **开发板请选择“ESP32S3 Dev Module”**

![image-20240911190459676](.\img\board.png)

​	**点击工具选项，红框内的配置请参考下图进行配置**

![image-20240911190557969](.\img\tool.png)



- ## 寻线 IIC寄存器说明（设备地址：0x52）



### 寻线（图像分辨率：160*120）




| 寄存器 | 数据格式               | 具体描述                                                     |
| ------ | ---------------------- | ------------------------------------------------------------ |
| 0xA0   | unsigned char data [4] | **第一区域**<br/>data[1]:红色中心X轴坐标<br/>data[2]:红色中心Y轴坐标<br/>data[3]:红色检测框宽度<br/>data[4]:红色检测框长度 |
| 0xA1   | unsigned char data [4] | **第二区域**<br/>data[1]:红色中心X轴坐标<br/>data[2]:红色中心Y轴坐标<br/>data[3]:红色检测框宽度<br/>data[4]:红色检测框长度 |
| 0xA2   | unsigned char data [4] | **第一区域**<br/>data[1]:绿色中心X轴坐标<br/>data[2]:绿色中心Y轴坐标<br/>data[3]:绿色检测框宽度<br/>data[4]:绿色检测框长度 |
| 0xA3   | unsigned char data [4] | **第二区域**<br/>data[1]:绿色中心X轴坐标<br/>data[2]:绿色中心Y轴坐标<br/>data[3]:绿色检测框宽度<br/>data[4]:绿色检测框长度 |
| 0xA4   | unsigned char data [4] | **第一区域**<br/>data[1]:蓝色中心X轴坐标<br/>data[2]:蓝色中心Y轴坐标<br/>data[3]:蓝色检测框宽度<br/>data[4]:蓝色检测框长度 |
| 0xA5   | unsigned char data [4] | **第二区域**<br/>data[1]:蓝色中心X轴坐标<br/>data[2]:蓝色中心Y轴坐标<br/>data[3]:蓝色检测框宽度<br/>data[4]:蓝色检测框长度 |
| 0xA6   | unsigned char data [4] | **第一区域**<br/>data[1]:紫色中心X轴坐标<br/>data[2]:紫色中心Y轴坐标<br/>data[3]:紫色检测框宽度<br/>data[4]:紫色检测框长度 |
| 0xA7   | unsigned char data [4] | **第二区域**<br/>data[1]:紫色中心X轴坐标<br/>data[2]:紫色中心Y轴坐标<br/>data[3]:紫色检测框宽度<br/>data[4]:紫色检测框长度 |

