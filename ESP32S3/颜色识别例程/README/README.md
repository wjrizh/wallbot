# README

- ## Notices for Development Board Selection

  Select  **ESP32S3 Dev Module** as the development board

![image-20240911190459676](.\img\board.png)



​	**Click on the Tool in menu bar, and refer to the the configuration highlighted in the red box below for setup.**

![image-20240911190557969](.\img\tool.png)



- ## Detection IIC Register Instruction (Device address: 0x52)

  



### Color Detection （Image Resolution：240x240）

| Register address | Data format (unsigned char)                                  |
| ---------------- | ------------------------------------------------------------ |
| 0x00             | data[0]:X-axis coordinate of the red center<br/>data[1]:Y-axis coordinate of the red center<br/>data[2]:Width of the red recognition box<br/>data[3]:Length of the red recognition box |
| 0x01             | data[0]:X-axis coordinate of the green center<br/>data[1]:Y-axis coordinate of the green center<br/>data[2]:Width of the green recognition box<br/>data[3]:Length of the green recognition box |
| 0x02             | data[0]:X-axis coordinate of the blue center<br/>data[1]:Y-axis coordinate of the blue center<br/>data[2]:Width of the blue recognition box<br/>data[3]:Length of the blue recognition box |
| 0x03             | data[0]:X-axis coordinate of the purple center<br/>data[1]:Y-axis coordinate of the purple center<br/>data[2]:Width of the purple recognition box<br/>data[3]:Length of the purple recognition box |
| 0x04             | data[0]:Red ID number<br/>data[1]:Green ID number<br/>data[2]:Blue ID number<br/>data[3]:Purple ID number |

