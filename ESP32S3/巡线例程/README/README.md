# README

- ## Notices for Development Board Selection

  Select  **ESP32S3 Dev Module** as the development board

![image-20240911190459676](.\img\board.png)



​	**Click on the Tool in menu bar, and refer to the the configuration highlighted in the red box below for setup.**

![image-20240911190557969](.\img\tool.png)



- ## Line Following IIC Register Instruction (Device address: 0x52)



### Line Following（Image Resolution：160x120）


| Register address | Data format (unsigned char)                                  |
| ---------------- | ------------------------------------------------------------ |
| 0xA0             | **Region 1**<br/>data[0]:X-axis Coordinated of the red center <br/>data[1]:Y-axis Coordinated of the red center<br/>data[2]:Width of the red detection box<br/>data[3]:Length of the red detection box<br/> |
| 0xA1             | **Region 2**<br/>data[0]:X-axis Coordinated of the red center <br/>data[1]:Y-axis Coordinated of the red center<br/>data[2]:Width of the red detection box<br/>data[3]:Length of the red detection box<br/> |
| 0xA2             | **Region 1**<br/>data[0]:X-axis Coordinated of the green center <br/>data[1]:Y-axis Coordinated of the green center<br/>data[2]:Width of the green detection box<br/>data[3]:Length of the green detection box<br/> |
| 0xA3             | **Region 2**<br/>data[0]:X-axis Coordinated of the green center <br/>data[1]:Y-axis Coordinated of the green center<br/>data[2]:Width of the green detection box<br/>data[3]:Length of the green detection box<br/> |
| 0xA4             | **Region 1**<br/>data[0]:X-axis Coordinated of the blue center <br/>data[1]:Y-axis Coordinated of the blue center<br/>data[2]:Width of the blue detection box<br/>data[3]:Length of the blue detection box<br/> |
| 0xA5             | **Region 2**<br/>data[0]:X-axis Coordinated of the blue center <br/>data[1]:Y-axis Coordinated of the blue center<br/>data[2]:Width of the blue detection box<br/>data[3]:Length of the blue detection box<br/> |
| 0xA6             | **Region 1**<br/>data[0]:X-axis Coordinated of the purple center <br/>data[1]:Y-axis Coordinated of the purple center<br/>data[2]:Width of the purple detection box<br/>data[3]:Length of the purple detection box<br/> |
| 0xA7             | **Region 2**<br/>data[0]:X-axis Coordinated of the purple center <br/>data[1]:Y-axis Coordinated of the purple center<br/>data[2]:Width of the purple detection box<br/>data[3]:Length of the purple detection box<br/> |
