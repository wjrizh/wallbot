# README

- ## Notices for Development Board Selection

  Select  **ESP32S3 Dev Module** as the development board

![image-20240911190459676](.\img\board.png)



​	**Click on the Tool in menu bar, and refer to the the configuration highlighted in the red box below for setup.**

![image-20240911190557969](.\img\tool.png)



- ## Face Detection IIC Register Instruction (Device address: 0x52)

  



### Face Detection（Image Resolution：240x240）

| Register address | Data format (unsigned char)                                  |
| :--------------- | :----------------------------------------------------------- |
| 0x01             | data[0]:X-axis coordinate of the face center<br/>data[1]:Y-axis coordinate of the face center<br/>data[2]:Width of the detection box<br/>data[3]:length of the detection box<br/> |





