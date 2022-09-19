In this project, I printed a image using STM32L5 on the screen. The LCD I used is OLED LCD. So, libraries I used is only compatible with OLED LCD. In order to display pictures on the screen, pictures must be converted to hexadecimal from jpg or jpeg. OLED LCD is 128x64 pixels. So before converting images to hexadecimal, set images as 128x64 pixels. If not, pictures may not fit the screen. 

Follow directions respectively :
1.) Convert image jpg to hex
2.) Assign the hexadecimal equivalent of the image to a variable in a new file with the .h extension
3.) Use the required function
4.) Also use I2C communication interface to display spmething on screen. Because, OLED is only compatible I2C connectivity
