# VAL3000 HTTP Server Firmware
## Getting Started with the VAL3000

This firmware will get you started with spinning the VAL-1000 WiFi stepper board.

## Step 1 - Connect Power & Motor
1. Connect a 12V 2A+ power cord like [this](https://amzn.to/30qsbh7)
2. Connect a stepper motor like [this](https://amzn.to/3kUWkP4)

If you need to move something heavy, over 50 lbs, consider using a geared motor [like this](https://amzn.to/3c9xmI0)

## Step 2 - Open Browser on device with Wi-Fi
Use your phone, laptop, or other device with Wifi and open up a browser such as Chrome or Firefox.

Go to http://192.168.4.1

## Step 3 - Test

In the top box, enter a value from 0-100, the motor should spin.


## Step 3 - Setup

The position value is a percentage vate from 0-100. If you enter 100, it will move 100 percent of the way. If you enter 0, it will move to the home position.

The position percentage value is based on the max steps value. The max steps is the number of steps you want it to move to open 100 percent of the way. 

For example, if you have a curtain opener, and need it to move 20000 steps to open and close, you will enter 20000 here. 

There are 200 steps per revolution. To figure out how many steps to set, first measure the distance you want your device to travel. Let's say it's 20 inches, which is 508 mm.

Next, measure the diameter of the pulley you are using. In the case of the S1 curtain opener, it is 12mm.

Next, convert 12mm diameter to circumferance which is 37.7 mm.

12mm = 0.472441 inches
37.7mm = 1.484252 inches

Next, divide the total travel by cicumferance 

inches = travel/1.484252

if total travel is 20 inches, then revolutions = 13.4748 revolution
at 200 steps/revoltion, total steps = 2694.96

## Step 2 - Download and open the Arduino code
Using Arduino, open the VAL-1000.ino file

You will need to install the ESP32 core, as well as several libraries:

Install the ESP32 core by following the instructions [here](https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/)

Install the following libriaires inside Arduino
1. [TMC Stepper](https://www.arduino.cc/reference/en/libraries/tmcstepper/)
2. [FastAccelStepper](https://www.arduino.cc/reference/en/libraries/fastaccelstepper/)
3. [ezTime](https://www.arduino.cc/reference/en/libraries/eztime/)
4. [ESPUI](https://github.com/s00500/ESPUI)
5. [ArduinoJson](https://www.arduino.cc/reference/en/libraries/arduinojson/)

Install the following libraries outside Arduino. These cannot be installed using the Arduino library manager.
1. [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer)
2. [AsyncTCP](https://github.com/me-no-dev/AsyncTCP)  

a. Installing the ESPAsyncWebServer library

- [Click here](https://github.com/me-no-dev/ESPAsyncWebServer/archive/refs/heads/master.zip) to download the ESPAsyncWebServer library. You should have a .zip folder in your Downloads folder
- Unzip the .zip folder and you should get ESPAsyncWebServer-master folder
- Rename your folder from ESPAsyncWebServer-master to ESPAsyncWebServer
- Move the ESPAsyncWebServer folder to your Arduino IDE installation libraries folder

b. Installing the AsyncTCP Library

- The ESPAsyncWebServer library requires the AsyncTCP library to work. Follow the next steps to install that library:
- [Click here](https://github.com/me-no-dev/AsyncTCP/archive/refs/heads/master.zip) to download the ESPAsyncTCP library. You should have a .zip folder in your Downloads folder
- Unzip the .zip folder and you should get ESPAsyncTCP-master folder
- Rename your folder from AsyncTCP-master to AsyncTCP
- Move the ESPAsyncTCP folder to your Arduino IDE installation libraries folder

Finally, re-open your Arduino IDE. Compile the sketch and everything should compile.

## Step 3 - Upload the sketch
You do not need to change anything in the code.

Upload the firmware to your board.


## Step 4 - Connect to board as Access Point

Using your smart phone or laptop (or any device with wifi), look for the wifi network name VALAR-AP-XXXXXXXX

Connect to VALAR-AP-XXXXXXXX

This is an access point which means you will connect directly to the device and will lose you internet connection.

When prompted, enter the wifi password "password". If you want to change this, you can modify the code in the API.h file for the variable ap_password.

In your browser, enter http://192.168.4.1/

You are now connected to the device and can begin to control the motor directly.

## Step 5 - Add board to your network

It's best to add this device to your network so you do not need to connect to it directly. 

To do so, go to http://192.168.4.1/wifi

Enter in the name and password of your wifi router. Your device will now connect to your router.

Change your wifi network back to your router because you will get disconnected from the VALAR-AP network.

You now need to find the IP address that your router assigned to the VAL-1000 device. To do this, use a network scanning tool like [angryip](https://angryip.org/)

The hostname should be "esp32-arduino". Find the IP address of this device and enter the IP address into your browser.

You should now be connected to the device.

## Step 6 - Control your motor

Under the "Position" section, enter a value of 50 and click "Set Position". The motor should begin spinning to the 50 percent position.

Set motor parameters by modifiying the following values:

### max_steps
This value is used to set the maximum number of steps to move the motor to 100 percent. If you have a curtain or window, set the number of steps that will be required to move the motor to open/close the curtain all the way. 

There are 200 steps per revolution so use some math to figure out how many steps you need.

### current
This value (400-2000) sets the amount of current in milliamps that will run through the motor. The higher the current, the stronger the motor will be. However, it will also run hotter as well as make setting the stall value more difficult. It is recommended to set the current to the minimum required to move your motor. Start at a small value and move up until the motor moves how you want. The maximum value is 2000 mA. 

### stall
This value (0-255) sets the stall value of the motor. Stallgaurd is fairly complex and I recommend taking my [Udemy course](https://www.udemy.com/course/trinamic/?referralCode=F21BCEB4F4C3C664D13A) to learn more. Set this value to 0 to disable it if you do not understand how it works.

### accel
This value sets the acceleration of the motor. It is recommened you keep this the same as max_speed when using stallGuard.

### max_speed
This value (0-500) sets the speed of the motor. The driver will use the acceleration value to speed up to this value.

Press the "Set Parameters" button to set the values.

### Set Zero
Pressing this button will set the position of the device back to 0. 


## Step 7 - Control via API

You may want to set up some automations to control your device. To trigger the motor to move, simply send an HTTP command via your browser or preferred method.

Go to the API tab to find the exact address to send the commands to.

Replace the "0" with a value of 0-100 to set the position.
