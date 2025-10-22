# fan-drone-software


## how to set up the software

1. Clone this repo using Github Desktop or Git. 
2. Install VSCode and Arduino IDE. 
3. Open VSCode, and install an extension called "PlatformIO"
4. Once PlatformIO is installed, click on the alien ant logo on the left sidebar and find an option that says "import project". 
5. click on the "import project" option and select this repo. 
6. platform io will try and install dependencies for the first time and it might take some time. 


## How to setup arduino IDE
1. Download arduino IDE 2.x version
2. Also download the drivers for the esp32 boards at: [for windows and mac](https://www.silabs.com/software-and-tools/usb-to-uart-bridge-vcp-drivers?tab=downloads) and [for m1 macs](https://github.com/WCHSoftGroup/ch34xser_macos)
3. Install arduino IDE and open it, it will run a first time setup, let it do its thing. 
4. Go to the board manager (second icon on the left side bar), search for esp32, and install the package by expressif systems (second one in the image). It will take a while. 
![alt text](image-1.png)

5. Go to the top and there should be a dropdown that says "Select Board", find esp32-dev-module in the left hand side, find the port that's called "COM" something on the right. Select that and save. 
![alt text](image-2.png)

6. You are all set.

## structure of the project
1. ```readme.md``` is readme
2. ```platformio.ini``` is the config file for platform io, it contains the information for which board to use and whatnot. We will be using the esp32-s3-devkitm-1 board. 
3. put your actual code files into ```/src```. 

## buiding and running code
1. connect the board to the laptop
2. press ```ctrl + alt + b``` to build, a terminal window will pop up. 
3. to upload, open the platform io sidebar, and click "upload". 
4. to monitor, open the platform io sidebar and click "monitor".


written by carl sept 20 2025.

## Notes Oct 11 Build and Troubleshooting
- Got the sensor hooked up and working

### How to setup  MPU9250 and laptop
1. Hook up the wires using this image as a reference. Use 8 as clock and 9 as the other one.  

![alt text](image.png)

2. Clone the repo and head to the ekf branch Richard created. 

3. Open the project in Platform IO, then import. 

4. Use the alien ant menu to build, upload. 

### How to read MPU9250 raw values 
1. build and run the project. 

2. Fire up Arduino IDE, find the appropriate board and COM setup. Board should be ESP32-Dev-Module and port should be either COM5 or COM7 depending on your computer. 

3. Open serial monitor, set baud rate at 115200 and data should start showing up

4. Then open serial plotter, if values dont show, go to "Tools" > "Serial Plotter", values should start showing up soon. 

Richard and Carl Oct 11.

## carl notes oct 16
This probably is quite similar to what we're doing (but answers given was to use a low-pass instead): https://robotics.stackexchange.com/questions/12633/extended-kalman-filter-for-imu


This example illustrates a MEKF example: https://matthewhampsey.github.io/blog/2020/07/18/mekf

This example gives the complete math to implement a EKF for the mpu9250 sensor: https://www.sikhrobotics.com/orientation/ekf/implementation/

This example gives the full math to implement a kalman filter: https://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/

This is the code for the blog link above: https://github.com/TKJElectronics/KalmanFilter

This might be the most use for us: https://github.com/simondlevy/TinyEKF

TinyEKF is a full blown library and it seems to be doing what we want it to do, we just need to declare the model and 
feed the values in. 

This is another similar implementation but inspired by EKF, not exactly an EKF algorithm: https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

by carl

## Oct 22 Notes

I've collected time measurements on gathering orientation estimates from our MPU 9250. On average, updating registers and estimating orientation takes ~211us which is ideal if we're shooting for a control loop period of less than 1kHz (1000us). Also, since I haven't posted it yet, here is a video visualizing the RPY orientation of our sensor at a moderate sample rate (note its not perfect since its my hand and not a testing rig).

https://github.com/user-attachments/assets/7d2adfda-667f-469d-be9e-9518fe9fe755

Here are some other changes in our flight controller approach:
+ switched to using the MPU library sensor fusion algorithm (mahony filter) as its simpler, just as accurate/fast as our comlementary filter, and more appropriate for a first prototype.
+ designed the structure of our flight controller firmware to run off two cores: one core for control, another for reporting. these cores share a state structure of our flight controller, accessible via semaphore.
+ began outlining a cascaded PID control loop with a gryo-rate based inner loop and gyro-angle based outer loop. perhaps a single PID control loop for yaw as well.

Richard 
