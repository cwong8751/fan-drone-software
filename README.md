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

# EKF notes

Initialization:
  x₀ = [1, 0, 0, 0, 0, 0, 0]ᵀ
  P₀ = small diagonal matrix
  Q, R = tuned process/measurement noise matrices

For each time step:
  1. Read IMU data (gyro, accel, mag)
  2. PREDICT:
       - compute q_dot = 0.5 * Ω(gyro - bias) * q
       - integrate: q += q_dot * dt
       - normalize q
       - propagate P = F P Fᵀ + Q
  3. UPDATE:
       - normalize accel, mag
       - compute predicted gravity & magnetic field
       - compute residual y = z_meas - z_pred
       - compute H, K, and apply x ← x + K y
       - normalize quaternion
  4. Output current quaternion estimate

also need to tune Q and R parameters. bias might be optional actually.

Written by Richard


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
