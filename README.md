# fan-drone-software


## how to set up the software

1. Clone this repo using Github Desktop or Git. 
2. Install VSCode and Arduino IDE. 
3. Open VSCode, and install an extension called "PlatformIO"
4. Once PlatformIO is installed, click on the alien ant logo on the left sidebar and find an option that says "import project". 
5. click on the "import project" option and select this repo. 
6. platform io will try and install dependencies for the first time and it might take some time. 


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
