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


## how to esp-now
esp-now is a way to talk between two esp32s. 

the ```findmac.cpp``` file contains code that is used to find the mac address of both esp32s. They must be found before running ```espnow.cpp```. 

After both mac addresses are found, then go to ```espnow.cpp``` and set the two mac addresses to:

```

uint8_t peerAddress[] = {0x98, 0x88, 0xE0, 0x14, 0xD6, 0xF0}; // <-- change this for each board

```

Since both boards run the same file, you need to set the mac address to the receiver and then flash one board and then set it to sender and then flash the second board. 

Then flash both boards with the same code but with different peer address. They shouhld be able to talk to each other. **Just remember to comment out main.cpp and findmac.cpp** so its the only file running. 

oct 22 by carl
