Requirements:
1. Arduino IDE https://www.arduino.cc/en/software
2. Blynk IOT platform setup

Steps: 
1. Download Arduino IDE https://www.arduino.cc/en/software
2. Install ESP32 on arduino IDE https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/
3. Installing MPU-6050 Libraries
   - Open your Arduino IDE and go to Sketch > Include Library > Manage Libraries. The Library Manager should open.
   - Type “adafruit mpu6050” on the search box and install the library.
   - Then, search for “Adafruit Unified Sensor”. Scroll all the way down to find the library and install it.
   - Finally, search for “Adafruit Bus IO” and install it.
4. Installing Pulse Sensor Arduino Library
   - Open Arduino IDE and click on Sketch > Library > Manage Libraries
   - Type ‘pulsesensor’ in the search bar and press enter. Install the latest version of the library.
   - After installation of the library, restart your IDE
5. Blynk IOT Setup:
   - Register https://blynk.cloud/dashboard/register
   - After creating account share me your email, I will transfer device
   - After transfering device to your account, select that device and then go to device info, copy those credentials shown on screen and replace them with the ones in the code
   - Also, change wifi name and password according to the wifi you are going to connect it.
   - You can do the connection of sensors with ESP 32 as shown in the image uploaded on the github repo
   - After that you can connect ESP32 with your laptop, go to your Arduino IDE where code is open and then select proper port and then push code to ESP32, while pushing code continuously hold reset button on ESP32
   - Now you are good to go, Now you can see readings on terminal, blynk IOT web dashboard and android dashboard if configured.
   - Next time you just need to connect ESP32 to laptop as a power source no need to run code, it will automatically connect to the WiFi network you wrote in code.

