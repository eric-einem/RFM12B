RFM12B Library
----------------
By Eric Einem (eric.einem@plantaware.com)
<br/>
Based on the RFM12 driver from Felix Rusu (felix@lowpowerlab.com)
<br/>
http://opensource.org/licenses/mit-license.php

###Features:
- Compatible with ChipKit Arduino compatible PIC32MX boards.
- Any pin can be selected for the SPI SS pin
- easy API with a few simple functions for basic usage
- 127 possible nodes on 256 possible networks
- 128 bytes max message length
- customizable transmit power (8 levels) for low-power transmission control
- customizable air-Kbps rate allows fine tuning the transmission reliability vs speed (transmitting slower is more reliable but takes more time which implies more power usage)
- Low battery detector with customizable low voltage threshold
- Interrupt driven
- Support for targeted ACK instead of broadcasted ACK (possible because of the new source byte in the header)
encryption with XXTEA algorithm by David Wheeler, adapted from http://en.wikipedia.org/wiki/XXTEA
Support for these chips: PIC32, ATMega8 family (ATmega168, ATMega328) ATMega2560, ATMega1280, ATMega644P, ATMega32u4. Tested on an ATMega328 and ChipKit WF32

###Limitations
- The Encryption does not seem to work on the ChipKit WF32.
- The sleep_mode is not supported.  Not efficient for battery powered use.
- The crc16update function is not optimized.
- Better suited for the more powerful PIC32MX processor as the code is not as optimized as the LowPowerLab version.  All IO is performed using the Arudino library, not with AVR or PIC32 registers.

###Installation
Copy the content of this library in the "Arduino/libraries/RFM12B" folder.
<br />
To find your Arduino folder go to File>Preferences in the Arduino IDE or MPIDE.
<br/>
See [this tutorial](http://learn.adafruit.com/arduino-tips-tricks-and-techniques/arduino-libraries) on Arduino libraries.

###Saple usage
- [Sender](https://github.com/LowPowerLab/RFM12B/blob/master/Examples/Send/Send.ino)
- [Receiver](https://github.com/LowPowerLab/RFM12B/blob/master/Examples/Receive/Receive.ino)
- More examples in the [Examples folder](https://github.com/LowPowerLab/RFM12B/tree/master/Examples)


