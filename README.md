## Simple Project for testing the TWAI interface

Flash it to an ESP32 ECO3 connected to a CAN-Bus and send following message:
CAN-ID = 0x1, 
Message Length = 8,
Data = 0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67, 0x89,
CycleTime = 5ms 

Then get some interference to the wire/connection (e.g. pull the CAN-Connector and reinsert it) and see messages in the application delivered from the TWAI interface, which have never been on the CAN-Bus.

Tested with ESP-IDF 5.3.1 and some modifications from here: https://github.com/espressif/esp-idf/issues/12474
