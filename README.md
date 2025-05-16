## Simple Project for testing the TWAI interface

Flash it to an ESP32 ECO3 connected to a CAN-Bus and send following message:
CAN-ID = 0x1, 
Message Length = 8,
Data = 0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67, 0x89,
CycleTime = 5ms 

Then get some interference to the wire/connection (e.g. pull the CAN-Connector and reinsert it) and see messages in the application delivered from the TWAI interface, which have never been on the CAN-Bus.

Tested with ESP-IDF 5.3.1 and some modifications from here: https://github.com/espressif/esp-idf/issues/12474

## Results

A TWAI-RX Task is spawend which simply checks if the message content is the same as send out by the PC-Application:
![grafik](https://github.com/user-attachments/assets/8d8e4191-086c-4757-a69a-059bda9f9d3e)

When the connection is cut and reinserted (not every time, but often) there is a message with incorrect data deliverd to the ESP32 Application layer:
![grafik](https://github.com/user-attachments/assets/c7bb6547-4652-4662-ac55-df517cbc5ab6)

This message was never sent and other devices on the CAN-Bus don't see this message.

