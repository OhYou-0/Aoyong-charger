quick project to utilize a esp32-s3 as a modbus monitor for probably most Aoyong solar chargers to map inputs to a local web UI.
Built specifically for the seeed xiao esp32-s3, others probably wont work unless they have the same flash structure
Connect esp32 to the display side of the comms cable as here:
aoyong VCC = xiao 5v or vusb
aoyong Tx/B = xiao D7 or gpio44
aoyong VSS = xiao gnd

functions as standalone with the wifi ssid mqtt and password as "password", or you can connect it to your wifi to view it there. 
Also provides a telnet server for debug purposes.

Designed specifically for the red charge controllers that match the search term "480VDC 24-192V 60A MPPT Solar Charge Controller". Mine was branded "JYYHSRPOWER" but inside the boards are branded Aoyong.

This project was mostly done from scratch through Codex, I left some artifacts of the learning process in to be used for adapting this to other charge controllers or any modbus devices. 
The mppt_sniffer.py can be called by Codex as a tool to decode modbus payloads utilizing a cp2102 usb to ttl adaptor or similar. The notes should be useful for building context.

<img width="945" height="1012" alt="image" src="https://github.com/user-attachments/assets/35f3e0bb-1088-4b8d-ac5c-a95740e306f7" />
