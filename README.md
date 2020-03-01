# SIGNALESP ESP8266 with cc1101 support 


This is a special version of the SIGNALESP software that works for me. Unfortunately I did not get the official version (https://github.com/RFD-FHEM/SIGNALESP) to work. It contains the wifi-manager and I did not find a version of that software that worked. So I decided on building my own version without wifi-manager. This means that you have to fill in the wifi data for your router connection into the source before you compile the code.


### Getting started

You have to find the following lines:

```
const char* ssid = "<ssid>"; 
const char* password = "<password>";
```
Here you have to replace <ssid> with the SSID of your WIFI and <password> with the password of ypur WIFI.

And this place in the code:


```
IPAddress staticIP(192,168,0,60);
IPAddress gateway(192,168,0,1);
IPAddress subnet(255,255,255,0);
```
Here you can set the static IP address, the gateway and the subnet the SIGNALESP is going to have.

After changing these places in code you can compile the software and transfer it to your ESP8266.
