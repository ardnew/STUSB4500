# STUSB4500
##### Arduino library for USB PD sink controller (STUSB4500)

----

###### Features include:
- [x] Works with native Arduino Wire (I²C) library
- [x] USB Power Delivery v2.0 and v3.0 compatible state machine
- [x] Designed for [STUSB4500 Compact Breakout](https://www.tindie.com/products/oxplot/stusb4500-compact-breakout/)
- [x] Can analyze and identify all available power profiles
- [x] Can request and use any arbitrary power profile +5-20V/0-5A in 50mV/10mA increments
   - Does not lose VBUS on power transition, maintains power output
- [x] Detect cable orientation (CC1/CC2 termination)
- [x] Uses ALRT/ATCH interrupt pins for efficient response
- [x] Completely enable/disable power output
- [x] Provides user callback support for:
   - Cable attach/detach events 
   - PD source capability discovery events
- [x] Set up to 3 custom power profiles for fallback
- [ ] Reformat NVM for default power profiles in standalone mode (not yet implemented)

[An example sketch](examples/basic-demo/basic-demo.ino) is included that demonstrates callback functionality, analyzing available source power profiles, and setting a custom power profile. 
