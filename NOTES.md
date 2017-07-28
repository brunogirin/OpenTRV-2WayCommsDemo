# 2 Way Demo Implementation Notes
Based on the COHEATM2 config.

Hub and TRV are split into different source files and secure comms are removed.
- The hub firmware (OT2WayGateway.ino) should be used with a REV8 (deved used on a REV10 without issue though)/
- The TRV firmware (OT2WayTRV.ino) should be used with a REV7/TRV1.5.
    - Make sure to set per valve house-codes on line 61 before compilation.

## Git Repo Tags
Tag: 20170728-2WayCommsDemo
Repos:
- OpenTRV-2WayCommsDemo
- OTRadioLink
- OTProtocolCC

## Radio Protocol:
- What channel?
- Both the hub and the TRV run the radio in RX mode continuously. It is not recommended to run them on batteries for long periods of time.
- Uses the OTProtocolCC protocol with the following alterations:
    - The rad-open-percentage (rp) field for poll and response messages are used to send the set-point in centigrade.
    - The rp field is constrained by the TRV to the range [15,30]. Any value above or below this range will be held at the maximum or minimum respectively.
    - The TRV will periodically send a "response" frame to the gateway containing the current temperature and humidity of the room.
    
## Serial Protocol:
This demo repurposes OTProtocolCC for 2 way communications with remote control of TRV set-point and regular TRV telemetry.
- Set point is controlled via the CC1PollAndCommand model.
```
//    If CC1 hub then allow +CC1 ? command to poll a remote relay.
//    Full command is:
//        +CC1 ? hc1 hc2 rp lc lt lf
//    i.e. six numeric arguments, see below, with out-of-range values coerced (other than housecodes):
//                Factory method to create instance.
//                Invalid parameters (except house codes) will be coerced into range.
//                  * House code (hc1, hc2) of valve controller that the poll/command is being sent to.
//                  * rad-set-point [15,30] in 1 C steps. The set-point the TRV will attempt to keep the room at. // TODO
//                                  Originally [0,100] 0-100 in 1% steps, percent open approx to set rad valve.
//                  * light-colour  [0,3] bit flags 1==red 2==green (lc) 0 => stop everything. This field is ignored.
//                  * light-on-time [1,15] (0 not allowed) 30-450s in units of 30s (lt). This field is ignored.
//                  * light-flash   [1,3] (0 not allowed) 1==single 2==double 3==on (lf). This field is ignored.
//                Returns instance; check isValid().
//                static CC1PollAndCommand make(uint8_t hc1, uint8_t hc2,
//                                              uint8_t rp,
//                                              uint8_t lc, uint8_t lt, uint8_t lf);
//    e.g. +CC1 ? 70 04 20 1 1 1
//        - Send a poll and command (+CC1 ?) to the valve with house code "70 04".
//        - Set the valve set-point to 20 C.
//        - The last three values are ignored but are preserved to minimise coding changes.
```
- The TRV sends a poll response once a minute and upon reception of a poll and command frame.
```
Respond to the hub with sensor data.
Can use read() for very freshest values at risk of some delay/cost.
Full response is:
    +CC1 * hc1 hc2 rh tp tr al s w sy
* House code (hc1, hc2) of valve controller that the poll response is being sent from.
* rh: Relative humidity, scaled from [0,100] to [0,50] i.e. 2% per count.
* tp: Ignored. Secondary temperature sensor.
* tr: Temperature reading in range [0,199] in 1/16ths of a C.
* al: Ignored. Ambient light.
* s:  Ignored. Switch toggle state.
* e:  Ignored.
* sy: Valve run state. Returns true if the valve motor driver is running normally.

As for poll and command packet, unused fields have been left as they are to minimise coding changes.
```

## TODO
- Add randomness to regular radio TX (not sure if it already does this in the radio/OTProtocolCC libraries).
- Switch to proper poll-response model. Probably TRV polls and hub responds to reduce TRV power consumption.
- Clean up serial protocol.
- Make house codes settable via the CLI. Currently hardcoded in ```housecode1``` and ```housecode2```.
