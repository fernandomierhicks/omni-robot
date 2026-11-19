| Lead | From | To | Through | mm | Carries | Cable |
| --- | --- | --- | --- | --- | --- | --- |
| BAT | battery | bat_deck (battery) | not routed: battery removal (v15): robot on its stand, unplug the XT90 under the deck, open the straps, the pack drops out |  | 6S pack (18-25.2 V) in: the pack, its pigtail, the XT90 pair under the deck, the board lead up through the deck's lead hole | the pack's own XT90 pigtail, 2 x 10 AWG (HRB 6S 3300: VERIFY connector, gauge and length) |
| BAT_AS | bat_deck (battery) | as_in (anti_spark) | floor 1 | 137 | 6S in: the pack's lead from the deck's lead hole to the anti-spark switch | 2 x 12 AWG silicone, twisted |
| AS_INA | as_out (anti_spark) | ina226_in (xt30_block) | floor 1 | 164 | 6S switched, through the 30 A inline fuse (blade holder on this lead) | 2 x 12 AWG silicone, twisted |
| INA_BLK | ina226 IN- | block bus | not routed: inside the block (the INA226 stands on its side) |  | 6S metered, into the distribution block |  |
| BAL | battery |  | not routed: stays with the pack; its XH7 tucked against the pack's front end |  | 6S balance lead (charging only) |  |
| M_FR | motor_FR (pod FR) | blk_m2 (xt30_block) | floor 1 | 178 | drive bus 6S + CAN1 in / out (the chain runs through every drive lead) | 2 x 18 AWG silicone (bus +/-) + 2 twisted pairs 26 AWG (CAN in, CAN out) in braided sleeve |
| M_FL | motor_FL (pod FL) | blk_m1 (xt30_block) | floor 1 | 256 | drive bus 6S + CAN1 in / out (the chain runs through every drive lead) | 2 x 18 AWG silicone (bus +/-) + 2 twisted pairs 26 AWG (CAN in, CAN out) in braided sleeve |
| M_RR | motor_RR (pod RR) | blk_m4 (xt30_block) | floor 1 | 117 | drive bus 6S + CAN1 in / out (the chain runs through every drive lead) | 2 x 18 AWG silicone (bus +/-) + 2 twisted pairs 26 AWG (CAN in, CAN out) in braided sleeve |
| M_RL | motor_RL (pod RL) | blk_m3 (xt30_block) | floor 1 | 308 | drive bus 6S + CAN1 in / out (the chain runs through every drive lead) | 2 x 18 AWG silicone (bus +/-) + 2 twisted pairs 26 AWG (CAN in, CAN out) in braided sleeve |
| BLK_12 | blk_12 (xt30_block) | c12v_in (conv_12v) | floor 1 | 306 | 6S to the 12 V converter | 2 x 18 AWG silicone, twisted |
| BLK_5 | blk_5 (xt30_block) | c5v_in (conv_5v) | floor 1 | 216 | 6S to the 5 V converter | 2 x 20 AWG silicone, twisted |
| BLK_19 | blk_19 (xt30_block) | c19v_in (conv_19v) | floor 1 -> slot R (in the side fin's shadow) -> floor 2 | 266 | 6S up to the 19 V converter (second floor) | 2 x 18 AWG silicone, twisted |
| SIG_DOWN | blk_sense (xt30_block) | carrier_J1 (carrier) | floor 1 -> slot R (in the side fin's shadow) -> floor 2 | 424 | one signal lead, carrier <-> distribution block: pack sense (+, GND) to J1, CAN1 H / L to J6 (U2's 120 ohm is this end of the chain), the INA226's I2C to J21; short breakouts at both ends | 8 x 26 AWG in a jacket: CAN H / L (a twisted pair), pack sense +/-, I2C 3V3 / GND / SDA / SCL |
| BUTTON | switch_main (switch_main) | as_btn (anti_spark) | floor 1 -> slot L (in the side fin's shadow) -> floor 2 | 539 | the main button (latching = off / e-stop) to the anti-spark switch's control input | 2 x 24 AWG, jacketed (the main button to the anti-spark switch) |
| C12_OUT | c12v_out (conv_12v) | ina3221_1_in (ina3221) | floor 1 -> slot L (in the side fin's shadow) -> floor 2 | 549 | 12 V out, up to the INA3221 CH1 | 2 x 18 AWG silicone, twisted |
| C5_OUT | c5v_out (conv_5v) | ina3221_3_in (ina3221) | floor 1 -> slot R (in the side fin's shadow) -> floor 2 | 261 | 5 V out, up to the INA3221 CH3 | 2 x 20 AWG silicone, twisted |
| C19_OUT | c19v_out (conv_19v) | ina3221_2_in (ina3221) | floor 2 | 112 | 19 V out to the INA3221 CH2 | 2 x 18 AWG silicone, twisted |
| R12 | ina3221_1_out (ina3221) | sa_dc (servo_adapter) | floor 2 | 330 | 12 V to the servo bus adapter (the SO-101) | 2 x 18 AWG silicone, twisted |
| R19 | ina3221_2_out (ina3221) | pc_dc (computer) | floor 2 | 144 | 19 V to the computer | 2 x 18 AWG silicone, twisted |
| R5_DISP | ina3221_3_out (ina3221) | disp_pwr (display) | floor 2 | 251 | 5 V to the face display and the speaker amp (branch at the crown) | 2 x 20 AWG silicone, twisted |
| R5_HUB | ina3221_3_out (ina3221) | hub_pwr (usb_hub) | floor 2 | 283 | 5 V to the powered USB hub | 2 x 20 AWG silicone, twisted |
| SBUS | servo_adapter | SO-101 base servo | not routed: arm kit: inside the SO-101, routed by its own clips |  | Feetech TTL half-duplex bus + 12 V |  |
| CAN1 | carrier_J6 (carrier) | blk_can (xt30_block) | not routed: XH2 both ends |  | CAN1 H / L to the drives' chain: in SIG_DOWN (its breakouts) |  |
| I2C_226 | carrier_J21 (carrier) | ina226_i2c (xt30_block) | not routed: XH4 both ends |  | I2C to the INA226 (robot total current): in SIG_DOWN (its breakouts) |  |
| I2C_3221 | carrier_J12 (carrier) | ina3221_i2c (ina3221) | floor 2 | 273 | I2C to the INA3221 (12 V, 19 V, 5 V) | 4 x 26 AWG in a jacket (3V3, GND, SDA, SCL; or 5 V, GND, TX, RX) |
| IMU | carrier_J3 (carrier) | imu (imu) | floor 2 | 65 | BNO085: J3 data (SCL/SCK, SDA/MISO, AD0/MOSI, CS, INT, RST) + J4 power, in one jacket | 8 x 26 AWG in a jacket (the BNO085: J4 power + J3 data) |
| TEENSY | teensy_usb (carrier) | pc_usb3_2 (computer) | floor 2 | 183 | USB 2 (Teensy native CDC; also the Teensy's power) | USB 2 lead |
| D435 | d435 (d435) | pc_usb3_1 (computer) | floor 2 | 87 | USB 3 (5 Gb/s) | USB 3 lead (the D435 lead is drawn Ø4.5, VERIFY) |
| SO101 | sa_usb (servo_adapter) | hub_d1 (usb_hub) | floor 2 | 138 | USB (servo bus adapter), through the hub | USB-C lead |
| HUB | hub_up (usb_hub) | pc_usbc (computer) | floor 2 | 224 | USB 2 uplink of the hub | USB-C lead |
| LIDAR | lidar (lidar) | lidar_ad_in (lidar_adapter) | floor 1 -> slot R (in the side fin's shadow) -> floor 2 | 290 | A2M8 UART + 5 V (its own lead) | the A2M8's own lead to its USB adapter (VERIFY) |
| LIDARUSB | lidar_ad_usb (lidar_adapter) | pc_usb2_2 (computer) | floor 2 | 274 | USB 2 (A2M8 adapter) | USB 2 lead |
| HDMI | pc_hdmi (computer) | disp_hdmi (display) | floor 2 | 212 | HDMI video to the 5 in face | flat FPC HDMI cable (a round HDMI lead is Ø7, R 35: too stiff here), ASSUMED |
| MIC | mic_usb (mic_array) | pc_usb2_1 (computer) | floor 2 | 383 | USB audio: 4-mic array in, playback out to the amp (AEC reference) | USB 2 lead |
| SPK | mic array audio out | speaker | not routed: on the crown (amp beside the speaker, not drawn) |  | 3.5 mm line out -> class-D amp (PAM8302 class, ASSUMED) -> 4 ohm 3 W speaker |  |
| STATUS | carrier_J15 / J20 | RGB LED, NeoPixel strip | not routed: not placed yet: where the face shows them (open) |  | RGB status LED, NeoPixel strip (Nov 18: the buzzer and the ELRS receiver are on the carrier itself, no leads) |  |

Lidar-plane crossings: 6, all in the side fins' shadows: slot R BLK_19, SIG_DOWN, C5_OUT, LIDAR; slot L BUTTON, C12_OUT.
