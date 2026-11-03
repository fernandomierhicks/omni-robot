# Robot computer and cameras (compute_choice.py)

Reference CPU: i7-14700K (WSL) (GB6 single 2921, 7-Zip single 7412 MIPS).
First pass (Python glue, the two scored steps summed): measured 1.83 (Python 0.88) + extras (est) 0.82 = 2.65 reference cores.
Modes (py): MANUAL 1.69 measured + 0.31 est = 2.00; PATROL 2.04 measured + 0.51 est = 2.55; CATCH 1.89 measured + 0.51 est = 2.40; peak 2.55 (PATROL); camera head 0.57, the rest 2.32.
Modes (cpp): MANUAL 1.12 measured + 0.31 est = 1.43; PATROL 1.50 measured + 0.51 est = 2.01; CATCH 1.40 measured + 0.51 est = 1.91; peak 2.01 (PATROL); camera head 0.55, the rest 1.80.
Modes (lean): MANUAL 1.11 measured + 0.31 est = 1.42; PATROL 1.30 measured + 0.51 est = 1.81; CATCH 1.13 measured + 0.51 est = 1.64; peak 1.81 (PATROL); camera head 0.52, the rest 1.59.
Modes (lean_rerun): MANUAL 1.17 measured + 0.31 est = 1.48; PATROL 1.31 measured + 0.51 est = 1.82; CATCH 1.15 measured + 0.51 est = 1.66; peak 1.82 (PATROL); camera head 0.45, the rest 1.59.
Modes (sleep): MANUAL 0.95 measured + 0.31 est = 1.26; PATROL 1.07 measured + 0.51 est = 1.58; CATCH 1.12 measured + 0.51 est = 1.63; peak 1.63 (CATCH); camera head 0.52, the rest 1.44.
Modes (compose): MANUAL 0.78 measured + 0.31 est = 1.09; PATROL 0.90 measured + 0.51 est = 1.41; CATCH 1.39 measured + 0.51 est = 1.90; peak 1.90 (CATCH); camera head 0.54, the rest 1.36.
Modes (compose_all): MANUAL 0.70 measured + 0.31 est = 1.01; PATROL 0.85 measured + 0.51 est = 1.36; CATCH 1.16 measured + 0.51 est = 1.67; peak 1.67 (CATCH); camera head 0.55, the rest 1.23.
Modes (catch_cpp): MANUAL 0.57 measured + 0.31 est = 0.88; PATROL 0.76 measured + 0.51 est = 1.27; CATCH 0.65 measured + 0.51 est = 1.16; peak 1.27 (PATROL); camera head 0.33, the rest 1.17.

## Computers (CPU: cores needed of the board's, 75% cap)

| Computer | $ | Cores | first pass | C++ glue | C++ glue + lean Nav2 | + catch asleep outside CATCH | + Nav2 composed | + catch nodes in C++ | Ubuntu 24.04 | CSI |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Raspberry Pi 5 2GB | 77.5 | 4.0 | FAIL 6.2-8.6 cores | FAIL 4.7-6.5 cores | FAIL 4.2-5.8 cores | FAIL 3.8-5.3 cores | FAIL 4.4-6.1 cores | UNKNOWN 2.9-4.1 cores | yes | 2 |
| RUBIK Pi 3 (QCS6490) | 99.0 | 5.4 | FAIL 5.1-5.1 cores | PASS 3.9-3.9 cores | PASS 3.5-3.5 cores | PASS 3.1-3.1 cores | PASS 3.6-3.6 cores | PASS 2.4-2.4 cores | partly | 2 |
| Raspberry Pi 5 4GB | 130.0 | 4.0 | FAIL 6.2-8.6 cores | FAIL 4.7-6.5 cores | FAIL 4.2-5.8 cores | FAIL 3.8-5.3 cores | FAIL 4.4-6.1 cores | UNKNOWN 2.9-4.1 cores | yes | 2 |
| Intel N150 mini PC, 16 GB | 156.87 | 4.0 | FAIL 4.8-6.0 cores | FAIL 3.6-4.5 cores | FAIL 3.2-4.1 cores | UNKNOWN 2.9-3.7 cores | FAIL 3.4-4.3 cores | PASS 2.3-2.9 cores | yes | 0 |
| Raspberry Pi 5 8GB | 180.0 | 4.0 | FAIL 6.2-8.6 cores | FAIL 4.7-6.5 cores | FAIL 4.2-5.8 cores | FAIL 3.8-5.3 cores | FAIL 4.4-6.1 cores | UNKNOWN 2.9-4.1 cores | yes | 2 |
| ODROID-H4 Ultra | 220.0 | 8.0 | PASS 4.5-4.5 cores | PASS 3.4-3.4 cores | PASS 3.0-3.0 cores | PASS 2.7-2.7 cores | PASS 3.2-3.2 cores | PASS 2.1-2.1 cores | yes | 0 |
| ROCK 5B+ 8GB | 251.0 | 5.4 | FAIL 6.3-9.1 cores | FAIL 4.7-6.9 cores | FAIL 4.3-6.2 cores | UNKNOWN 3.8-5.6 cores | FAIL 4.5-6.5 cores | UNKNOWN 3.0-4.4 cores | partly | ? |
| Raspberry Pi 5 16GB | 305.0 | 4.0 | FAIL 6.2-8.6 cores | FAIL 4.7-6.5 cores | FAIL 4.2-5.8 cores | FAIL 3.8-5.3 cores | FAIL 4.4-6.1 cores | UNKNOWN 2.9-4.1 cores | yes | 2 |
| LattePanda IOTA 8GB / 64GB | 339.0 | 4.0 | FAIL 4.8-6.2 cores | FAIL 3.6-4.7 cores | FAIL 3.2-4.2 cores | UNKNOWN 2.9-3.8 cores | FAIL 3.4-4.4 cores | PASS 2.3-3.0 cores | yes | 0 |
| Jetson Orin Nano 4GB module | 349.0 | 6.0 | UNKNOWN  | UNKNOWN  | UNKNOWN  | UNKNOWN  | UNKNOWN  | UNKNOWN  | yes | ? |
| M8, Ryzen 5 PRO 6650H, 16 GB + 512 GB | 389.99 | 6.0 | PASS 4.0-4.0 cores | PASS 3.0-3.0 cores | PASS 2.7-2.7 cores | PASS 2.5-2.5 cores | PASS 2.9-2.9 cores | PASS 1.9-1.9 cores | yes | 0 |
| NUC 14 Pro barebone, Core Ultra 5 125H | 394.0 | 7.5 | PASS 3.4-3.4 cores | PASS 2.6-2.6 cores | PASS 2.3-2.3 cores | PASS 2.1-2.1 cores | PASS 2.4-2.4 cores | PASS 1.6-1.6 cores | yes | 0 |
| Jetson Orin Nano Super Developer Kit (8 GB) | 399.0 | 6.0 | FAIL 8.0-9.6 cores | FAIL 6.1-7.3 cores | FAIL 5.4-6.5 cores | FAIL 4.9-5.9 cores | FAIL 5.7-6.8 cores | UNKNOWN 3.8-4.6 cores | yes | 2 |
| Jetson Orin Nano 8GB module | 399.0 | 6.0 | FAIL 8.0-9.6 cores | FAIL 6.1-7.3 cores | FAIL 5.4-6.5 cores | FAIL 4.9-5.9 cores | FAIL 5.7-6.8 cores | UNKNOWN 3.8-4.6 cores | yes | ? |
| Ryzen 7 8845HS mini PC (barebone) | 399.79 | 8.0 | PASS 3.2-3.2 cores | PASS 2.4-2.4 cores | PASS 2.2-2.2 cores | PASS 1.9-1.9 cores | PASS 2.3-2.3 cores | PASS 1.5-1.5 cores | yes | 0 |
| NucBox M6 Ultra, Ryzen 5 7640HS, 16 GB + 512 GB | 499.99 | 6.0 | PASS 3.1-3.1 cores | PASS 2.4-2.4 cores | PASS 2.1-2.1 cores | PASS 1.9-1.9 cores | PASS 2.2-2.2 cores | PASS 1.5-1.5 cores | yes | 0 |
| Jetson Orin NX 8GB module | 649.0 | 6.0 | UNKNOWN  | UNKNOWN  | UNKNOWN  | UNKNOWN  | UNKNOWN  | UNKNOWN  | yes | ? |
| MS-03 barebone, Core Ultra 5 336H (Panther Lake) | 759.0 | ? | UNKNOWN  | UNKNOWN  | UNKNOWN  | UNKNOWN  | UNKNOWN  | UNKNOWN  | ? | 0 |
| Jetson Orin NX 16GB module | 999.0 | 8.0 | FAIL 7.2-8.8 cores | UNKNOWN 5.4-6.7 cores | UNKNOWN 4.9-6.0 cores | PASS 4.4-5.4 cores | UNKNOWN 5.1-6.3 cores | PASS 3.4-4.2 cores | yes | ? |
| Jetson AGX Orin 64GB Developer Kit | 3499.0 | 12.0 | PASS 6.2-6.2 cores | PASS 4.7-4.7 cores | PASS 4.2-4.2 cores | PASS 3.8-3.8 cores | PASS 4.4-4.4 cores | PASS 3.0-3.0 cores | yes | ? |
| Jetson AGX Thor Developer Kit | 5499.0 | ? | UNKNOWN  | UNKNOWN  | UNKNOWN  | UNKNOWN  | UNKNOWN  | UNKNOWN  | yes | ? |
| Jetson Orin Nano 2 (announced) | ? | 8.0 | UNKNOWN  | UNKNOWN  | UNKNOWN  | UNKNOWN  | UNKNOWN  | UNKNOWN  | ? | ? |

## Cameras

| Camera | $ | Verdict | Checks |
| --- | --- | --- | --- |
| 2.3MP x2 AR0234 color global-shutter synchronized stereo kit for Jetson | 259.99 | UNKNOWN | global shutter PASS; color PASS; frame rate PASS; hardware sync PASS; latency UNKNOWN |
| 2.3MP x2 AR0234 color global-shutter synchronized stereo kit for Raspberry Pi | ? | UNKNOWN | global shutter PASS; color PASS; frame rate UNKNOWN; hardware sync PASS; latency UNKNOWN |
| AR0234 2.3MP global-shutter camera with onboard ISP (xISP) for Orin Nano / NX | ? | FAIL | global shutter PASS; color PASS; frame rate UNKNOWN; hardware sync FAIL (no); latency UNKNOWN |
| 1MP x4 OV9782 color global-shutter quad kit (Camarray HAT) | ? | FAIL | global shutter PASS; color PASS; frame rate FAIL (45.0); hardware sync PASS; latency UNKNOWN |
| 1MP OV9782 color global-shutter USB camera (UVC, M12 low-distortion lens) | ? | UNKNOWN | global shutter PASS; color PASS; frame rate PASS; hardware sync PASS; latency UNKNOWN |
| 1MP OV9281 mono global-shutter camera for Raspberry Pi | 25.99 | FAIL | global shutter PASS; color FAIL (mono); frame rate PASS; hardware sync UNKNOWN (partly); latency UNKNOWN |
| 1MP x2 OV9281 mono global-shutter stereo kit | 164.99 | FAIL | global shutter PASS; color FAIL (mono); frame rate UNKNOWN; hardware sync PASS; latency UNKNOWN |
| 5MP OG05B10 color global-shutter camera for Raspberry Pi | 149.99 | FAIL | global shutter PASS; color PASS; frame rate FAIL (30.0); hardware sync UNKNOWN (VERIFY); latency UNKNOWN |
| AR0234 Full HD color global-shutter camera for Jetson Orin NX / Orin Nano | ? | UNKNOWN | global shutter PASS; color PASS; frame rate PASS; hardware sync PASS; latency UNKNOWN |
| Raspberry Pi Global Shutter Camera (IMX296) | 50.0 | UNKNOWN | global shutter PASS; color PASS; frame rate PASS; hardware sync PASS; latency UNKNOWN |
| OAK-FFC OV9782 module | 94.99 | UNKNOWN | global shutter PASS; color PASS; frame rate PASS; hardware sync PASS; latency UNKNOWN |
| OAK 4 D (RVC4) | 849.0 | UNKNOWN | global shutter PASS; color UNKNOWN (mono_stereo_color_center); frame rate UNKNOWN; hardware sync UNKNOWN; latency UNKNOWN |
| ZED X One GS | 399.0 | UNKNOWN | global shutter PASS; color PASS; frame rate PASS; hardware sync UNKNOWN (VERIFY); latency UNKNOWN |
| Gemini 335L | 359.0 | UNKNOWN | global shutter PASS; color UNKNOWN (mono_stereo_color_center); frame rate PASS; hardware sync UNKNOWN (VERIFY); latency UNKNOWN |
| RealSense D455 | ? | UNKNOWN | global shutter PASS; color UNKNOWN (mono_stereo_color_center); frame rate UNKNOWN; hardware sync UNKNOWN (VERIFY); latency UNKNOWN |

## Systems (computer + cameras)

| System | $ (known parts + est extras) | CPU, first pass | CPU, C++ glue | CPU, C++ glue + lean Nav2 | CPU, + catch asleep outside CATCH | CPU, + Nav2 composed | CPU, + catch nodes in C++ | Cameras | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| GMKtec M8 (6650H) + Pi 5 2GB camera head + 2 Pi GS cameras | 642 | PASS (3.1-3.1 cores; head 1.3-1.9 cores) | PASS (2.7-2.7 cores; head 1.3-1.8 cores) | PASS (2.4-2.4 cores; head 1.2-1.7 cores) | PASS (2.2-2.2 cores; head 1.2-1.7 cores) | PASS (2.1-2.1 cores; head 1.2-1.7 cores) | PASS (1.8-1.8 cores; head 0.8-1.1 cores) | UNKNOWN | budget: both computers $467; lenses, cooler, buck |
| GMKtec M6 Ultra (7640HS) + Pi 5 2GB camera head + 2 Pi GS cameras | 752 | PASS (2.5-2.5 cores; head 1.3-1.9 cores) | PASS (2.1-2.1 cores; head 1.3-1.8 cores) | PASS (1.9-1.9 cores; head 1.2-1.7 cores) | PASS (1.7-1.7 cores; head 1.2-1.7 cores) | PASS (1.6-1.6 cores; head 1.2-1.7 cores) | PASS (1.4-1.4 cores; head 0.8-1.1 cores) | UNKNOWN | lenses, cooler, buck |
| GMKtec M8 (6650H) + Pi 5 4GB camera head (in stock) + 2 Pi GS cameras | 695 | PASS (3.1-3.1 cores; head 1.3-1.9 cores) | PASS (2.7-2.7 cores; head 1.3-1.8 cores) | PASS (2.4-2.4 cores; head 1.2-1.7 cores) | PASS (2.2-2.2 cores; head 1.2-1.7 cores) | PASS (2.1-2.1 cores; head 1.2-1.7 cores) | PASS (1.8-1.8 cores; head 0.8-1.1 cores) | UNKNOWN | the in-stock Pi (Adafruit) |
| Orin Nano Super kit + 2 e-con AR0234 (trigger) | 399 + econ-ecam25-cuonx | FAIL (8.0-9.6 cores) | FAIL (6.1-7.3 cores) | FAIL (5.4-6.5 cores) | FAIL (4.9-5.9 cores) | FAIL (5.7-6.8 cores) | UNKNOWN (3.8-4.6 cores) | UNKNOWN | cameras' price on request |
| Orin Nano Super kit + Arducam AR0234 sync stereo | 659 | FAIL (8.0-9.6 cores) | FAIL (6.1-7.3 cores) | FAIL (5.4-6.5 cores) | FAIL (4.9-5.9 cores) | FAIL (5.7-6.8 cores) | UNKNOWN (3.8-4.6 cores) | UNKNOWN | trigger VERIFY on this kit |
| Orin NX 16GB on a J401 + 2 e-con AR0234 | 1304 + econ-ecam25-cuonx | FAIL (7.2-8.8 cores) | UNKNOWN (5.4-6.7 cores) | UNKNOWN (4.9-6.0 cores) | PASS (4.4-5.4 cores) | UNKNOWN (5.1-6.3 cores) | PASS (3.4-4.2 cores) | UNKNOWN | + NVMe and cooler (est $60) |
| Ryzen 7 8845HS mini PC + OAK-FFC 4P + 2 OV9782 | 999 | PASS (3.2-3.2 cores) | PASS (2.4-2.4 cores) | PASS (2.2-2.2 cores) | PASS (1.9-1.9 cores) | PASS (2.3-2.3 cores) | PASS (1.5-1.5 cores) | PASS | + 32 GB DDR5 and an SSD (est $200) |
| Core Ultra 5 NUC 14 Pro + OAK-FFC 4P + 2 OV9782 | 993 | PASS (3.4-3.4 cores) | PASS (2.6-2.6 cores) | PASS (2.3-2.3 cores) | PASS (2.1-2.1 cores) | PASS (2.4-2.4 cores) | PASS (1.6-1.6 cores) | PASS | + RAM and SSD (est $200); launch price |
| N150 mini PC 16GB + OAK-FFC 4P + 2 OV9782 | 556 | FAIL (4.8-6.0 cores) | FAIL (3.6-4.5 cores) | FAIL (3.2-4.1 cores) | UNKNOWN (2.9-3.7 cores) | FAIL (3.4-4.3 cores) | PASS (2.3-2.9 cores) | PASS |  |
| Pi 5 16GB + 2 Pi Global Shutter cameras (XTR) | 455 | FAIL (6.2-8.6 cores) | FAIL (4.7-6.5 cores) | FAIL (4.2-5.8 cores) | FAIL (3.8-5.3 cores) | FAIL (4.4-6.1 cores) | UNKNOWN (2.9-4.1 cores) | UNKNOWN | + two C/CS lenses (est $50) |
| Orin Nano Super kit + OAK-FFC 4P + 2 OV9782 | 798 | FAIL (8.0-9.6 cores) | FAIL (6.1-7.3 cores) | FAIL (5.4-6.5 cores) | FAIL (4.9-5.9 cores) | FAIL (5.7-6.8 cores) | UNKNOWN (3.8-4.6 cores) | PASS | the OAK does capture and ISP; CPU as the kit |

Camera head (a Pi 5 with the cameras): catch_perception, camera capture, stamping, Bayer ROI at 2 x 120 fps, AprilTag detection + tag_slam.
Raspberry Pi 5s for the whole stack at 75% of 4 cores each, if it split perfectly: first pass 3-3; C++ glue 2-3; C++ glue + lean Nav2 2-2; + catch asleep outside CATCH 2-2; + Nav2 composed 2-3; + catch nodes in C++ 1-2.

Extras (est, reference cores): operator console + face node 0.11; voice: Kokoro bursts + Vosk listening 0.15; slam_toolbox mapping instead of localization 0.2; camera capture, stamping, Bayer ROI at 2 x 120 fps 0.2; lidar driver 0.03; AprilTag detection + tag_slam 0.1; frames, misc 0.03. The first pass adds them all; the modes add what each mode would run (MODE_EXTRAS).
