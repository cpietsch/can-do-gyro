# Demo video script (40 s)

Voice-over and on-screen captions. The first version (70 words) took 26 s at your pace, so this one has 103 words for
40 s. The times assume the shots come in this order; move the lines to where those shots are in the video.

| Time | Shot | Caption | Voice-over |
| --- | --- | --- | --- |
| 0:00–0:05 | the unit on the table | **XIAO STM32C5 + a CAN motor** | This is a XIAO STM32C5 riding on a brushless motor: a gimbal in one axis. |
| 0:05–0:10 | a hand turns the bottom back and forth, the top stays | **Turn the bottom: the top stays put** | However I turn the bottom, the top keeps facing the same way. |
| 0:10–0:16 | picking it up, tilting, carrying it | **Tilt it, carry it** | The XIAO's on-board IMU tracks its orientation in 3D, so I can tilt it, carry it around… |
| 0:16–0:19 | sideways, upside down | **Sideways. Upside down.** | …turn it sideways, or even upside down. |
| 0:19–0:27 | close-up of the XIAO and the wires | **IMU + CAN transceiver on board** · **1 kHz loop** | Every millisecond it reads the gyro and sends the motor a new target over CAN, through the transceiver right on the board. |
| 0:27–0:33 | wide shot, nothing plugged in but the battery | **M5Stack RollerCAN · standalone on a 2S LiPo** | The motor is an M5Stack RollerCAN on a small LiPo, and everything runs on the XIAO. |
| 0:33–0:37 | it holds while the bottom turns | **Idea: hubcaps that stand still** | Imagine hubcaps that stand still while the car drives. |
| 0:37–0:40 | end card | **can-do-gyro** · github.com/cpietsch/can-do-gyro | Code and wiring are on GitHub. |

## Voice-over, the whole text

This is a XIAO STM32C5 riding on a brushless motor: a gimbal in one axis. However I turn the bottom, the top keeps
facing the same way. The XIAO's on-board IMU tracks its orientation in 3D, so I can tilt it, carry it around, turn it
sideways, or even upside down. Every millisecond it reads the gyro and sends the motor a new target over CAN, through
the transceiver right on the board. The motor is an M5Stack RollerCAN on a small LiPo, and everything runs on the
XIAO. Imagine hubcaps that stand still while the car drives. Code and wiring are on GitHub.

## Post text

> A self-righting platform with the Seeed Studio XIAO STM32C5: its on-board IMU tracks the orientation, and over CAN
> it drives an M5Stack RollerCAN so the top always faces the same way. Zephyr, open source:
> github.com/cpietsch/can-do-gyro #XIAO #STM32 #Zephyr #CANbus
