# Demo video script (40 s)

Captions for the screen, and an optional voice-over (~70 words, an easy pace for 40 s). The times assume the shots
come in this order: move the lines to where the shots are in the video.

| Time | Shot | Caption | Voice-over |
| --- | --- | --- | --- |
| 0:00–0:04 | the unit on the table | **XIAO STM32C5 + a CAN motor** | This is a XIAO STM32C5 riding on a BLDC motor. |
| 0:04–0:11 | a hand turns the bottom back and forth, the top stays | **Turn the bottom: the top stays put** | However I turn the bottom, the top keeps facing the same way. |
| 0:11–0:18 | picking it up, tilting, carrying it | **Tilt it, carry it** | The XIAO's on-board IMU tracks its orientation in 3D… |
| 0:18–0:25 | sideways, upside down | **Sideways. Upside down.** | …so it works tilted, sideways, even upside down. |
| 0:25–0:32 | close-up of the XIAO and the wires | **IMU + CAN transceiver on board** · **1 kHz loop, all on the XIAO** | Every millisecond it reads the gyro and sends the motor a new target over CAN, through the transceiver on the board. |
| 0:32–0:36 | wide shot, nothing plugged in but the battery | **Standalone on a 2S LiPo** | No computer, just a small battery. |
| 0:36–0:40 | end card | **can-do-gyro** · github.com/cpietsch/can-do-gyro | The code is on GitHub. |

Post text:

> A self-righting platform with the Seeed Studio XIAO STM32C5: its on-board IMU tracks the orientation, and over CAN
> it drives an M5Stack RollerCAN so the top always faces the same way. Zephyr, open source:
> github.com/cpietsch/can-do-gyro #XIAO #STM32 #Zephyr #CANbus
