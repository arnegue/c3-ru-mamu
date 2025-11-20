# Setup
## Setup USBIPD
### Windows Powershell (as admin)#
#### Search device to use
``usbipd list``

#### Bind
``usbipd bind --busid <bus_id>``

#### Attach
``usbipd bind --wsl --busid <bus_id>``

### In WSL
``sudo chmod 666 /dev/ttyUSB<id>``


# Deployment
## Build
``cargo build``

## Flash
``espflash flash target/riscv32imc-esp-espidf/debug/c3-ru-mamu --monitor``

## Monitor only`
``espflash monitor``

(press Ctrl+R) to reset

# Architecture

There are many Interfaces taken over here: 4 UART interfaces (one might be NMEA0183 with RS232, NMEA0183 with RS422, Seatalk. Three might be a TWI (CAN) used for NMEA2000/SeatalkNG) and the data might be transmitted via WiFi. How to do that:

* Each interface gets one FreeRTOS-Task for receiving/transmitting data.
* These interfaces forward the data to their assigned protocol tasks (may be more than one)
* Theses protocol tasks translate the data and store them in the ship data base

In this project it looks like this:

```
Interface-Tasks | Translation-Tasks |      DataBase
_______________________________________________________
                ┌──[NMEA1 Task ]──┐
[SPI Task]──────│  [NMEA2 Task ]  │
                │  [NMEA3 Task ]  │───[Ship DataBase]
                └──[Setalk Task]  │
[TWI Task]─────────[NMEA2K Task]  │
[WiFi Task]────────[NMEA4 Task ]──┘
```

## Handling of one Interface Task to many Translation tasks
This pattern solves:

* Short Interrupt service routine
* SPI peripheral is only accessed by one task
* Zero mutex use inside ISR

```
                  ┌────────────────┐
          IRQ --->│   SPI ISR      │   (very fast)
                  └──────┬─────────┘
                         │ signals
                         ▼
                    ┌──────────┐
                    │ SPI Task │   (reads SPI FIFO / registers)
                    └────┬─────┘
                         │ distributes data
                         ▼
    ┌─────────────┬──────────────┬──────────────┐
    ▼             ▼              ▼              ▼
NMEA1 Task   NMEA2 Task     NMEA3 Task     Seatalk-task
```