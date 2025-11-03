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
