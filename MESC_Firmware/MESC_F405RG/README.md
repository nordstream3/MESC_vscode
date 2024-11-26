
1) In stm32cubemx, choose "stm32cubeide" to generate "STM32F405RGTX_FLASH.ld". (Remove *RAM.ld)
2) Then change to "cmake" and generate code again
3) Remember to put in NVM tags from previous "STM32F405RGTX_FLASH.ld"

# Terminal
sudo chmod a+rw /dev/ttyACM0;putty -serial /dev/ttyACM0

# Clean
cd build
rm -r *

# Configure
cd build
make ..

# Build
cd build
cmake --build . -- VERBOSE=1

# Create binary from .elf
arm-none-eabi-objcopy -O binary MESC_F405RG.elf MESC_F405RG.bin


# stm32cubeide have a few extra compiler and linker flags not yet in this vscode version:
# compiler flags:
    -fsingle-precision-constant --specs=nano.specs -mthumb
# linker flags:
    --specs=nosys.specs -static
Don't know if these make any difference.




# USING MESC:

# Erase flash
save -d

# Measure
status start
set curr_max 40
set curr_min -40
save
measure -r
measure -f
save
set uart_req 3
set uart_req 0

# Field weakening
set FW_curr 10
set uart_req 3

# Use HFI
set hfi_type 1
set SL_sensor 3
set uart_req 3

# Fast-loop log
log -fl


# Plotting log in gnuplot
. ../gnuplot/plot.sh '<json.data>' <col_begin>:<col_end>

# TTerm
https://github.com/TMaxElectronics/TTerm


# Testing lowest speed
set curr_max 20
set curr_min -20
set speed_ki 0.04
set speed_kp 0.02
set flux 0.0049
set hfi_volt 12
set control_mode 1

set speed_req 300
set speed_req 0
