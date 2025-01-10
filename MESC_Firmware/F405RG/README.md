
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
cmake ..

# Build
cd build
cmake --build . -- VERBOSE=1

# Create binary from .elf
arm-none-eabi-objcopy -O binary F405RG.elf F405RG.bin


# stm32cubeide have a few extra compiler and linker flags not yet in this vscode version:
# compiler flags:
    -fsingle-precision-constant --specs=nano.specs -mthumb
# linker flags:
    --specs=nosys.specs -static
Don't know if these make any difference.


# git stuff
git ls-files
git status --short
git add MESC_Firmware/.vscode/launch.json
git add MESC_Firmware/.vscode/settings.json
MESC_Firmware/Common/Inc/MESCfoc.h
git add MESC_Firmware/F405RG/.mxproject
git commit -m "Add all untracked files"
git config --global user.email xxx@yyy.zzz
git config --global user.name xxx
git commit -m "Add all untracked files"
git push origin main
git branch main
git config --global user.name "xxx"
git config --global user.email "jens_overby@yahoo.com"
git config -l
git push origin main
git config --global credential.helper cache
git pull
cd ..
exit
git clone https://github.com/nordstream3/MESC_vscode.git
cd MESC_vscode/
git status
git commit
git commit -am "Commit all modified files"
git status
git push


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
