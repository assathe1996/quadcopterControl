# CMU 18776 Course Project
## Installation

- Copy this folder names 'lqr_att_controller' and 'observer' in 'Firmware/src/modules'.
- Add line 'modules/lqr_att_controller' and 'module/observer' in 'Firmware/boards/px4/sitl/default.cmake'.
- In some versions of px4, we may need to comment out lines for AUTO_DISARM_PREFLIGHT in 'Firmware/src/modules/commander/ commander.cpp.' This prevents auto disarming of the vehicle when we manually arm the vehicle. In version 1.9 comment out lines 1715 to 1718.
- Compile with make px4_sitl jmavsim.

## Usage
1. lqr_att_controller
2. observer
