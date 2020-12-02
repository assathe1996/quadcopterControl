# CMU 18776 Course Project
## Installation

- Copy the folders 'lqr_att_controller' and 'observer' in 'Firmware/src/modules'.
- Add line 'modules/lqr_att_controller' and 'module/observer' in 'Firmware/boards/px4/sitl/default.cmake'.
- In some versions of px4, we may need to comment out lines for AUTO_DISARM_PREFLIGHT in 'Firmware/src/modules/commander/ commander.cpp.' This prevents auto disarming of the vehicle when we manually arm the vehicle. In version 1.9 comment out lines 1715 to 1718.
- Compile with make px4_sitl jmavsim.

## Usage
1. lqr_att_controller
- Arm the vehicle using command: 'commander arm'.
- Land the vehicle using command: 'commander land'.
- Start the controller using command: 'lqr_att_controller start'.
- Stop the controller using command: 'lqr_att_controller stop'.
- Set height for the controller using command: 'lqr_att_controller set_height <value>'.  Note: the value should be negative as UP direction is negative in the simulator.
- Display current height using command: 'lqr_att_controller show_height'.
- Usage instructions will be visible by typing ‘lqr_att_controller’ in the command prompt.
2. observer
