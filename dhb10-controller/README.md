# DHB-10 Motor Controller

This module handles controlling and reading from the DHB-10 Motor controller.

## Relay Support
Work needs to be done here to allow relay to be optionally supported.

### Relay Wiring for ZynqRobotControllerv2 on Arlo Robot

Installing a relay on the Arlo robot resulted from issues faced with a required power-toggle of the DHB-10 motor controller after entering an error state. This error state trigger has not been determined though can be ignored with the installation of a relay in place of the manual switch that controls the power supply to the Arlo's motor controller.

<p><img src="https://github.com/smartsystemslab-uf/ROS-ArloRobot/blob/master/dhb10-controller/Resources/Relay%20Wiring%20for%20ZynqRobotControllerv2%20on%20Arlo%20Robot.jpg?raw=True"/> </p>

Relays can be added to the ZynqRobotControllerv2 setup as shown above.

### Installation
 - Remove the 4 screws holding the top plastic plate of the power distribution board. Remove the plate and attached switches.
 - Remove and store the switch in the slot labelled "Motors." There are clips on the sides of the switch to allow this to be removed.
 - Add headers or solder wires to the relay to allow connection to the pins, 3V, GND, and Signal as shown above. (USE DUPONT WIRES)
 - Solder 2 wires to the lower two terminals of the exposed motor controller switch terminals. (USE AT LEAST 20 GAUGE AWG WIRES)
 - Connect these two wires, in any order, to the Common and Normally Open ports of the relay.
 - Wire jumper wires to the Zybo. (See: [Targeted Wiring for Arlo Robot - ZynqRobotControllerv2](https://github.com/smartsystemslab-uf/ZynqRobotController/tree/master/FPGA/Zynq_Robot_Controller_v2#targeted-wiring-for-arlo-robot))
   - Relay Signal: Zybo PMPOD JE Pin 10
   - Relay GND: Zybo PMOD JE Pin 11
   - Relay 3V: Zybo PMOD JE Pin 12


### Testing
With the ZynqRobotControllerv2 design loaded on to a ZyboZ710, the GPIO tests found [HERE](https://github.com/smartsystemslab-uf/ZynqRobotController/blob/master/Software/UIO/GPIO_0_Low.cpp) and [HERE](https://github.com/smartsystemslab-uf/ZynqRobotController/blob/master/Software/UIO/GPIO_0_High.cpp) can be used to test the relay installation.


### Proper Terminal Soldering
We need to be sure not to destroy the terminals for the motor controller switch in case we wish to replace it. Solder on the outside and near the bottom of the terminals as shown below.

<p><img src="https://github.com/smartsystemslab-uf/ROS-ArloRobot/blob/master/dhb10-controller/Resources/Relay%20Wiring%20for%20ZynqRobotControllerv2%20on%20Arlo%20Robot.jpg?raw=True"/> </p>
