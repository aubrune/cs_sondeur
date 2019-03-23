# Robot sondeur

```
sudo apt install ros-melodic-rosbridge-library ros-melodic-rosbridge-server ros-melodic-rosserial-msgs ros-melodic-rosserial-client
ROS_MASTER_URI=http://10.42.0.49:11311


cd ~/ros_ws/src
# Do not get git clone https://github.com/Comau/eDO_core
git clone https://github.com/Comau/eDO_core_msgs
git clone https://github.com/Comau/eDO_description
git clone https://github.com/Comau/eDO_moveit
```

## TODO
algorithm thing stop
trajectory sensors control msgs in deps

# E.do retroengineering
## Nodes

```
$ rosnode list 
/aprik3_1460_1547199028921
/c5g_1481_1547199031203
/edo_algorithms
/edo_state_machine
/rosbridge_websocket
/rosout
/rosserial_server_serial_node
/rostopic_1562_1547199033282
/system_cmd_node
```

## Services
```
$ rosservice list|grep -v logger
/algo_control_switch_srv
/algo_jnt_number_srv
/algo_load_configuration_file_srv
/machine_bridge_sw_version_srv
/system_command_srv
```

### Service `/system_command_srv`
Served in https://github.com/Comau/eDO_core/blob/master/src/systemcmd.py
Node: /system_cmd_node
Type: edo_core_msgs/SystemCommand
Args: command data

Command 0: return UTC
Command 1: Set LAN IP
Command 2: Get LAN IP
Command 3: Set Wifi credentials
Command 4: SSID 

### Service `/algo_jnt_number_srv`

```
rosservice call /algo_jnt_number_srv "{}" 
counter: 7
joints_mask: 127
joints_aux_mask: 64
```

counter = -2 when motors are off 

### Service `/algo_control_switch_srv`
```
rosservice info /algo_control_switch_srv
Node: /edo_algorithms
URI: rosrpc://192.168.12.1:60654
Type: edo_core_msgs/ControlSwitch
Args: mode
```

In order to disable the algorithm manager:
`rosservice call /algo_control_switch_srv "mode: 1"`

Mode 0 = Take control
Mode 1 = Release control


Get the current state:
```
rostopic echo /algorithm_state -n1
```

Available Algorithm states:
NO MESSAGE PUBLISHED # Control released
"UNINITIALIZED(0)"   # Robot has just been powered up!
"INITIALIZED(1)"     # Motors are compliant
"MOVING(2)",
"WAITING(3)",
"BLOCKED(4)",
"FINISHED(5)",
"PAUSE(6)",
"RECOVERY(7)",
"SWITCHED_OFF(8)"


## Machine states

Get current state:
```
rostopic echo /machine_state -n1
```

COMMAND_STATE = 255, /* Temporary state when a command is executed */
INIT = 0, /* Initial State */
NOT_CALIBRATE = 1, /* Not calibrated */
CALIBRATE = 2, /* Calibrated */
MOVE = 3, /* Move execution in progress */
JOG = 4, /* Jog execution in progress */
MACHINE_ERROR = 5, /* Error state, needing a reboot */
BRAKED = 6 /* brake active, no motor power supply */


Machine opcode bits:
NACK = 0, /* Bit 0 - At least 1 joint didn't send an ACK */
JOINT_ABSENT = 1, /* Bit 1 - A joint is not publishing its status. Hard stop. */
JOINT_OVERCURRENT = 2, /* Bit 2 - Joint in over current. Hard stop. */
JOINT_UNCALIBRATED = 3, /* Bit 3 - Joints not calibrated. Only jogs are accepted. */
POSITION_ERROR = 4, /* Bit 4 - Position error. Hard stop. */
ROSSERIAL_ERROR = 5, /* Bit 5 - Rosserial error. No state from joints. Hard stop. */
BRAKE_ACTIVE = 6, /* Bit 6 - Brake active. No power supply provided to motors. */
EMERGENCY_STOP = 7, /* Bit 7 - E-stop active */
FENCE = 8 /* Bit - Fence active */

## Some topics

### Joint state `/machine_algo_jnt_state`
edo_core_msgs/JointStateArray

### USB Joint state ``
edo_core_msgs/JointStateArray
Same as previous joint states, bypasses the state machine. To be preferred.

### Cartesian EEF pose
edo_core_msgs/CartesianPose
rosmsg show edo_core_msgs/CartesianPose
float32 x
float32 y
float32 z
float32 a
float32 e
float32 r
string config_flags

```
rostopic echo /cartesian_pose -n1
x: -1.00390100479
y: -0.0482611581683
z: 1117.71777344
a: -176.291915894
e: 0.135620936751
r: -178.48449707
config_flags: "S E W "
---
```


