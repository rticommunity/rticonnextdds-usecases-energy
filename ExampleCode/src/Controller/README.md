# Example Code: Microgrid Controller

The Controller handles setpoints for devices during islanded mode, along with 
decisions regarding the active VF (Voltage/Frequency) Device. It publishes to 
device control topics and VF Control. The simulator also subscribes to all 
measurement topics in order to help balance the system during islanded 
operation. The optimizer also sets up VF Control transfer between the simulated 
Energy Storage and Generator systems.

## Microgrid Controller
This application combines microgrid control and microgrid optimization. Within
the application these roles are kept relatively separate, which could allow for
splitting them into two different applications.

### Published Topics

- **Control_Power**
- **Control_Device**
- **Status_Microgrid**
- **VF_Device_Active**

### Subscribed Topics

- **Meas_NodePower**
- **Meas_SOC**
- **Info_Resource**
- **Info_Generator**
- **Info_Battery**
- **Status_Device**
- **Status_Microgrid**
- **VF_Device**
