# Example Code: Generator

The Generator Simulator publishes the power it is generating, status, system 
information, and VF (Voltage/Frequency) Device capability. The status includes 
whether or not the system is acting as a VF Device. The Generator Simulator, 
along with the ESS Simulator can act as a VF device in island mode and includes 
code that simulates this behavior. In the case of the generator simulator, a 
delay is included that does not allow the device to output power if it is 
disabled.

### Published Topics

- **Meas_NodePower**
- **Info_Generator**
- **Status_Device**
- **VF_Device**
- **Control_Power**: This is only used to create a base value for its own power 
  level

### Subscribed Topics

- **Control_Power**
- **Control_Device**
- **VF_Device_Active**
