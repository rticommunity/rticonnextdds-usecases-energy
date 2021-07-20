# Example Code: Energy Storage

The ESS (Energy Storage System) Simulator publishes the power it is absorbing 
or generating, status, system information, and VF (Voltage/Frequency Device 
capability. The status includes the systems SOC (State of Charge) and whether 
or not the system is acting as a VF Device. The ESS Simulator, along with the 
Generator Simulator can act as a VF device in island mode and includes code 
that simulates this behavior. The ESS simulator subscribes to control topics 
for connection and output. There is also a subscription to a topic used to 
manually establish SOC for simulation.

## Energy Storage Simulated Device

This exaple simulates an Energy Storage on the microgrid. This device acts as 
both an intelligent load and as both a current source and VF device.

### Published Topics

- **Meas_NodePower**
- **Meas_SOC**
- **Info_Battery**
- **Status_Device**
- **VF_Device**
- **Control_Power**: This is only used to create a base value for its own power 
  level

### Subscribed Topics

- **Control_Power**
- **Control_Device**
- **VF_Device_Active**
- **Control_SOC**: Used only within simulation to manually change device SOC.
