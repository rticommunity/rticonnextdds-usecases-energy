# Example Code: Load

The Load Simulator publishes the power it is absorbing, status, and system 
information. The simulator subscribes to a Control message that allows the Load 
to be connected or disconnected from the grid independently. There is also a 
subscription to a topic used to manually set load for simulation.

### Published Topics

- **Meas_NodePower**
- **Info_Resource**
- **Status_Device**
- **Control_Power**: This is only used to create a base value for its own power 
  level

### Subscribed Topics

- **Control_Power**
- **Control_Device**
