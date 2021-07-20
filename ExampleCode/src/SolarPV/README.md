# Example Code: Solar PV

The solar simulator publishes the PV system power (PV output), PV system
information, and PV system status. The solar simulator subscribes to control
topics for connection and output (curtailment). There is also a subscription
to a solar irradiance topic used for simulation.

### Published Topics

- **Meas_NodePower**
- **Info_Resource**
- **Status_Device**
- **Control_Power**: This is only used to create a base value for its own power
  level
- **Control_Irradiance**: Used to provide an irradiance instance if no other
  device is doing so.

### Subscribed Topics

- **Control_Power**
- **Control_Device**
- **Control_Irradiance**: Wholly for simulation. In a real system this is
  environmentally dependent.
