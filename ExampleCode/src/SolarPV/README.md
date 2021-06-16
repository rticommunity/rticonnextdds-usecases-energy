# Example Code: Solar PV

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
