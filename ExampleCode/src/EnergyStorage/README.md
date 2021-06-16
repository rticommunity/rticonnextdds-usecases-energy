# Example Code: Energy Storage

## Energy Storage Simulated Device

This exaple simulates an Energy Storage on the microgrid. This device acts as both an intelligent
load and as both a current source and VF device.

### Published Topics

- **Meas_NodePower**
- **Meas_SOC**
- **Info_Battery**
- **Status_Device**
- **VF_Device**
- **Control_Power**: This is only used to create a base value for its own power level

### Subscribed Topics

- **Control_Power**
- **Control_Device**
- **VF_Device_Active**
- **Control_SOC**: Used only within simulation to manually change device SOC.
