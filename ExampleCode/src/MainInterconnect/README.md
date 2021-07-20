# Example Code: Main Interconnection

The Recloser Simulator publishes the power flow through the main 
interconnection between the microgrid and the main grid. This is the device 
that is responsible for islanding and resyncing the microgrid from and to the 
main grid, as well as managing the active VF (Voltage/Frequency) Device. The 
recloser simulator publishes the power flowing through it, along with device 
status, device information, and microgrid status. The simulator subscribes to a 
topic that allows for planned islanding, planned resynchronization, and 
immediate islanding.

### Published Topics

- **Meas_NodePower**
- **Status_Device**

### Subscribed Topics

- **Control_Device**
- **Control_Power**: Used within the simulation for the Power Flow Simulator to 
  provide the power flowing in and out of the microgrid.
