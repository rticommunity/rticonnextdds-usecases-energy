# Example Code: Power Flow Simulator

The Power Flow Simulator takes information from each device on the network and 
generates the powerflow through the main interconnection. The simulator 
subscribes to all measurement topics and publishes to a topic used by the main 
interconnection for its power measurement topic.

### Published Topics

- **Control_Power**: Used to set VF Power levels during islanded operation and 
  main interconnect power levels during grid-connected operation.

### Subscribed Topics

- **Meas_NodePower**
- **Status_Microgrid**
- **VF_Device_Active**
