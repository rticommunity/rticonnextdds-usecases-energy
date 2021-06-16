# Example Code: Visualization

## Visualizer
Combining GTK+ 3.0 and RTI Connext DDS, this application provides a graphical
user interface to the system, taking data from the different grid elements and
presenting it to the user. This interface can also be used to modify some system
parameters and cause the microgrid to connect to and disconnect from the main
grid.

### Published Topics

- **Control_Power**
- **Control_Device**
- **Control_Microgrid**
- **Control_Irradiance**
- **Control_SOC**
- **VF_Device_Active**

### Subscribed Topics

- **Meas_NodePower**
- **Meas_SOC**
- **Info_Resource**
- **Info_Generator**
- **Info_Battery**
- **Status_Device**
- **Status_Microgrid**

## Building the Example :wrench:

To build this example, you must have gtk+ and gtkmm installed on your
development system. Glade is also very useful for investigating and editing
the *Visualizer.glade* configuration file.

On Ubuntu you can install the necessary (and optional) prerequisites with the
following commands.

``` shell
sudo apt update
sudo apt upgrade
sudo apt install libgtkmm-3.0-dev glade
```

Once the dependencies are installed, follow the instructions in the README in
the `src` directory
