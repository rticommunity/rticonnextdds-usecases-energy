# Building the Project :wrench:

To build this project, first run CMake to generate the corresponding build
files. In order for the provided script to function this must be done in the
`build` directory created within the `src` directory.

```sh
mkdir build
cd build
cmake ..
```

Once you have run CMake, you will find a number of new files in your build
directory (the list of generated files will depend on the specific CMake
Generator). To build the example, run CMake as follows:

```sh
cmake --build .
```

**Note**: if you are using a multi-configuration generator, such as Visual
Studio solutions, you can specify the configuration mode to build as follows:

```sh
cmake --build . --config Release|Debug
```

Alternatively, you can use directly the generated infrastructure (e.g.,
Makefiles or Visual Studio Solutions) to build the example. If you generated
Makefiles in the configuration process, run make to build the example. Likewise,
if you generated a Visual Studio solution, open the solution and follow the
regular build process.

## Running the Example

Simply run the `runAll.sh` script located in the `resources/scripts` directory.
```sh
./runAll.sh
```

## Customizing the Build

### Configuring Build Type and Generator

By default, CMake will generate build files using the most common generator for
your host platform (e.g., Makefiles on Unix-like systems and Visual Studio
solution on Windows), \. You can use the following CMake variables to modify the
default behavior:

-   `-DCMAKE_BUILD_TYPE` -- specifies the build mode. Valid values are Release
    and Debug. See the [CMake documentation for more details.
    (Optional)](https://cmake.org/cmake/help/latest/variable/CMAKE_BUILD_TYPE.html)

-   `-DBUILD_SHARED_LIBS` -- specifies the link mode. Valid values are ON for
    dynamic linking and OFF for static linking. See [CMake documentation for
    more details.
    (Optional)](https://cmake.org/cmake/help/latest/variable/BUILD_SHARED_LIBS.html)

-   `-G` -- CMake generator. The generator is the native build system to use
    build the source code. All the valid values are described described in the
    CMake documentation [CMake Generators
    Section.](https://cmake.org/cmake/help/latest/manual/cmake-generators.7.html)

For example, to build a example in Debug/Static mode run CMake as follows:

```sh
cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_SHARED_LIBS=ON .. -G "Visual Studio 15 2017" -A x64
```

### Configuring Connext DDS Installation Path and Architecture

The CMake build infrastructure will try to guess the location of your Connext
DDS installation and the Connext DDS architecture based on the default settings
for your host platform.If you installed Connext DDS in a custom location, you
can use the CONNEXTDDS_DIR variable to indicate the path to your RTI Connext DDS
installation folder. For example:

```sh
cmake -DCONNEXTDDS_DIR=/home/rti/rti_connext_dds-x.y.z ..
```

Also, If you installed libraries for multiple target architecture on your system
(i.e., you installed more than one target rtipkg), you can use the
CONNEXTDDS_ARCH variable to indicate the architecture of the specific libraries
you want to link against. For example:

```sh
cmake -DCONNEXTDDS_ARCH=x64Linux3gcc5.4.0 ..
```

# Data Model: Energy Comms

Each device within these subdirectories shares a common data model. Through this
data model they are able to operate together to support islanded operation and
seemless handoffs of VF support.

## Device Descriptions

- **Controller**: Microgrid Controller. This application is relatively
  lightweight as-is, but the inclusion of more comprehensive optimization could
  make this application very resource intensive. In that case it should be
  possible to separate optimization and microgrid control into separate
  applications to ensure smooth operation.

- **EnergyStorage**: Energy Storage. This device simulates a energy storage unit
  capable of providing VF support in the case of islanded operation. As time goes
  on the SOC will change depending on the charging or discharging of the unit.

- **Generator**: Generator. This simulated generator is capable of providing VF
  support to the microgrid. It includes a simulated ramp time to go from
  Disabled_Ready to Enabled_On.

- **Load**: Load. This mostly acts as a static load, although it can be changed
  through the Control_Power topic. With some small changes the load could also
  change randomly or by reading a file.

- **MainInterconnect**: Main Interconnect. This device simulates a recloser that
  sits between the microgrid and the main grid.

- **PowerFlowSim**: Power Flow Simulator. This is a very simple power flow
  simulator that provides the power going through the main interconnect during
  grid-connected operation and the power necessary for the active VF device to
  provide during islanded operation.

- **SolarPV**: Solar Array. This is a simulated solar array that changes its
  output based on solar irradiance and limited through curtailment.

## Topic Descriptions

- **Meas_NodePower**: Node Power Measurement. Specifies the power supplied to or
  pulled from the network node (in kW) the device is attached to.

- **Meas_SOC**: State of Charge Measurement. Specific to Energy Storage.
  Specifies the current state of charge of the energy storage device as a
  percentage of it's total capacity. The value is between 0.0 and 100.0.

- **Info_Resource**: Generic Resource Information. This information is available
  for all connected electric devices and includes the node the device is
  connected to, the Max Load (in kW) the device can pull, and the Max Generation
  (in kW) that the device can provide. In this system Load is negative and
  Generation is positive.

- **Info_Generator**: Generator Information. This includes the generic resource
  information and adds a Generator Efficiency Curve and a Ramp Up Time. The
  Generator Efficiency Curve is a bounded sequence of points describing the
  generator's efficiency at different power levels, with the power levels being
  measured in kW and the efficiency being between 0.0 and 1.0. The Ramp Up Time
  is the amount of time it takes the generator to go from off to ready to take
  over as a VF Device and is measured in integer seconds.

- **Info_Battery**: Energy Storage Information. This includes the generic
  resource information and adds a Capacity value. The capacity is measured in kWh.

- **Status_Device**: Current Device Status. This topic contains Connection
  Status and Operation Status. Connection Status defines whether there is an
  electrical connection between the device and the node or, in the case of the
  Main Interconnect, whether the device itself is closed. Operation Status
  provides a variety of states the device could be in.

- **Status_Microgrid**: Microgrid Status. The microgrid has 4 distinct operating
  states: Connected, Island Request, Islanded, and Resynchronization Request.
  Each state and state transition requires different operations to be performed,
  especially in terms of optimization.

- **Control_Microgrid**: Microgrid Control. The Control_Microgrid topic uses the
  same type as Status_Microgrid. The only difference is that this topic is used
  to cause a change in the microgrid's state.

- **Control_Device**: Device Connection Control. This allows devices to be
  connected or disconnected from the microgrid. In the case of the Main
  Interconnect, this topic defiines whether or not the microgrid is to connect
  to or disconnect from the main grid.

- **Control_Power**: Power Setpoint. In a real microgrid this topic allows
  optimization to set the power output of assets including generators, energy
  storage units, PV arrays (curtailment), and smart loads. This is being done
  within this example, but this topic is also being used to support the
  simulation. The Power Setpoint is measured in kW.

- **Control_Irradiance**: Irradiance Control. This topic is used wholly for
  simulation and provides a mechanism to change the solar irradiance present on
  the solar panels during simulation.

- **Control_SOC**: SOC Control. This topic is used wholly for simulation and
  provides a mechanism to change the SOC of an energy storage system during
  simulation.

- **VF_Device**: VF Device Available. This topic actively uses ownership and
  ownership strength to vie for which device will be providing voltage and
  frequency support to the microgrid while islanded. As the energy storage
  system charges and discharges the device taking this role can change.

- **VF_Device_Active** Command Device to Support Voltage and Frequency. This
  topic specifies which device will take on the responsibility of voltage and
  frequency support and when it will do so. This allows for seamless hand-off
  of VF support, as well as mode changes for islanding and resynchronization.
