#!/bin/sh

filename=$0
script_dir=`dirname $filename`
executable_es_name="EnergyStorage"
executable_load_name="Load"
executable_gen_name="Generator"
executable_pv_name="SolarPV"
executable_main_name="MainInterconnect"
executable_sim_name="PowerFlowSim"
executable_control_name="Controller"
executable_viz_name="Visualizer"
bin_dir=${script_dir}/../../src/build

printError(){
    echo "***************************************************************"
    echo $1 executable does not exist in:
    echo $2
    echo ""
    echo Please, try to recompile the application using the command:
    echo " $ cmake --build ."
    echo while in the build directory.
    echo "***************************************************************"
}

execute() {
if [ -f $2/$1 ]
then
    cd $2
    gnome-terminal --tab -- bash -c "./$1; sleep 5"
else
    printError $1 $2
fi
}

execute $executable_sim_name $bin_dir
sleep 4 # Make sure PowerFlowSim is running before starting devices

execute $executable_es_name $bin_dir
execute $executable_load_name $bin_dir
execute $executable_gen_name $bin_dir
execute $executable_pv_name $bin_dir
execute $executable_main_name $bin_dir
sleep 5 # Make sure all devices are running before starting controller

execute $executable_control_name $bin_dir
execute $executable_viz_name $bin_dir
