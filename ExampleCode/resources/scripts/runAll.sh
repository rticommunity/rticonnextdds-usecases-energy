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

if [ -f $bin_dir/$executable_es_name ]
then
    cd $bin_dir
    gnome-terminal --tab -- bash -c "./$executable_es_name $*"
else
    echo "***************************************************************"
    echo $executable_name executable does not exist in:
    echo $bin_dir
    echo ""
    echo Please, try to recompile the application using the command:
    echo " $ cmake --build ."
    echo while in the build directory.
    echo "***************************************************************"
fi

if [ -f $bin_dir/$executable_load_name ]
then
    cd $bin_dir
    gnome-terminal --tab -- bash -c "./$executable_load_name $*"
else
    echo "***************************************************************"
    echo $executable_name executable does not exist in:
    echo $bin_dir
    echo ""
    echo Please, try to recompile the application using the command:
    echo " $ cmake --build ."
    echo while in the build directory.
    echo "***************************************************************"
fi

if [ -f $bin_dir/$executable_gen_name ]
then
    cd $bin_dir
    gnome-terminal --tab -- bash -c "./$executable_gen_name $*"
else
    echo "***************************************************************"
    echo $executable_name executable does not exist in:
    echo $bin_dir
    echo ""
    echo Please, try to recompile the application using the command:
    echo " $ cmake --build ."
    echo while in the build directory.
    echo "***************************************************************"
fi

if [ -f $bin_dir/$executable_pv_name ]
then
    cd $bin_dir
    gnome-terminal --tab -- bash -c "./$executable_pv_name $*"
else
    echo "***************************************************************"
    echo $executable_name executable does not exist in:
    echo $bin_dir
    echo ""
    echo Please, try to recompile the application using the command:
    echo " $ cmake --build ."
    echo while in the build directory.
    echo "***************************************************************"
fi

if [ -f $bin_dir/$executable_main_name ]
then
    cd $bin_dir
    gnome-terminal --tab -- bash -c "./$executable_main_name $*"
else
    echo "***************************************************************"
    echo $executable_name executable does not exist in:
    echo $bin_dir
    echo ""
    echo Please, try to recompile the application using the command:
    echo " $ cmake --build ."
    echo while in the build directory.
    echo "***************************************************************"
fi

if [ -f $bin_dir/$executable_sim_name ]
then
    cd $bin_dir
    gnome-terminal --tab -- bash -c "./$executable_sim_name $*"
else
    echo "***************************************************************"
    echo $executable_name executable does not exist in:
    echo $bin_dir
    echo ""
    echo Please, try to recompile the application using the command:
    echo " $ cmake --build ."
    echo while in the build directory.
    echo "***************************************************************"
fi

if [ -f $bin_dir/$executable_control_name ]
then
    cd $bin_dir
    gnome-terminal --tab -- bash -c "./$executable_control_name $*"
else
    echo "***************************************************************"
    echo $executable_name executable does not exist in:
    echo $bin_dir
    echo ""
    echo Please, try to recompile the application using the command:
    echo " $ cmake --build ."
    echo while in the build directory.
    echo "***************************************************************"
fi

if [ -f $bin_dir/$executable_viz_name ]
then
    cd $bin_dir
    gnome-terminal --tab -- bash -c "./$executable_viz_name $*"
else
    echo "***************************************************************"
    echo $executable_name executable does not exist in:
    echo $bin_dir
    echo ""
    echo Please, try to recompile the application using the command:
    echo " $ cmake --build ."
    echo while in the build directory.
    echo "***************************************************************"
fi
