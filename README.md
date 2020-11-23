# Automated Test Generator for Toyota HSR Bot (LUCY)

This package aims to enables the HSR bot to examine itself for basic
faults in a variety of test case scenarios that range from grasping 
actions to the execution of complex scenarios automatically.


## Software Requirements

* `Ubuntu 16.04 LTS`
* `Python 3.6.12 64-bit`
* `Python 2.7.12 64-bit`
* `Gazebo 7.16.1`
* `Catkin-pkg 0.4.22-100`
* `roskinetic`
* `numpy 1.11.0`
* `numpy-stl`
* `cuda 11.0`
* `cuddnn 8`
* `numpy-stl`
* `tensorflow 1.4.0`
* `keras 2.0.8`
* `numpy-stl`
* `pandas 0.17.1`
* `termcolor 1.1.0`
* `Toyota HSR package`
* `MAS HSR package`
* `MAS Domestic package`
* `ssd_keras_ros package`
* `yocs_cmd_vel_mux package`
* `pytest==4.6.11`
* `maven`
* `jdk 8`
* `allure-pytest==2.6.0`
* `allure==2.6.0`
* `allure-python-commons==2.6.0`
* `hypothesis--4.57.1`
* `reportlab`

## Hardware Requirements

These constitute the bare minimum requirements to run this package.

* `+8 Gb ram`
* `+Intel® Core™ i5-6300HQ CPU @ 2.30GHz × 4 `
* `+Nvidia GeForce GTX 960M/PCIe/SSE2`
* `+250 Gb hard disk`

## Setup

1. git clone and Install the Toyota HSR package from gitlab in the `catkin_ws/src/`.
2. git clone and Install the MAS domestic repository package from github in the `catkin_ws/src/`.
3. git clone and Install the MAS HSR package from gitlab in the `catkin_ws/src/`.
4. Build the catkin_ws.

## First time installation

- Correct directory of world file.

## Settings
- Add to bash.rc file `export ROBOT_ENV=atg_lab`
- Add the map folder to `mdr_environments` which should contain the `map.yaml`,`map.pgm` and `navigations_goal.yaml` files.

### Information
After setting up the Toyota HSR environment. You will have to source the `atg` package and it is best to add it
in the `~/.bashrc` below the ros kinetic package.


## Configuration

To configure the parameters of the simulator goto the utilities file and set the parameters in the configuration file.

## Simulation Startup

To use simply open the simulator with Lucy in it, run `./atg.sh` from `$(Parent directory)/atg`.

## Running Navigation Test

- Run `./atg.sh` from `$(Parent directory)/atg`.
- Run `python2 -m pytest --alluredir=results tests/nav_test.py -v -s` from `$(Parent directory)/atg/src`.
- Run `allure serve results/` from `$(Parent directory)/atg/src` to view results.

## Running Perception Test

- Run `atg.sh` from `$(Parent directory)/atg`.
- Run `python2 -m pytest --alluredir=results tests/perceive_test.py -v -s` from `$(Parent directory)/atg/src`.
- Run `allure serve results/` from `$(Parent directory)/atg/src` to view results.

## Running Pick Action Test

- Run `atg.sh` from `$(Parent directory)/atg`.
- Run `python2 -m pytest --alluredir=results tests/pick_test.py -v -s` from `$(Parent directory)/atg/src`.
- Run `allure serve results/` from `$(Parent directory)/atg/src` to view results.

## Running Place Action Test

- Run `atg.sh` from `$(Parent directory)/atg`.
- Run `python2 -m pytest --alluredir=results tests/place_test.py -v -s` from `$(Parent directory)/atg/src`.
- Run `allure serve results/` from `$(Parent directory)/atg/src` to view results.


## Acknowledgments
