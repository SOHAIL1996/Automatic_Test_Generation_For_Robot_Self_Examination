# Automated Test Generator for Toyota HSR Bot (LUCY)

This package aims to enables the HSR bot to examine itself for basic
faults in a variety of test case scenarios that range from grasping 
actions to the execution of complex scenarios automatically.

<!-- ## README contents

1. [Defined environments](#Defined-environments)
2. [Gazebo entities](#Gazebo-entities)
    1. [Gazebo worlds](#Gazebo-worlds)
        * [Square world](#Square-world)
    2. [Object models](#Object-models)
3. [Setup](#Setup)
4. [Usage](#Usage)
5. [Requirements](#Requirements)
6. [Acknowledgments](#Acknowledgments) -->

## SoftwareRequirements

* `Python 3.6.12 64-bit`
* `Gazebo 7.16.1`
* `Catkin-pkg 0.4.22-100`
* `roskinetic`
* `numpy 1.11.0`
* `numpy-stl`
* `pandas 0.17.1`
* `termcolor 1.1.0`
* `Toyota HSR package`
* `MAS HSR package`
* `MAS Domestic package`

## Setup

1. Set up the package:

### Information
After setting up the Toyota HSR environment. You will have to source the `atg` package and it is best to add it
in the `~/.bashrc` below the ros kinetic package.

## Configuration

## Usage

To use simply open the terminal goto the package directory and run `./atg.sh`

## Gazebo entities

### Gazebo worlds

### Object models

## Navigation

- Run `atg.sh`.
- Run `rosrun mdr_move_base_action move_base_action_client_test <Navigation goal>`
- Run `python3.7 -m pytest action_test.py -v -s`

## First time installation

- Correct directory of world file.

## Settings
- Add to bash.rc file `export ROBOT_ENV=atg_lab`
- Add the map folder to `mdr_environments` which should contain the `map.yaml`,`map.pgm` and `navigations_goal.yaml` files.


## Acknowledgments
