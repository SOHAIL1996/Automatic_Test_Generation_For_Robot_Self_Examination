# Automated Test Generator for Toyota HSR Bot (LUCY)

Robot automation has come a long way since its inception. Robots now have the ability to perform complex tasks like driving, cooking, or performing surgery. However, the question arises are how safe are these robots? or in other words, if one was to go to a hospital and have a robot perform surgery on them, what would be the success rate of that surgery. The good news is that there is a 99.52% chance that the surgery will be successful; the bad news is that there is a 0.48% chance that the surgery is unsuccessful. 
   
Our research and development project aims to facilitate testing complex robotic systems similar to robotic surgery by automatically generating a set of simulated test case scenarios in which a robot can assess its performance via tests. The Toyota HSR robot (Lucy) will be used as our test subject. Several behaviors of hers will be assessed such as navigation, perception, and manipulation over four different use-cases. The first use-case will be on navigation, in which she will have to navigate to various locations designated by a scenario generator. The second use-case will be on perception in which she will have to perceive various objects designated by the scenario generator. The third use-case will be on manipulation, in which she will have to pick an object designated by the scenario generator. The fourth use-case will be the combination of the aforementioned three use-cases along with an extra action of placing the object back on the table.
    
For the test framework, we will apply property-based tests. In this method, the environment provides evidence for the actions of a robot and affirms them. Finally, a report is generated that provides detailed information on each use-case's success and failure, which in a later project may be extended to use machine learning algorithms to improve the robot's planning strategy.

![Overview of Automatic Test Generator](https://github.com/SOHAIL1996/ATG/blob/master/Res%26Dev/Images/overview_test_suite-1.png)

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
* `tensorflow 1.4.0`
* `keras 2.0.8`
* `pandas 0.17.1`
* `termcolor 1.1.0`
* `Toyota HSR package`
* `MAS HSR package`
* `MAS MDR package`
* `yocs_cmd_vel_mux package`
* `pytest==4.6.11`
* `maven`
* `jdk 8`
* `allure-pytest==2.6.0`
* `allure==2.6.0`
* `torch==1.4.0`
* `torchvision==0.5.0`
* `allure-python-commons==2.6.0`
* `hypothesis--4.57.1`
* `future`

## Hardware Requirements

These constitute the bare minimum requirements to run this package.

* `8 Gb ram`
* `Intel® Core™ i5-6300HQ CPU @ 2.30GHz × 4 `
* `Nvidia GeForce GTX 960M/PCIe/SSE2`
* `250 Gb hard disk`

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

To configure the parameters of the simulator, open the utilities folder and set the parameters in the configuration file.

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

## Running Complex Test

- Run `atg.sh` from `$(Parent directory)/atg`.
- Run `python2 -m pytest --alluredir=results tests/complex_test.py -v -s` from `$(Parent directory)/atg/src`.
- Run `allure serve results/` from `$(Parent directory)/atg/src` to view results.


