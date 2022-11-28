# Automated Test Generator for Toyota HSR Bot (LUCY)

<p align="center" style="border:solid #6495ED">
  <img src="https://github.com/SOHAIL1996/Automatic_Test_Generation_For_Robot_Self_Examination/blob/master/Res%26Dev/videos/Toyota%20HSR%20robot%20(Lucy)%20-%20Performing%20tasks%20in%20a%20complex-scenario.gif" width="100%"/>
</p>

   One of the key challenges in domestic robotics is ensuring the correct behaviors of a robot when it performs a task. However, even simple scenarios in which a robot is tasked with grasping a cup poses a problem in the domestic environment, problems such as collision with obstacles, failure to grasp the object, or simply failing to recognize the cup. These problems' root causes lay in the environment's unpredictability, robot's lack of knowledge, hardware failures, and software faults. These problems are further compounded when it comes to complex-scenarios due to each scenario's dependence on the preceding scenario. The established approach for discovering these problems is through testing.

   The aim of this project was to facilitate the testing of domestic robotic systems by automatically generating a set of simulated test case scenarios in which a robot assessed its performance. Furthermore, the Toyota Human Support Robot (Lucy) was used as the test subject. Several of her behaviors were assessed, such as navigation, perception, and manipulation over four different use-cases. 
    
   The first use-case was on navigation, in which she had to navigate to various locations designated by a scenario generator. The second use-case was on the perception in which she had to perceive various objects selected by the scenario generator. The third use-case was on manipulation, in which she had to pick an object specified by the scenario generator. The fourth use-case was a combination of the aforementioned three use-cases along with an extra action of placing the object back on the table. 
    
   The framework for this project consisted of property-based tests. Properties were assigned to various actions of Lucy, which were then verified and validated. Verification and validation were achieved by using a simulated environment that provided evidence and affirmed Lucy's actions. At the end of each use-case, a report was generated, which provided comprehensive information on each property's success or failure within that use-case.

   The result of this project was that it correctly and consistently identified Lucy's failed actions (i.e. navigation, perception, and manipulation) in a variety of randomized use-case scenarios, which were otherwise considered a success by her planner. Moreover, the generated use-case reports provided deeper insights into the type and location of failure for each failed property.

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

![Overview of Automatic Test Generator](https://github.com/SOHAIL1996/ATG/blob/master/Res%26Dev/Images/nav_test-1.png)

## Running Perception Test

- Run `atg.sh` from `$(Parent directory)/atg`.
- Run `python2 -m pytest --alluredir=results tests/perceive_test.py -v -s` from `$(Parent directory)/atg/src`.
- Run `allure serve results/` from `$(Parent directory)/atg/src` to view results.

![Overview of Automatic Test Generator](https://github.com/SOHAIL1996/ATG/blob/master/Res%26Dev/Images/perceive_test-1.png)

## Running Pick Action Test

- Run `atg.sh` from `$(Parent directory)/atg`.
- Run `python2 -m pytest --alluredir=results tests/pick_test.py -v -s` from `$(Parent directory)/atg/src`.
- Run `allure serve results/` from `$(Parent directory)/atg/src` to view results.

![Overview of Automatic Test Generator](https://github.com/SOHAIL1996/ATG/blob/master/Res%26Dev/Images/pick_test-1.png)


## Running Complex Test

- Run `atg.sh` from `$(Parent directory)/atg`.
- Run `python2 -m pytest --alluredir=results tests/complex_test.py -v -s` from `$(Parent directory)/atg/src`.
- Run `allure serve results/` from `$(Parent directory)/atg/src` to view results.

![Overview of Automatic Test Generator](https://github.com/SOHAIL1996/ATG/blob/master/Res%26Dev/Images/complex_scenario-1.png)

## Acknowledgements

Supervised by:
 - Prof. Dr. Paul G. Pl ̈oger
 - Prof. Dr. Nico Hochgeschwender
 - Alex Mitrevski
