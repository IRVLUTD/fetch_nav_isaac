# fetch_nav_isaac
[![Python Version](https://img.shields.io/badge/Python-3.10.13-blue?logo=python)](https://www.python.org/downloads/release/python-1013/)
[![NVIDIA Isaac Sim](https://img.shields.io/badge/NVIDIA%20Isaac%20Sim-2023.1.1-blue?logo=nvidia)](https://developer.nvidia.com/isaac-sim)
[![NVIDIA Driver Version](https://img.shields.io/badge/NVIDIA%20Driver-Version%20525.85.05-informational?logo=nvidia)](https://www.nvidia.com/Download/index.aspx)
[![CUDA Version](https://img.shields.io/badge/CUDA-Version%2012.0-%2376B900?logo=nvidia)](https://developer.nvidia.com/cuda-zone)


A repository that stores the codebase for Python Interface to control Fetch Robot for Navigation purposes in NVIDIA Isaac Sim.

## Current Features:

1. Initialization of Fetch to move its base on a ground plane in Isaac Sim. The arm and the RGB camera are both initialized.
2. Capability of moving the base to a given global position (x, y) within a given position tolerance (current best is 0.01 metres).
3. Receiving a stream of images from the RGB camera attached to the prim: `head_tilt_link/head_camera_link`.
4. Synthetic data generation such as 2D and 3D bounding box detection and rudimentary segmentation
using the Isaac Sim's built-in libraries.

## Setup and Config:

- You are required to install [Omniverse](https://www.nvidia.com/en-us/omniverse/download/), in which you would install [NVIDIA Isaac Sim](https://www.youtube.com/watch?v=ZUX9SrPGrbk), 
its cache and set up a Nucleus server.
- It is essential that the Nucleus server is running at all times when you are working with the Isaac sim, even for the standalone scripts.

- All the configuration parameters are stored under `/config/config.ini`
 file. These could be changed as per the need of the application. 
Make sure that all the paths and the file names are correct.

## Execution:
Path to the Isaac Sim python would be of the following format:
`~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh`

Execute the following script to launch the driver to navigate Fetch:
`<path to isaac sim python> -m src.launch.fetch_nav_driver`

### 1. Move Fetch around:

- Enter the global target position coordinates (x, y) in the terminal after executing the python script mentioned above. Enter (q, q) to exit.
- The script calculates the corrected coordinates to reduce the error of about 0.06 metres from the built-in Differential Controller to less than 0.01 metres.
  (Update: A bug in the coordinate correction logic was corrected in the latest release)
- Wait for our friend to reach its destination. 
- Repeat as many times as you like. 

Congratulations, you are now commanding a robot.

#### Scope for improvements:

- Multiple redundant rotations while going long distances along the -x axis.
- Add logic to stop rendering either when the robot's position is within the tolerance limit or it has stopped moving.
- May be prone to errors while travelling long distances and very large or very small slopes (y/x).

## Troubleshooting

- In case of an 150 (GLX) error, execute the following command:

`export MESA_GL_VERSION_OVERRIDE=4.6`

- In case of a `Detected blocking function call` error and consequential compromise on the program runtime, please check for a
valid Nucleus server installation in Omniverse. Ensure that the server is running to avoid the error.