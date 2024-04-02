# fetch_nav_isaac
[![Python Version](https://img.shields.io/badge/Python-3.10.13-blue)](https://www.python.org/downloads/release/python-1013/)
[![NVIDIA Isaac Sim](https://img.shields.io/badge/NVIDIA%20Isaac%20Sim-2023.1.1-blue)](https://developer.nvidia.com/isaac-sim)


A repository that stores the codebase for Python Interface to control Fetch Robot for Navigation purposes in NVIDIA Isaac Sim.

## Current Features:

1. Initialization of Fetch to move its base on a ground plane in Isaac Sim. The arm and the RGB camera are both initialized.
2. Capability of moving the base to a given global position.
3. Receiving a stream of images from the RGB camera attached to the prim: `head_tilt_link/head_camera_link`.
4. Synthetic data generation such as 2D and 3D bounding box detection and rudimentary segmentation
using the Isaac Sim's built-in libraries.

## Execution:
Path to the Isaac Sim python would be of the following format:
`~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh`

Execute the following script to launch the driver to navigate Fetch:
`<path to isaac sim python> -m src.launch.fetch_nav_driver`

## Troubleshooting

In case of an 150 (GLX) error, execute the following command:

`export MESA_GL_VERSION_OVERRIDE=4.6`