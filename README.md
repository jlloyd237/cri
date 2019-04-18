# Common Robot Interface
>Provides a common Python 3 programming interface for different robot arms.

Common Robot Interface (CRI) currently supports ABB robot controllers, and UR controllers that implement the Real-Time Data Exchange (RTDE) protocol.

A robot jogger tool built on top of this framework is also included in the package.

## Installation

To install the package on Windows, OS X or Linux, clone the repository and run the setup script from the repository root directory:

```sh
python setup.py install
```
If you wish to use it, the robot jogger tool requires the PyQt5 GUI framework, which can be installed using the Python package installer:
```sh
pip install pyqt5
```
In a Conda environment, the CRI framework and robot jogger tool can be installed using:
```sh
conda create -n cri python=3.6
conda activate cri
python setup.py install
pip install pyqt5
```

## Uploading and running the server

The framework uses a client-server approach for communicating with the back-end robot controller.  Before running the server on the controller, the code must be uploaded using an FTP client or by some other means.

For an ABB robot, upload the `/cri/abb/abb_server.mod` file to the controller, install it as the main program module under a new program and then run the program using the FlexPendant or RobotStudio application.

For a UR robot, upload the `/cri/ur/rtde_server.script`file to the controller, install it as the main script under a new program and then run the program using the Teach Pendant.

## Usage example

A couple of examples that demonstrate how to use the framework are included in the `/tests` directory.  These can be modified to suit different system configurations and requirements (e.g., IP address, work space, work frame, etc.)

After installing and running the server (see above), the test scripts can be run from the `/tests` directory using:

```sh
python abb_robot_test.py
```
or
```sh
python rtde_robot_test.py
```
The robot jogger tool is included in the `/tools/robot_jogger` directory.  After installing and running the server (see above), the tool can be run from this directory using:
```sh
python robot_jogger.py
```

## Meta

John Lloyd – jlloyd237@gmail.com

Distributed under the GPL v3 license. See ``LICENSE`` for more information.

[https://github.com/jloyd237/cri](https://github.com/jlloyd237/)
