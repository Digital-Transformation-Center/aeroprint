Launching AeroPrint
===============================

.. contents:: Table of Contents
   :depth: 2

Introduction
--------------------------------

Launching AeroPrint in the past has involoved a series of steps that were not always intuitive. This document will guide you through the process of launching AeroPrint.

Requirements
------------------------------

- AeroPrint Repository is cloned to your local machine.
- aeroprint_setup.bash script has been executed.
- The packages have been built for Starling and host (see `Building the Package <https://github.com/Digital-Transformation-Center/aeroprint/wiki/Running-ROS-Packages#build-the-package>`_)

Usage
-------------------------------

To run AEROPRINT:

1. Navigate to the AeroPrint directory.
2. Execute the application:

.. code-block:: bash

   bash aeroprint.bash

Host User Interface Features
----------------------------
- **System Checks**: The host user interface performs system checks to ensure that the system is ready for operation.
- **Splash Screen**: A splash screen is displayed on startup, configurable via the `ENABLE_SPLASH` and `SPLASH_TIME` variables.
- **Dynamic Path Handling**: Automatically detects and adjusts paths based on the installation directory.
- **ROS Integration**: Communicates with ROS nodes for initiating and monitoring scans.
- **Customizable UI**: The UI colors and dimensions can be customized through the code.

Configuration
-------------------------------

AEROPRINT can be configured by modifying the following variables in `aeroprint_gui.py`:

- `ENABLE_SPLASH`: Enable or disable the splash screen.
- `SPLASH_TIME`: Duration for which the splash screen is displayed.
- `DTC_BLUE`, `DTC_RED`, `BLACK`: Color schemes used in the UI.

Development
-----------

AEROPRINT is currently in Beta. Contributions and feedback are welcome. Please contact the author for more information or to contribute.

License
------------------------------------

AEROPRINT is provided under [LICENSE NAME]. For more details, see the LICENSE file in the repository.