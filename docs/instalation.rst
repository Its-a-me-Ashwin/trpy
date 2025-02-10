=================
Installation
=================

Follow the steps below to install the package and get it working with your robot.

Prerequisites
-----------------
- Ensure you have your **U2D2 module** connected to the robot.  
  This module acts as a bridge between USB serial and Dynamixel servos.
- The Python script connects to this module via **pyserial** to communicate with the robot.

Installation Methods
------------------------

You can install the package in two ways:
1. **Build and install manually**
2. **Use a pre-generated wheel**

### Manual Installation

To install the package manually, follow these steps:

.. code-block:: bash

   git clone https://github.com/Its-a-me-Ashwin/trpy.git
   cd trpy/trpy
   pip install -u .

### Testing the Installation

Once installed, run the test script:

.. code-block:: bash

   python ./test/test_motion.pyserial

**Expected Outcome:**
- The robot should move to a specific orientation for a few seconds.
- Ensure the robot is **held in place** before executing the test command, as the weight distribution may change.
