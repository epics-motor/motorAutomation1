# motorAutomation1

EPICS motor driver for the following [Aerotech](https://www.aerotech.com/) controller: [Automation1](https://www.aerotech.com/product/software/automation1-software-based-machine-controller/)

motorAutomation1 is a submodule of [motor](https://github.com/epics-modules/motor).  When motorAerotechAutomation1 is built in the ``motor/modules`` directory, minor manual configuration is needed.

motorAutomation1 can also be built outside of motor by copying it's ``EXAMPLE_RELEASE.local`` file to ``RELEASE.local`` and defining the paths to ``MOTOR`` and itself.

motorAutomation1 contains an example IOC that is built if ``CONFIG_SITE.local`` sets ``BUILD_IOCS = YES``.  The example IOC can be built outside of driver module.

**Note:** In order to build and run this motor module, the user must do the following:

1. Copy the C API header files (from your Automation1 installation) to the ```motorAutomation1/automation1Sup/Include``` directory.
2. Copy the C API shared object files (libautomation1c.so and libautomation1compiler.so) to the ```motorAutomation1/automation1Sup/Lib``` directory.
3. Add the path of the bin generated during the build (when building from ```MOTOR``` it should be ```/synApps_X_X/support/motor-RX-X/bin/linux-x86_64```) to the system's 
   '''LD_LIBRARY_PATH''' or equivalent variable used to find runtime libraries.

If using st.cmd.automation1 from the example IOC, be sure to change the host name to the IP address of your Automation1 controller.

The Automation1 C API for Linux supports Debian 10.x. This motor module has also been tested on Ubuntu 18.04. Any Linux distribution that has GCC 8.3.0, GLIBC 2.28,
GLIBCXX 3.4.25, libsodium 1.0.17, or newer versions, may work.

