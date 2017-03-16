HBBA - Hybrid Behavior-Based Architecture
=========================================

This is the (in progress) implementation of HBBA, the robot control architecture
built at 3IT-IntRoLab (Université de Sherbrooke).
It has been built with ROS and our robots (IRL-1/TR and /AZ3 variants) in mind,
but has been designed to be compatible with various platforms.

Google or-tools has to be installed first to build the Intention Translator
(iw_translator).
Unfortunately, it is not currently offered as a Debian package, but can be built
from source or downloaded as a pre-built version for Ubuntu 14.04.
Also, see the "or_tools" subfolder of the iw_translator package for a dpkg
generation script that can then be used to install or-tools.

The whole distribution also requires those system dependencies:

 - bison
 - flex
 - libv8-dev

To build the distribution, do not forget to fetch the hbba_base submodule like
this (from the root directory of this repository):

$ git submodule init; git submodule update

Then, the whole system should build from a single catkin_make.
