HBBA - Hybrid Behavior-Based Architecture
=========================================

This is the (in progress) implementation of HBBA, the robot control architecture
built at 3IT-IntRoLab (Université de Sherbrooke).
It has been built with ROS and our robots (IRL-1/TR and /AZ3 variants) in mind,
but has been designed to be compatible with various platforms.

To build the Intention Translator, which relies on Google or-tools, you need to
install these packages (on Ubuntu 14.04):

 - autoconf
 - bison
 - curl
 - flex
 - g++
 - gawk
 - gettext
 - help2man (for or-tools)
 - libtool
 - make
 - python-setuptools
 - python-dev
 - subversion
 - texinfo (for makeinfo, needed by or-tools)
 - texlive (for or-tools)
 - zlib1g-dev

Furthermore, libv8-dev is required by the IW Script Engine.

To build the distribution, do not forget to fetch the hbba_base submodule like
this (from the root directory of this repository):

$ git submodule init; git submodule update

Then, the whole system should build from a single catkin_make.
However, especially if your Internet connection is unstable, the download of
some third-party dependencies pulled by or-tools might fail.
If it's the case, you can normally safely re-start the build process until
everything is downloaded correctly.
