=============
OpenHumanoids
=============

**Private:**

.. image:: https://travis-ci.com/openhumanoids/oh-distro-private.svg?token=r3Tk8EVmRT74kcFqh9rX&branch=master
    :target: https://travis-ci.com/openhumanoids/oh-distro-private


**Public:**

.. image:: https://travis-ci.org/openhumanoids/oh-distro.svg?branch=master
    :target: https://travis-ci.org/openhumanoids/oh-distro

.. contents:: Table of Contents

Introduction
============

OpenHumanoids is the software platform developed by the MIT DRC team and
used to compete in successive phases of the DARPA Robotics Challenge
with the Boston Dynamics Atlas robot. It also full supports the NASA Valkyrie
and several fixed base arms.

This README describes how to download and build the OpenHumanoids source code
and how to satisfy 3rd party dependencies.

If you are looking for instructions specific to the public open source release, 
please supplement the following instructions on this page with the additional
instructions `here <additional_instructions_open_source_release.rst>`_.


Background
----------

.. image:: http://img.youtube.com/vi/FiH4pId-zFM/0.jpg
   :target: https://www.youtube.com/watch?v=FiH4pId-zFM

`MIT DRC youtube video <https://www.youtube.com/watch?v=FiH4pId-zFM>`_


System Requirements
-------------------

These instructions are written for Ubuntu 14.04 Trusty 64-bit.


Download Instructions
=====================

Install Git
-----------

The DRC source code is stored in a Git repository. To download the
source code you may need to first install Git on your system:

::

    sudo apt-get install git gitk git-gui


Getting Access
--------------

You may need permission to access the git repository hosted on GitHub. To
do so, create a GitHub account at `GitHub.com <https://github.com>`_ if
you don't already have one.

Next, add your public SSH key to your GitHub account so that you can easily
push and pull over SSH.  Read the `generating ssh keys <https://help.github.com/articles/generating-ssh-keys>`_
article for instructions to generate and link an ssh key to your account.

Download the source code
------------------------

Download the repository with the ``git clone`` command and cd into the distro:

::

    git clone git@github.com:openhumanoids/oh-distro-private.git && cd oh-distro-private


Users of the open source release need to remote ``-private`` from the URL and directory name
and also de-initialize some of our submodules as noted in the 
`supplementary instructions <additional_instructions_open_source_release.rst>`_.

Initialize the submodules (Drake, director, pronto):

::

    git submodule update --init --recursive

If you're an OpenHumanoids organization member, add the *sandbox* remote. The *sandbox* is the
location where branches can be shared and deactivate the origin remote.

::

    git remote set-url --push origin no_push
    git remote add sandbox git@github.com:oh-dev/oh-distro-private.git
    git fetch sandbox


Dependencies
============


Required Packages
-----------------
There are required 3rd party dependencies that can be satisfied by
installing packages on Ubuntu. Install with the following commands:

::

    sudo apt-get update

    sudo apt-get install build-essential cmake debhelper freeglut3-dev gtk-doc-tools libboost-filesystem-dev libboost-iostreams-dev libboost-program-options-dev libboost-random-dev libboost-regex-dev libboost-signals-dev libboost-system-dev libboost-thread-dev libcurl4-openssl-dev libfreeimage-dev libglew-dev libgtkmm-2.4-dev libltdl-dev libgsl0-dev libportmidi-dev libprotobuf-dev libprotoc-dev libqt4-dev libqwt-dev libtar-dev libtbb-dev libtinyxml-dev libxml2-dev ncurses-dev pkg-config protobuf-compiler python-matplotlib libvtk5.8 libvtk5-dev libvtk5-qt4-dev libqhull-dev python-pygame doxygen mercurial libglib2.0-dev python-dev gfortran f2c libf2c2-dev spacenavd libspnav-dev python-numpy python-scipy python-yaml python-vtk python-pip libgmp3-dev libblas-dev liblapack-dev libv4l-dev subversion libxmu-dev libusb-1.0-0-dev python-pymodbus graphviz curl libwww-perl libterm-readkey-perl libx264-dev libopenni-dev swig libqglviewer-dev libsuitesparse-dev libsdl1.2-dev


Java 8
------
You will also need Java 8 which you can install via:

::

    sudo add-apt-repository ppa:openjdk-r/ppa
    sudo apt-get update 
    sudo apt-get install openjdk-8-jdk

If you previously had another version of Java installed make sure to configure the alternatives 
for ``java`` and ``javac`` to use the correct version of Java:

::

    sudo update-alternatives --config java
    sudo update-alternatives --config javac

If required, further information can be found `here <http://ubuntuhandbook.org/index.php/2015/01/install-openjdk-8-ubuntu-14-04-12-04-lts/>`_.

gcc/g++ 4.9
-----------
More recent Matlab and Drake versions require C++14 support for which you will need to install gcc/g++ 4.9:

::
    
    sudo add-apt-repository ppa:ubuntu-toolchain-r/test
    sudo apt-get update
    sudo apt-get install gcc-4.9 g++-4.9
    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.9 60 --slave /usr/bin/g++ g++ /usr/bin/g++-4.9

CMake 3.5
---------
Some of our dependencies require a newer CMake than the system-provided 2.8.12. You can install a local copy and add it to your path by running:

::

    mkdir -p ~/tools
    cd ~/tools
    wget https://cmake.org/files/v3.5/cmake-3.5.2-Linux-x86_64.tar.gz
    tar zxvf cmake-3.5.2-Linux-x86_64.tar.gz
    rm cmake-3.5.2-Linux-x86_64.tar.gz
    cd cmake-3.5.2-Linux-x86_64/bin
    echo "export PATH=`pwd`:\$PATH" >> ~/.bashrc

LCM Dependency
--------------

LCM (v1.3.1) is a required dependency which must be installed from source. Further instructions and tutorials are available `here <http://lcm-proj.github.io/>`_.

::

    wget https://github.com/lcm-proj/lcm/releases/download/v1.3.1/lcm-1.3.1.zip
    unzip lcm-1.3.1.zip
    cd lcm-1.3.1
    ./configure
    make
    sudo make install


Install Matlab
--------------

Download Matlab R2016a from `MathWorks <https://mathworks.com/downloads/>`_. Unzip the file you just downloaded (e.g., unzip ./R2016a-linux64.zip)
cd into the resulting directory and run ``sudo ./install``.

When prompted for how to install, choose "Log in with a MathWorks Account."

Choose a "Typical" install and click next through the rest of the process. You will need to enter your Mathworks username and password during the install process, and you should see a single license that you can use for the install (this comes from a lookup of the activation key).

After installing MATLAB, two of the symlinks for libraries need to be changed:

::

   cd /usr/local/MATLAB/R2016a/sys/os/glnxa64
   ls -l

The symbolic links for libstdc++.so.6 and libgfortran.so.3 should point to versions in /usr/lib, not local ones.

Now, modify the symlinks:

::

   sudo rm libgfortran.so.3
   sudo ln -s /usr/lib/x86_64-linux-gnu/libgfortran.so.3.0.0 libgfortran.so.3
   sudo rm libstdc++.so.6
   sudo ln -s /usr/lib/gcc/x86_64-linux-gnu/4.4/libstdc++.so libstdc++.so.6

Instructions for MOSEK
----------------------

Mosek is a solver used in the footstep planner. Obtain an academic licence from `Mosek <http://license.mosek.com/academic>`_.
Check your email and place your license in ``~/mosek/mosek.lic``.
The Mosek code is checked out as part of the project externals.


Build Instructions
==================

Environment Setup
-----------------

The behavior of certain build steps can be affected by environment
variables, so you should setup your environment before starting the
build. The environment is setup by sourcing the file
*oh-distro-private/software/config/drc\_environment.sh*. Typically, users will source
this file automatically in their ~/.bashrc file by adding this line to
~/.bashrc:

::

    source /path-to/oh-distro-private/software/config/drc_environment.sh

If you have already done this, make sure your ~/.bashrc contains the
correct path to the drc\_environment.sh file in the oh-distro source code
directory that you just cloned with git.

Matlab Environment Setup
------------------------

Create a file ~/Documents/MATLAB/startup.m that contains the line:

::

    run([getenv('DRC_BASE'), '/software/build/config/drc_control_setup.m'])




Compiling
---------

Make sure you have sourced the drc\_environment.sh file to setup the DRC
environment prior to building. If you did not source the file
automatically in ~/.bashrc, then do so now with the following command:

::

    cd oh-distro-private
    source software/config/drc_environment.sh

If you are a member of the OpenHumanoids organization, run make to build externals and then the main codebase:

::

    cd software/externals
    make -j
    cd ..
    make -j


Instructions for GUROBI
-----------------------

Gurobi is a solver used in our walking controller. Install its dependencies with the following commands:

::

    sudo apt-get install curl libwww-perl libterm-readkey-perl

Then generate an academic licence: First make an account
http://www.gurobi.com/download/licenses/free-academic , then use the Gurobi
key client (grbgetkey) to store the license on your machine. Place it in the suggested
location (~/gurobi.lic)

The grbgetkey module is built as part of the externals.

Note that the tarball for Gurobi is part of our tree and the gurobi pod uses it
to avoid needing to download it from Gurobi.

ROS
===

ROS is not required per se. If you would like to use this distribution in conjunction with IHMC's SCS, your own controllers for Valkyrie, or to use EXOTica for planning and optimization, please install ROS Indigo including MoveIt and ROS-Control. Valkyrie uses ROS-Control for the Hardware API and our LCM2ROSControl translator package hence requires ROS Control.

You may need to follow the ROS Indigo installation instructions available `here <http://wiki.ros.org/indigo/Installation/Ubuntu>`_
to set up the correct sources, keys, and ``.bashrc`` parameters.

::

    sudo apt-get install ros-indigo-desktop-full ros-indigo-moveit-full ros-indigo-ros-control ros-indigo-hardware-interface ros-indigo-controller-interface ros-indigo-joint-limits-interface python-catkin-tools

Initialize catkin workspace (only required once):
::

    cd $DRC_BASE/catkin_ws
    rm -rf devel/ build/ logs/
    catkin config --init --mkdirs --no-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS=-std=c++11
    # Additional command on any workstation that does not extend other workspaces:
    catkin config --extend /opt/ros/indigo
    # Additional command for vis04:
    catkin config --extend ~/val_ws/install

Compile catkin workspace:

::

    cd $DRC_BASE/catkin_ws
    catkin build -s

Before you run any ROS code from the catkin workspace, source the setup script:

::

    source catkin_ws/devel/setup.bash

This may be done by NASA scripts on vis04. If combining multiple workspaces, extend them in a sequence and source the setup script of the top-most workspace only.

Using system-installed PCL and OpenCV
=====================================
PCL (version 1.7.1) and OpenCV (~2.4.12.2) can also be system-installed instead of being built from source in the externals build step resulting in considerable build time savings. In order to do this:

Add a PPA and install PCL:

::
    
    sudo add-apt-repository -y ppa:v-launchpad-jochen-sprickerhof-de/pcl
    sudo apt-get update
    sudo apt-get install -y libpcl-1.7-all

Use e.g. our in-house packaged version of OpenCV 2.4.12.3 that matches the one being built from source as part of the externals build step:

::
    
    wget http://terminator.robots.inf.ed.ac.uk/public/opencv_2.4.12.2.deb
    sudo dpkg -i opencv_2.4.12.2.deb

Note: Using the in-house packaged version of OpenCV requires ``libdc1394-22-dev`` to be installed (``sudo apt-get install libdc1394-22-dev``).

These options are disabled by default on purpose. Then, to make use of the system-installed versions, configure the externals build with the corresponding flags:

::
    
    cd software/externals
    mkdir pod-build && cd pod-build
    cmake .. -DUSE_SYSTEM_PCL:BOOL=ON -DUSE_SYSTEM_OPENCV:BOOL=ON -DCMAKE_INSTALL_PREFIX:STRING=$DRC_BASE/software/build


Using externally installed Gurobi
=================================
If installing your own version of Gurobi, please add the following to your .bashrc file before compiling oh-distro (and after sourcing drc_environment.sh since these variables will also be set when sourcing our drc_environment config):

::

    export GUROBI_HOME=[PATH_TO]/gurobi/gurobi562/linux64
    export PATH=$PATH:$GUROBI_HOME/bin
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$GUROBI_HOME/lib
    export GRB_LICENSE_FILE=$HOME/gurobi.lic


Build Issues FAQ
================
ISSUE: make in externals failed:

* REASON: A submodule has been updated
* RESOLUTION:

  * retry (``make -j1``) and see which module failed
  * remove the relevent module from pod build: ``rm -rf pod-build/src/[module] pod-build/tmp/[module]``
  * continue making externals

ISSUE: drc_lcmtypes fails to build showing something like:

* .../oh-distro/software/drc_lcmtypes/lcmtypes/drc_robot_command_t.lcm: No such file or directory
* make[3]: *** [CMakeFiles/lcmgen_c] Error 255
* REASON: An LCM type has been removed or added.
* RESOLUTION:

  * make clean in drc_lcmtypes and then continue

ISSUE: drake won't build. 

* REASON: Drake is specially configured to build with Open Humanoids. Never make in software/drake, always make in software/drake/drake/
* RESOLUTION: Make a clean build of drake:

::

    cd <path-to>/oh-distro/software
    rm drake
    cd externals
    rm pod-build/src/drake-cmake-* pod-build/tmp/drake-cmake-* -Rf
    git submodule update --init --recursive
    cd externals
    make -j 1
    cd software/drake/drake
    make -j
