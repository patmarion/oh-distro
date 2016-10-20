===================================
Warning: Don't expect this to work!
===================================

We're releasing most of the source code from the MIT DRC project in
the hope that it will benefit the robotics community. But there are
parts of this software, like the Boston Dynamics Atlas software
interface, which we are not allowed to release publicly. As a result,
this public repo is *incomplete*. Some of the submodules and external
projects are *private*, and you won't be able to access them unless
you're a member of the team. Sorry!

We're actively working on making this a project that can be used by
people outside the group, but for now, you should consider it a
collection of (potentially) interesting code, not a functional
application.

The core algorithms and tools, however, live in their own projects
which are much better supported:

* Drake (planning, control, simulation, optimization): http://drake.mit.edu
* Pronto (state estimation): https://github.com/mitdrc/pronto
* Director (user interface): https://github.com/RobotLocomotion/director


==============================================
OpenHumanoids Public Installation Instructions
==============================================


These instructions are targeted for the public release and may likely be broken. They supplement the instructions given in the main README file.

Download the source code
------------------------

Download the repository with the ``git clone`` command and cd into the distro:

::

    git clone git@github.com:openhumanoids/oh-distro.git && cd oh-distro

If you are **not** a member of the OpenHumanoids organization, please deinit one private submodule or else the following command will fail:

::

    git submodule deinit catkin_ws/src/exotica-dev && git rm catkin_ws/src/exotica-dev

If you do not have access to private external submodules such as Gurobi, Snopt, or the Atlas drivers, you need to turn off BUILD_PRIVATE_EXTERNALS:

::

    cd oh-distro/software/externals
    mkdir pod-build && cd pod-build
    cmake .. -DBUILD_PRIVATE_EXTERNALS:BOOL=OFF -DCMAKE_INSTALL_PREFIX:STRING=$DRC_BASE/software/build
    cd ..
    make -j
    cd ..
    make -j

Please make sure to install Gurobi and Snopt manually.
