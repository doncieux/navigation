Navigation
==========

This is the source code of two simple navigation experiments: an obstacle avoidance and a maze navigation. Neural networks are generated thanks to the sferes framework to solve these tasks. Several different fitness functions are provided for comparison as well as process helpers like behavioral diversity or novelty.

These experiments have illustrated tutorials given by Stephane Doncieux on selective pressures. This was part of a larger tutorial covering different topics in evolutionary robotics and given by Nicolas Bredeche, Jean-Baptiste Mouret and Stephane Doncieux at:
* Artificial Life conference, New York, 2014
* GECCO, Madrid, 2015
* ECAL, York 2015


Usage & installation
--------------------

### Dependencies:
sferes2 core (https://github.com/sferes2/sferes2)
sferes2 modules:
* fastsim (https://github.com/jbmouret/libfastsim for the core library and https://github.com/sferes2/fastsim to connect it to sferes)
* nn2 (https://github.com/sferes2/nn2)

### Installation
fastsim and nn2 need to be put in sferes2/modules.

In sferes2, create a file named modules.conf with this content:
    fastsim
    nn2

The navigation directory must be put in sferes2/exp.

Assuming every of those git repositories are in ~/git, it can be done like that:
    cd ~/git/sferes2/modules
    ln -s ~/git/fastsim
    ln -s ~/git/nn2
    cd ~/git/sferes2
    echo fastsim > modules.conf
    echo nn2 >> modules.conf
    cd ~/git/sferes2/exp
    ln -s ~/git/navigation    

To configure:
    ./waf configure

To compile:
    ./waf build --exp=navigation

### Executables

Created executables are in ~/git/sferes2/build/default/exp/navigation (or ~/git/sferes2/build/debug/exp/navigation for the executables with debug symbols).

Each executable has a long name made up with:
1. the name of the cpp file
2. the labels define

Labels allow to have different experiments sharing the same source file. Sferes includes a mechanism to decide what labels are defined for a particular experiment, see the wscript file. Let's suppose that we have built an experiment described in a my_exp.cpp with labels like FIT1, FIT2, ENV1 and ENV2. FIT1 correspond to a first definition of the fitness function and FIT2 is an alternative fitness function. ENV1 launches the experiment with the first environmental setup and ENV2 with the second one. my_exp.cpp contains then macros like:
    #ifdef FIT1
      // do whatever needed to compute FIT1 and take it into account
    #elif FIT2
      // do whatever needed to compute FIT1 and take it into account
    #else
    #error "You need to define either FIT1 or FIT2
    #endif


### Launching an experiment and looking at the results

To launch an experiment, simply launch the corresponding executable.



Author
------
- Stephane Doncieux stephane.doncieux@isir.upmc.fr: main author and maintainer

Other contributor
------------------
- Jean-Baptiste Mouret


See sferes2 main documentation for an overview of this software framework.



