#! /usr/bin/env python

import sferes
from waflib.Configure import conf

def build(bld):


    sferes.create_variants(bld,
		source='obstacle_avoidance.cpp',
                use = 'sferes2 fastsim',
                uselib = 'EIGEN SDL BOOST BOOST_GRAPH BOOST_THREAD TBB',
                target = 'obstacle_avoidance',
                cxxflags = bld.get_env()['CXXFLAGS'] + ['-Wno-unknown-pragmas'],
                variants = [
                                 'FIT1 ENVOA1',
                                # 'FIT1 ENVOA1 VISU',
                                 'FIT1 ENVOA2',
                                 'FIT1 ENVOA2 SAVEBMP',
                                # 'FIT1 ENVOA2 VISU',
                                 'FIT1 ENVOA3',
                                 'FIT3 ENVOA3 VISU SAVETRAJ',
                                #'FIT1 ENVOA3 VERBOSE',
                                # 'FIT2 ENVOA1',
                                # 'FIT2 ENVOA2',
                                # 'FIT2 ENVOA3',
                                # 'FIT3 ENVOA1',
                                # 'FIT3 ENVOA2',
                                # 'FIT3 ENVOA3',
                                       ])

    sferes.create_variants(bld,
		source='maze_navigation.cpp',
                use = 'sferes2 fastsim',
                uselib = 'EIGEN SDL BOOST BOOST_GRAPH BOOST_THREAD TBB',
                target = 'maze_navigation',
                cxxflags = bld.get_env()['CXXFLAGS'] + ['-Wno-unknown-pragmas'],
                variants = [
                            'STD LONG BIGSIZE',
                            'DIVERSITY LONG BIGSIZE',
                            'NOVELTY LONG BIGSIZE',
                            'STD VLONG VBIGSIZE',
                            'DIVERSITY VLONG VBIGSIZE',
                            'NOVELTY VLONG VBIGSIZE',
                            'STD FITDIST VLONG VBIGSIZE',
                            'DIVERSITY FITDIST VLONG VBIGSIZE',
                            'NOVELTY FITDIST VLONG VBIGSIZE',
                            'VISU',
                            'MAZE2',
                            'MAZE3',
                            'DIVERSITY',
                            'DIVERSITY MAZE2',
                            'DIVERSITY MAZE3',
                            'NOVELTY',
                            'NOVELTY SAVETRAJ',
                            'NOVELTY MAZE4 SAVETRAJ',
                            'NOVELTY MAZE2',
                            'NOVELTY MAZE3'
                    ])
