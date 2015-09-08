#! /usr/bin/env python

import sferes


def build(bld):


    sferes.create_variants(bld,
		source='obstacle_avoidance.cpp',
                uselib_local = 'sferes2 fastsim',
                uselib = 'EIGEN2 SDL BOOST_GRAPH BOOST_THREAD TBB',
                target = 'obstacle_avoidance',
                cxxflags = bld.get_env()['CXXFLAGS'] + ['-Wno-unknown-pragmas'],
                json = 'obstacle_avoidance.json',
                variants = [
                                 'FIT1 ENVOA1',
                                # 'FIT1 ENVOA1 VISU',
                                 'FIT1 ENVOA2',
                                 'FIT1 ENVOA2 SAVEBMP',
                                # 'FIT1 ENVOA2 VISU',
                                 'FIT1 ENVOA3',
                                 'FIT1 ENVOA3 VISU',
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
                uselib_local = 'sferes2 fastsim',
                uselib = 'EIGEN2 SDL BOOST_GRAPH BOOST_THREAD TBB',
                target = 'maze_navigation',
                cxxflags = bld.get_env()['CXXFLAGS'] + ['-Wno-unknown-pragmas'],
                json = 'maze_navigation.json',
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
                            'NOVELTY MAZE2',
                            'NOVELTY MAZE3'
                    ])
