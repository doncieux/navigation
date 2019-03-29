#! /usr/bin/env python

import sferes
from waflib.Configure import conf

def build(bld):


    sferes.create_variants(bld,
		source='maze_navigation.cpp',
                use = 'sferes2 fastsim',
                uselib = 'EIGEN SDL BOOST BOOST_GRAPH BOOST_THREAD TBB',
                target = 'maze_navigation',
                cxxflags = bld.get_env()['CXXFLAGS'] + ['-Wno-unknown-pragmas'],
                variants = [
                    
                    ## VARIANTS TO CHECK THE UNIFORMITY OF THE SAMPLING IN THE BEHAVIOR SPACE
                    'NOVELTY ARENA1 VVBIGSIZE NOEXIT GONE',
                    'NOVELTY ARENA1 TOWARDSCORNER VVBIGSIZE NOEXIT GONE',
                    'NOVELTY ARENA1 TURNED VVBIGSIZE NOEXIT GONE',
                    'NOVELTY ARENA1 NOEXIT',
                    'NOVELTY ARENA1 TOWARDSCORNER NOEXIT',
                    'NOVELTY MAZEHARD LOWERLEFT NOEXIT',
                    'NOVELTY MAZEHARD LOWERLEFT VVBIGSIZE NOEXIT GONE',

                    ## VARIANTS TO MEASURE p1
                    'NOVELTY ARENA1 GOALAREA1 VLONG',
                    'NOVELTY ARENA1 GOALAREA2 VVLONG',
                    'NOVELTY ARENA1',
                    'NOVELTY MAZEHARD LOWERLEFT GOALAREA1 VLONG',
                    'NOVELTY MAZEHARD LOWERLEFT GOALAREA1 VLONG2',
                    'NOVELTY MAZEHARD LOWERLEFT GOALAREA1 VLONG2 NBPOS2',
                    'NOVELTY MAZEHARD LOWERLEFT GOALAREA1 VLONG2 NBPOS10',
                    'NOVELTY MAZEHARD LOWERLEFT GOALAREA1 VVLONG2 NBPOS2',
                    'NOVELTY MAZEHARD LOWERLEFT GOALAREA1 VVLONG2 NBPOS10',
                    'NOVELTY MAZEHARD LOWERLEFT GOALAREA1 VVLONG',
                    'NOVELTY MAZEHARD LOWERLEFT GOALAREA2 VVLONG',
                    'NOVELTY MAZEHARD LOWERLEFT',
                    'NOVELTY MAZEHARD LOWERLEFT LONG',
                    'NOVELTY MAZEHARD LOWERLEFT LONG NBPOS2',
                    'NOVELTY MAZEHARD LOWERLEFT LONG NBPOS10',
                    'NOVELTY MAZEHARD LOWERLEFT VLONG',
                    'NOVELTY MAZEHARD LOWERLEFT VVLONG',
                    

                ])
