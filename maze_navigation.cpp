//| This file is a part of an experiment relying on the sferes2 framework.
//| Copyright 2015, ISIR / Universite Pierre et Marie Curie (UPMC)
//| Main contributor(s):
//|   * Stephane Doncieux, stephane.doncieux@isir.upmc.fr
//|   * Jean-Baptiste Mouret, mouret@isir.upmc.fr (sferes framework)
//|
//| This experiment allows to generate neural networks for simple
//| navigation tasks (obstacle avoidance and maze navigation).
//|
//| This software is governed by the CeCILL license under French law
//| and abiding by the rules of distribution of free software.  You
//| can use, modify and/ or redistribute the software under the terms
//| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
//| following URL "http://www.cecill.info".
//|
//| As a counterpart to the access to the source code and rights to
//| copy, modify and redistribute granted by the license, users are
//| provided only with a limited warranty and the software's author,
//| the holder of the economic rights, and the successive licensors
//| have only limited liability.
//|
//| In this respect, the user's attention is drawn to the risks
//| associated with loading, using, modifying and/or developing or
//| reproducing the software by the user in light of its specific
//| status of free software, that may mean that it is complicated to
//| manipulate, and that also therefore means that it is reserved for
//| developers and experienced professionals having in-depth computer
//| knowledge. Users are therefore encouraged to load and test the
//| software's suitability as regards their requirements in conditions
//| enabling the security of their systems and/or data to be ensured
//| and, more generally, to use and operate it in the same conditions
//| as regards security.
//|
//| The fact that you are presently reading this means that you have
//| had knowledge of the CeCILL license and that you accept its terms.

#ifdef NO_PARALLEL
# warning no parallel.
#endif

#include <sferes/phen/parameters.hpp>

#ifdef USE_SDL
#include <SDL.h>
#endif

#include <Eigen/Core>

#include <modules/nn2/mlp.hpp>
#include "phen_dnn.hpp"

#include <sferes/run.hpp>
#include <sferes/stc.hpp>
#include <sferes/misc.hpp>

#include <sferes/gen/evo_float.hpp>
#include <sferes/ea/nsga2.hpp>

#include <sferes/fit/fitness.hpp>
#include <sferes/eval/parallel.hpp>
#include <sferes/eval/eval.hpp>
#include <sferes/stat/pareto_front.hpp>
#include <sferes/stat/best_fit.hpp>
#include <sferes/modif/dummy.hpp>

#include <modules/fastsim/simu_fastsim.hpp>

#ifdef DIVERSITY
#include "modifier_diversity.hpp"
#endif

#ifdef NOVELTY
#include "modifier_novelty.hpp"
#include "stat_bd.hpp"
#endif

#include "stat_success.hpp"

using namespace sferes;
using namespace sferes::gen::evo_float;
using namespace sferes::gen::dnn;
using namespace fastsim;
using namespace nn;

#ifdef SAVETRAJ
#include "stat_traj.hpp"
#endif

struct Params
{
  struct dnn
  {
    static constexpr size_t nb_inputs       = 3+1; // laser + cst input
    static constexpr size_t nb_outputs      = 2; // 2 motors: left and right wheel
    static constexpr size_t min_nb_neurons  = 0;
    static constexpr size_t max_nb_neurons  = 30;
    static constexpr size_t min_nb_conns    = 8;
    static constexpr size_t max_nb_conns    = 250;

    static constexpr int io_param_evolving = true;
    static constexpr float m_rate_add_conn	= 0.1f;
    static constexpr float m_rate_del_conn	= 0.01f;
    static constexpr float m_rate_change_conn = 0.1f;
    static constexpr float m_rate_add_neuron  = 0.1f;
    static constexpr float m_rate_del_neuron  = 0.01f;

    static constexpr init_t init = ff;
  };

  struct evo_float
  {
    static constexpr mutation_t mutation_type = polynomial;
    //static constexpr cross_over_t cross_over_type = sbx;
    static constexpr cross_over_t cross_over_type = no_cross_over;
    static constexpr float cross_rate = 0.0f;
    static constexpr float mutation_rate = 0.1f;
    static constexpr float eta_m = 15.0f;
    static constexpr float eta_c = 10.0f;
  };

  struct pop
  {
#ifdef BIGSIZE
    static constexpr unsigned size = 200;
#elif defined (VBIGSIZE)
    static constexpr unsigned size = 400;
#elif defined (VVBIGSIZE)
    static constexpr unsigned size = 10000;
#else
    static constexpr unsigned size = 100;
#endif
#ifdef LONG
    static constexpr unsigned nb_gen = 4001;
#elif defined (VLONG)
    static constexpr unsigned nb_gen = 10001;
#elif defined (VLONG2)
    static constexpr unsigned nb_gen = 20001;
#elif defined (VVLONG)
    static constexpr unsigned nb_gen = 100001;
#elif defined (VVLONG2)
    static constexpr unsigned nb_gen = 200001;
#elif defined (G100)
    static constexpr unsigned nb_gen = 101;
#elif defined (GONE)
    static constexpr unsigned nb_gen = 1;
#else
    static constexpr unsigned nb_gen = 1001;
#endif
    static constexpr int dump_period = 50;
    static constexpr int initial_aleat = 1;
  };

  struct parameters
  {
    static constexpr float min = -5.0f;
    static constexpr float max = 5.0f;
  };

  struct simu
  {
    static constexpr int laser_range     = 100.0f;
    //Evalutations
    static constexpr float nb_steps = 2000;

    static constexpr float dt =0.01;


#ifdef MAZE2
    SFERES_STRING(map_name, SFERES_ROOT "/exp/navigation/maze2.pbm");
#elif defined(MAZE3)
    SFERES_STRING(map_name, SFERES_ROOT "/exp/navigation/maze3.pbm");
#elif defined(MAZE4)
    SFERES_STRING(map_name, SFERES_ROOT "/exp/navigation/maze4.pbm");
#elif defined(ARENA1)
    SFERES_STRING(map_name, SFERES_ROOT "/exp/navigation/arena1.pbm");
#elif defined(MAZEHARD)
    SFERES_STRING(map_name, SFERES_ROOT "/exp/navigation/maze_hard.pbm");
#else
    SFERES_STRING(map_name, SFERES_ROOT "/exp/navigation/maze.pbm");
#endif
  };

  struct fitness
  {
#ifdef MAZEHARD

#ifdef GOALAREA1
    // for a theoretical p1 = 10e-4
    // Expressed in percentage of map size
    static constexpr float min_x1=0.1;
    static constexpr float max_x1=0.11;
    static constexpr float min_y1=0.1;
    static constexpr float max_y1=0.11;

    static constexpr float min_x2=0.1;
    static constexpr float max_x2=0.11;
    static constexpr float min_y2=0.30;
    static constexpr float max_y2=0.31;

    static constexpr float min_x3=0.1;
    static constexpr float max_x3=0.11;
    static constexpr float min_y3=0.75;
    static constexpr float max_y3=0.76;
#elif defined(GOALAREA2)
    // for a theoretical p1 = 10e-6
    // Expressed in percentage of map size
    static constexpr float min_x1=0.1;
    static constexpr float max_x1=0.101;
    static constexpr float min_y1=0.1;
    static constexpr float max_y1=0.101;

    static constexpr float min_x2=0.1;
    static constexpr float max_x2=0.101;
    static constexpr float min_y2=0.30;
    static constexpr float max_y2=0.301;

    static constexpr float min_x3=0.1;
    static constexpr float max_x3=0.011;
    static constexpr float min_y3=0.75;
    static constexpr float max_y3=0.751;
#else
    // for a theoretical p1 = 10e-2
    // Expressed in percentage of map size
    static constexpr float min_x1=0.1;
    static constexpr float max_x1=0.2;
    static constexpr float min_y1=0.1;
    static constexpr float max_y1=0.2;
    
    static constexpr float min_x2=0.1;
    static constexpr float max_x2=0.2;
    static constexpr float min_y2=0.30;
    static constexpr float max_y2=0.40;

    static constexpr float min_x3=0.1;
    static constexpr float max_x3=0.2;
    static constexpr float min_y3=0.75;
    static constexpr float max_y3=0.85;

#endif
    
#else

#ifdef GOALAREA1
    // for a theoretical p1 = 10e-4
    // Expressed in percentage of map size
    static constexpr float min_x1=0.9;
    static constexpr float max_x1=0.91;
    static constexpr float min_y1=0.9;
    static constexpr float max_y1=0.91;

    static constexpr float min_x2=0.9;
    static constexpr float max_x2=0.91;
    static constexpr float min_y2=0.1;
    static constexpr float max_y2=0.11;

    static constexpr float min_x3=0.1;
    static constexpr float max_x3=0.11;
    static constexpr float min_y3=0.1;
    static constexpr float max_y3=0.11;
#elif defined(GOALAREA2)
    // for a theoretical p1 = 10e-6
    // Expressed in percentage of map size
    static constexpr float min_x1=0.9;
    static constexpr float max_x1=0.901;
    static constexpr float min_y1=0.9;
    static constexpr float max_y1=0.901;

    static constexpr float min_x2=0.9;
    static constexpr float max_x2=0.901;
    static constexpr float min_y2=0.1;
    static constexpr float max_y2=0.101;

    static constexpr float min_x3=0.1;
    static constexpr float max_x3=0.101;
    static constexpr float min_y3=0.1;
    static constexpr float max_y3=0.101;
#else
    // for a theoretical p1 = 10e-2
    // Expressed in percentage of map size
    static constexpr float min_x1=0.85;
    static constexpr float max_x1=0.95;
    static constexpr float min_y1=0.85;
    static constexpr float max_y1=0.95;

    static constexpr float min_x2=0.85;
    static constexpr float max_x2=0.95;
    static constexpr float min_y2=0.15;
    static constexpr float max_y2=0.25;

    static constexpr float min_x3=0.15;
    static constexpr float max_x3=0.25;
    static constexpr float min_y3=0.15;
    static constexpr float max_y3=0.25;
#endif

#endif
    
  };

#ifdef NOVELTY
  struct novelty
  {
    static constexpr unsigned int k = 15; //nb neighbors
    static constexpr unsigned int max_archive_size = 50000; //Max archive size

#ifdef NBPOS2
    static constexpr unsigned int nb_pos = 2;
#elif defined(NBPOS10)
    static constexpr unsigned int nb_pos = 10;
#else
    static constexpr unsigned int nb_pos = 1; //nb of pos in the behavior descriptor (1=final position only, this is the descriptor used by Lehman and Stanley in their paper on Novelty search)
#endif
  };
#endif

};


int success1_so_far=0;
int success2_so_far=0;
int success3_so_far=0;

namespace sferes
{

  // ********** Main Class ***********
  SFERES_FITNESS(FitMazeNavigation, sferes::fit::Fitness)
  {
  public:
    FitMazeNavigation():nb_coll(0), time(0), speed(0), lin_speed(0), success1(0), success3(0),stop_eval(false) { }

    // *************** _eval ************
    //
    // This is the main function to evaluate the individual
    // It runs fastsim (simu_t simu)
    //
    // **********************************
    template<typename Indiv>
      void eval(Indiv& ind)
    {
      ind.nn().simplify();

      nb_coll=0;
      speed=0;
      lin_speed=0;
      stop_eval=false;
#ifdef VERBOSE
	std::cout<<"Eval ..."<<std::endl;
#endif


      typedef simu::Fastsim<Params> simu_t;
      simu_t simu;
      assert(simu.map()!=NULL);

      // init

      init_simu(simu);
      ind.nn().init();

#ifdef SAVETRAJ
      std::ostringstream straj;
      straj<<"# map size "<<simu.map()->get_real_w()<<" "<<simu.map()->get_real_h()<<std::endl;
      straj<<"# "<<Params::simu::map_name()<<std::endl;
#endif


      time=0;

      success1=0;
      success2=0;
      success3=0;
      size_t i;
      // *** Main Loop ***
      for (i = 0; i < Params::simu::nb_steps && !stop_eval;)
	{
	  
	  // Number of steps the robot is evaluated
	  time++;
	  
	  // Update robot info & caracs
	  simu.refresh();
#ifdef VISU
	  if (1) {
#elif defined(NO_VISU)
	  if (0) {
#else
	  if (this->mode() == fit::mode::view) {
#endif
	    simu.refresh_view();
#ifdef SAVEBMP
	    // WARNING: use with caution as it will generate many BMP...
	    std::ostringstream os;
	    os<<"img_"<<std::setfill('0')<<std::setw(6)<<time<<".bmp";
	    std::cout<<"Saving image: "<<os.str()<<std::endl;
	    if (simu.display().save_BMP(os.str().c_str())!=0) {
	      std::cerr<<"ERROR, can't save file: "<<os.str()<<std::endl;
	    }
	    
#endif
	  }
	  
	  // Get inputs
	  get_inputs(simu);
	  
	  // Step  neural network -- outf is the output vector.
	  step_check(ind.nn());
	  
	  // move the robot and check for collision and if is still
	  move_check(simu);
	  
#ifdef SAVETRAJ
	  straj<<simu.robot().get_pos().get_x()<<" "<<simu.robot().get_pos().get_y()<<" "<<simu.robot().get_pos().theta()<<std::endl;
#endif
	  
	  
#ifdef NOVELTY
	  if ((i>0)&&(i%(int)(Params::simu::nb_steps/Params::novelty::nb_pos)==0))
	    pos_bd.push_back(simu.robot().get_pos());
#endif
	  
	  // loop forever if we are in the visualization mode
	  if (this->mode() != fit::mode::view)
	    i++;

	  }
	  
#ifdef NOVELTY
	  for (unsigned int j=pos_bd.size();j<Params::novelty::nb_pos;j++)
	    pos_bd.push_back(simu.robot().get_pos());
#endif

	  
	  end_pos=simu.robot().get_pos();
	  
	  // Compute the fitness value
#if defined(DIVERSITY) //|| defined(NOVELTY)
	  this->_objs.resize(2);
#endif
	  
#ifndef NOEXIT
	  if ((simu.robot().get_pos().get_x()>=simu.map()->get_real_w()*Params::fitness::min_x1)
	      &&(simu.robot().get_pos().get_x()<=simu.map()->get_real_w()*Params::fitness::max_x1)
	      &&(simu.robot().get_pos().get_y()>=simu.map()->get_real_h()*Params::fitness::min_y1)
	      &&(simu.robot().get_pos().get_y()<=simu.map()->get_real_h()*Params::fitness::max_y1)) {
	    //std::cout<<"The robot has found the goal:"<<simu.robot().get_pos().get_x()<<" "<<simu.robot().get_pos().get_y()<<std::endl;
	    success1=1;
	    success1_so_far+=1; // WARNING: not significant if individuals are reevaluated (which is the case for NSGA-2, for instance)
	  }
	  else {
	    //std::cout<<"Goal not found"<<std::endl;
	  }
	  if ((simu.robot().get_pos().get_x()>=simu.map()->get_real_w()*Params::fitness::min_x2)
	      &&(simu.robot().get_pos().get_x()<=simu.map()->get_real_w()*Params::fitness::max_x2)
	      &&(simu.robot().get_pos().get_y()>=simu.map()->get_real_h()*Params::fitness::min_y2)
	      &&(simu.robot().get_pos().get_y()<=simu.map()->get_real_h()*Params::fitness::max_y2)) {
	    //std::cout<<"The robot has found the goal:"<<simu.robot().get_pos().get_x()<<" "<<simu.robot().get_pos().get_y()<<std::endl;
	    success2=1;
	    success2_so_far+=1; // WARNING: not significant if individuals are reevaluated (which is the case for NSGA-2, for instance)
	  }
	  else {
	    //std::cout<<"Goal not found"<<std::endl;
	  }
	  if ((simu.robot().get_pos().get_x()>=simu.map()->get_real_w()*Params::fitness::min_x3)
	      &&(simu.robot().get_pos().get_x()<=simu.map()->get_real_w()*Params::fitness::max_x3)
	      &&(simu.robot().get_pos().get_y()>=simu.map()->get_real_h()*Params::fitness::min_y3)
	      &&(simu.robot().get_pos().get_y()<=simu.map()->get_real_h()*Params::fitness::max_y3)) {
	    //std::cout<<"The robot has found the goal:"<<simu.robot().get_pos().get_x()<<" "<<simu.robot().get_pos().get_y()<<std::endl;
	    success3=1;
	    success3_so_far+=1;  // WARNING: not significant if individuals are reevaluated (which is the case for NSGA-2, for instance)
	  }
	  else {
	    //std::cout<<"Goal not found"<<std::endl;
	  }
#endif
	  
	  /*	  
#if defined(FITDIST)
	  this->_objs[0] = -end_pos.dist_to(goal);
	  this->_value = this->_objs[0] ;
#else
	  this->_objs[0] = success;
	  this->_value = success;
#endif
	  */
	  
	  //std::cout<<"End_pos | "<<end_pos.get_x()<<" "<<end_pos.get_y()<<" | "<<end_pos.get_x()/simu.map()->get_real_w()<<" "<<end_pos.get_y()/simu.map()->get_real_h()<<std::endl;
	  
	  
#ifdef VERBOSE
	  static int nbeval=0;
	  std::cout<<"fit="<<this->_objs[0]<<" nbeval="<<nbeval<<std::endl;
	  nbeval++;
#endif
	  
#ifdef SAVETRAJ
	  traj=straj.str();
#endif
	  
	  
    } // *** end of eval ***


    template<typename Simu>
      void init_simu(Simu& simu)
    {


      this->_objs.resize(1);

      //Visualisation mode
#ifdef VISU
	  simu.init_view(true);
#elif !defined(NO_VISU)
	if(this->mode() == fit::mode::view)
	  simu.init_view(true);
#endif

      simu.init();

      // Adding robot sensors (no need to add effectors):
      // 3 lasers range sensors
      //right
      simu.robot().add_laser(Laser(M_PI / 4.0, 8.f*simu.robot().get_radius()*2.f));
      // left
      simu.robot().add_laser(Laser(-M_PI / 4.0, 8.f*simu.robot().get_radius()*2.f));
      //middle
      simu.robot().add_laser(Laser(0.0f, 8.f*simu.robot().get_radius()*2.f));

      old_pos=simu.robot().get_pos();
      inputs.resize(Params::dnn::nb_inputs);

#ifdef TOWARDSCORNER
      simu.robot().set_pos(Posture(simu.map()->get_real_w()*0.1,simu.map()->get_real_w()*0.1, -3.*M_PI/4.0));
#elif TURNED
      simu.robot().set_pos(Posture(simu.map()->get_real_w()*0.1,simu.map()->get_real_w()*0.1, -M_PI/4.0));
#elif LOWERLEFT
      simu.robot().set_pos(Posture(simu.map()->get_real_w()*0.1,simu.map()->get_real_w()*0.9, -M_PI/2.0));
#else
      simu.robot().set_pos(Posture(simu.map()->get_real_w()*0.1,simu.map()->get_real_w()*0.1, M_PI/4.0));
#endif
      simu.robot().move(0,0,simu.map());

    }




    // *** Get sensors inputs
    template<typename Simu>
      void get_inputs(Simu &simu)
    {
      // Update of the sensors
      size_t nb_lasers = simu.robot().get_lasers().size();

      // *** set inputs ***

      // inputs from sensors
      for (size_t j = 0; j < nb_lasers; ++j)
	{
	  float d = simu.robot().get_lasers()[j].get_dist();
	  float range = simu.robot().get_lasers()[j].get_range();
	  inputs[j] = (d == -1 ? 0 : 1 - d / range);
	}

      inputs[nb_lasers]=1;

    }

    // *** Step Neural Network and various checks
    template<typename NN>
      void step_check(NN &nn)
    {
      nn.step(inputs);
      outf.resize(nn.get_outf().size());
      assert(nn.get_outf().size() == 2);

      for(size_t j = 0; j < nn.get_outf().size(); j++)
	if(std::isnan(nn.get_outf()[j]))
	  outf[j] = 0.0;
	else
	  outf[j]=4*(2*nn.get_outf()[j]-1); // to put nn values in the interval [-4;4] instead of [0;1]

      //std::cout<<"Outf: "<<nn.get_outf()[0]<<" "<<nn.get_outf()[1]<<std::endl;

    }


    // *** Move and check if robot is colliding, or still
    template<typename Simu>
      void move_check(Simu &simu)
    {
      // *** move robot ***
      simu.move_robot(outf[0], outf[1]);


      // *** To save simulation time, we stop evaluation if the robot is stuck for more than 100 time steps ***
      if ((old_pos.dist_to(simu.robot().get_pos())<0.0001)&&
	  (fabs(old_pos.theta()-simu.robot().get_pos().theta())<0.0001)) {
	stand_still++;
	if (stand_still>100) {
	  stop_eval=true;
#ifdef VERBOSE
	  std::cout<<"Still robot, we stop the eval..."<<std::endl;
#endif
	  // We add collisions to be fair and avoid side effects
	  if (simu.robot().get_collision())
	    nb_coll+=Params::simu::nb_steps-time;
	}
      }
      else {
	if (simu.robot().get_collision()) {
	  nb_coll++;
	}
      }

      old_pos=simu.robot().get_pos();
    }

    float width, height, fit;
    int nb_coll, time;
    float speed, lin_speed;
    unsigned int stand_still;
    fastsim::Posture old_pos,end_pos;
#ifdef NOVELTY
    std::vector<fastsim::Posture> pos_bd; // behavior descriptor based on the position
#endif

    int success1, success2, success3;
    int get_success1_so_far(void) {return success1_so_far;}
    int get_success2_so_far(void) {return success2_so_far;}
    int get_success3_so_far(void) {return success3_so_far;}
    bool stop_eval;                                  // Stops the evaluation
    std::vector<float> outf, inputs;

#ifdef SAVETRAJ
  std::string traj;
#endif

  };

}


// ****************** Main *************************
int main(int argc, char **argv)
{
  srand(time(0));

  typedef FitMazeNavigation<Params> fit_t;

  typedef phen::Parameters<gen::EvoFloat<1, Params>, fit::FitDummy<>, Params> weight_t;
  typedef phen::Parameters<gen::EvoFloat<1, Params>, fit::FitDummy<>, Params> bias_t;

  typedef PfWSum<weight_t> pf_t;
  typedef phen::Parameters<gen::EvoFloat<4, Params>, fit::FitDummy<>, Params> node_label_t;
  typedef AfSigmoidBias<bias_t> af_t;
  typedef Neuron<pf_t, af_t >  neuron_t;
  typedef Connection <weight_t> connection_t;
  typedef sferes::gen::Dnn< neuron_t, connection_t, Params> gen_t;
  typedef phen::Dnn<gen_t, fit_t, Params> phen_t;


  typedef eval::Parallel<Params> eval_t;
  // STATS
  typedef boost::fusion::vector<
    sferes::stat::ParetoFront<phen_t, Params>,
    sferes::stat::BD<phen_t, Params>,
    sferes::stat::Success<phen_t, Params>    
#ifdef SAVETRAJ
    ,sferes::stat::Traj<phen_t, Params>
#endif
   >  stat_t;

  //MODIFIER
#ifdef DIVERSITY
  typedef boost::fusion::vector<modif::BehaviorDiv<Params> > modifier_t;
#elif defined(NOVELTY)
  typedef boost::fusion::vector<modif::BehaviorNov<Params> > modifier_t;
#else
  typedef boost::fusion::vector<modif::Dummy<Params> > modifier_t;
#endif

  typedef ea::Nsga2<phen_t, eval_t, stat_t, modifier_t, Params> ea_t;

  ea_t ea;
  run_ea(argc, argv, ea);

  return 0;
}
