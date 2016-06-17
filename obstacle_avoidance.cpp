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


using namespace sferes;
using namespace sferes::gen::evo_float;
using namespace sferes::gen::dnn;
using namespace fastsim;
using namespace nn;

struct Params
{
  struct dnn
  {
    static constexpr size_t nb_inputs       = 3; // laser
    static constexpr size_t nb_outputs      = 2; // 2 motors: left and right wheel
    static constexpr size_t min_nb_neurons  = 0;
    static constexpr size_t max_nb_neurons  = 30;
    static constexpr size_t min_nb_conns    = 8;
    static constexpr size_t max_nb_conns    = 250;

    static constexpr int io_param_evolving = true;
    static constexpr float m_rate_add_conn	= 0.0f;
    static constexpr float m_rate_del_conn	= 0.0f;
    static constexpr float m_rate_change_conn = 0.05f;
    static constexpr float m_rate_add_neuron  = 0.0f;
    static constexpr float m_rate_del_neuron  = 0.0f;

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
    static constexpr unsigned size = 100;
    static constexpr unsigned nb_gen = 1001;
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
    static constexpr float nb_steps = 1000;

#ifdef ENVOA1
    SFERES_STRING(map_name, SFERES_ROOT "/exp/navigation/arena1.pbm");
#elif defined(ENVOA2)
    SFERES_STRING(map_name, SFERES_ROOT "/exp/navigation/arena2.pbm");
#elif defined(ENVOA3)
    SFERES_STRING(map_name, SFERES_ROOT "/exp/navigation/arena3.pbm");
#else
    SFERES_STRING(map_name, SFERES_ROOT "/exp/navigation/arena1.pbm");
#endif
  };

};

std::string res_dir="not_initialized";

namespace sferes
{

  // ********** Main Class ***********
  SFERES_FITNESS(FitObstacle, sferes::fit::Fitness)
  {
  public:
    FitObstacle():nb_coll(0), time(0), speed(0), lin_speed(0), stop_eval(false) { }

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

      time=0;

      // *** Main Loop ***
      for (size_t i = 0; i < Params::simu::nb_steps && !stop_eval;)
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
        os<<res_dir<<"/img_"<<std::setfill('0')<<std::setw(6)<<time<<".bmp";
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

      // loop forever if we are in the visualization mode
      if (this->mode() != fit::mode::view)
        i++;


	}


#if defined(FIT1)
      // Compute the fitness value
      this->_objs[0] = 1.0/(float)(1+nb_coll);
      this->_value = 1.0/(float)(1+nb_coll);
#elif defined(FIT2)
      // Compute the fitness value
      this->_objs[0] = (speed/(float)Params::simu::nb_steps)*1.0/(float)(1+nb_coll);
      this->_value = (speed/(float)Params::simu::nb_steps)*1.0/(float)(1+nb_coll);
#elif defined(FIT3)
      // Compute the fitness value
      this->_objs[0] = (lin_speed/(float)Params::simu::nb_steps)*1.0/(float)(1+nb_coll);
      this->_value = (lin_speed/(float)Params::simu::nb_steps)*1.0/(float)(1+nb_coll);
#endif


#ifdef VERBOSE
      static int nbeval=0;
      std::cout<<"fit="<<this->_objs[0]<<" nbeval="<<nbeval<<std::endl;
      nbeval++;
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

      simu.robot().set_pos(Posture(simu.map()->get_real_w()/4.0,simu.map()->get_real_w()/4.0, M_PI/4.0));
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

      float s=(outf[0]+outf[1])/8.0; // in [-1;1]
      float ds=fabs(outf[0]-outf[1])/8.0; // in [0;1]
      speed+=s;
      lin_speed+=s*(1.0-ds);

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
    fastsim::Posture old_pos;
    bool stop_eval;                                  // Stops the evaluation
    std::vector<float> outf, inputs;



  };

}

// ****************** Main *************************
int main(int argc, char **argv)
{
  srand(time(0));

  typedef FitObstacle<Params> fit_t;

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
  typedef boost::fusion::vector<sferes::stat::ParetoFront<phen_t, Params> >  stat_t;

  //MODIFIER
  typedef boost::fusion::vector<modif::Dummy<Params> > modifier_t;

  typedef ea::Nsga2<phen_t, eval_t, stat_t, modifier_t, Params> ea_t;

  ea_t ea;
  res_dir=ea.res_dir();
  run_ea(argc, argv, ea);

  return 0;
}
