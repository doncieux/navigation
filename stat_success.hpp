#ifndef STAT_SUCCESS_HPP
#define STAT_SUCCESS_HPP
#include <sferes/stat/stat.hpp>
namespace sferes
{
  namespace stat
  {


    /** \brief Stastistic used to save the success of an experiment to generate points reaching the goal position
    */
    SFERES_STAT(Success, Stat)
    {
    public:
      template<typename E>
	void refresh(const E& ea)
      {

	this->_create_log_file(ea, "success.dat");
	
	int nb_success1_this_gen=0;
	int nb_success2_this_gen=0;
	int nb_success3_this_gen=0;
	for (size_t i = 0; i < ea.pop().size(); ++i)
	  {
	    nb_success1_this_gen+=ea.pop()[i]->fit().success1;
	    nb_success2_this_gen+=ea.pop()[i]->fit().success2;
	    nb_success3_this_gen+=ea.pop()[i]->fit().success3;
	  }
	
	(*this->_log_file) << ea.gen() << " " << ea.nb_evals()
			   << " " << ea.pop()[0]->fit().get_success1_so_far()<<" "<<nb_success1_this_gen
			   << " " << ea.pop()[0]->fit().get_success2_so_far()<<" "<<nb_success2_this_gen
			   << " " << ea.pop()[0]->fit().get_success3_so_far()<<" "<<nb_success3_this_gen
			   <<std::endl;
	
      }

    };
  }
}
#endif
