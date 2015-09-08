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



#ifndef MODIFIER_DIV_HPP
#define MODIFIER_DIV_HPP

namespace sferes
{
  namespace modif
  {
    namespace diversity
    {

      struct dist_t{
	float dist;
	int index;
	bool operator<(const dist_t &disto) const {return dist<disto.dist;};
      };      

      template<typename Phen, typename Params>
      struct _parallel_behavior_div
      {
	typedef typename std::vector<boost::shared_ptr<Phen> > pop_t;
	
	pop_t _pop;
	
	_parallel_behavior_div(pop_t& pop) : _pop(pop) {}
	_parallel_behavior_div(const _parallel_behavior_div& ev) : _pop(ev._pop) {}
	
	void operator() (const parallel::range_t& r) const
	{
	  
	  for (size_t i = r.begin(); i != r.end(); ++i)
	    {
	      float sum=0.0;

	      for (size_t j = 0; j < _pop.size(); ++j) {
		if(i!=j) 
		  sum += ::sqrt(
                        (_pop[i]->fit().end_pos.get_x()-_pop[j]->fit().end_pos.get_x())*(_pop[i]->fit().end_pos.get_x()-_pop[j]->fit().end_pos.get_x())
                        +(_pop[i]->fit().end_pos.get_y()-_pop[j]->fit().end_pos.get_y())*(_pop[i]->fit().end_pos.get_y()-_pop[j]->fit().end_pos.get_y()));
	      }
	      int obj_num = 1;
	      assert(obj_num<_pop[i]->fit().objs().size());
	      _pop[i]->fit().set_obj(obj_num, sum/_pop.size());
	    }
	}
      };
    }

    SFERES_CLASS(BehaviorDiv)
    {
    public:
      
      template<typename Ea>
	void apply(Ea& ea)
      {
	// parallel compute
	parallel::init();
	parallel::p_for(parallel::range_t(0, ea.pop().size()), 
			diversity::_parallel_behavior_div<typename Ea::phen_t, Params>(ea.pop()));
      }
     
    protected:      
      typedef std::vector<struct point_entropy> behavior_t;
      
    };
  }
}

#endif
