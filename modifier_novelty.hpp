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



#ifndef MODIFIER_NOV_HPP
#define MODIFIER_NOV_HPP


namespace sferes
{
  namespace modif
  {
    namespace novelty
    {

      struct dist_t{
	float dist;
	int index;
	bool operator<(const dist_t &disto) const {return dist<disto.dist;};
      };      

      template<typename Phen, typename Archive, typename Params>
      struct _parallel_behavior_nov
      {
	typedef typename std::vector<boost::shared_ptr<Phen> > pop_t;
	
	pop_t _pop;
	const Archive& _apop; 
	
	_parallel_behavior_nov(pop_t& pop, const Archive& apop) : _pop(pop), _apop(apop) {}
	_parallel_behavior_nov(const _parallel_behavior_nov& ev) : _pop(ev._pop), _apop(ev._apop) {}
	
	void operator() (const parallel::range_t& r) const
	{
	  
	  for (size_t i = r.begin(); i != r.end(); ++i)
	    {
	      float sum=0.0;
	      float sum2 = 0.0;
	      std::vector<struct dist_t> v_dist;

	      //Maximum archive size
	      int arsize=Params::novelty::max_archive_size;
	      if(_apop.size()<Params::novelty::max_archive_size)
		arsize=_apop.size();

	      for (size_t j = 0; j < _pop.size() + arsize; ++j) {
		float hd=1.0;
		float hd2=1.0;
		if(i!=j) {
		  if(j<_pop.size()) {
		    
            hd = ::sqrt(
                          (_pop[i]->fit().end_pos.get_x()-_pop[j]->fit().end_pos.get_x())*(_pop[i]->fit().end_pos.get_x()-_pop[j]->fit().end_pos.get_x())
                          +(_pop[i]->fit().end_pos.get_y()-_pop[j]->fit().end_pos.get_y())*(_pop[i]->fit().end_pos.get_y()-_pop[j]->fit().end_pos.get_y()));

		  }
		  else {

            hd = ::sqrt(
                          (_pop[i]->fit().end_pos.get_x()-_apop[j-_pop.size()-arsize+_apop.size()].get_x())*(_pop[i]->fit().end_pos.get_x()-_apop[j-_pop.size()-arsize+_apop.size()].get_x())
                          +(_pop[i]->fit().end_pos.get_y()-_apop[j-_pop.size()-arsize+_apop.size()].get_y())*(_pop[i]->fit().end_pos.get_y()-_apop[j-_pop.size()-arsize+_apop.size()].get_y()));
		  }
		   
		}
		struct dist_t dist_temp;
		dist_temp.index=j;
		dist_temp.dist=hd;
		v_dist.push_back(dist_temp);
		
	      }
	      //Sort the distance vector
	      sort(v_dist.begin(),v_dist.end());
	      
	      //Get the sparsness with the k closest vectors	      
	      float d = 0.0f;
	      for(size_t j=0; j<Params::novelty::k; ++j) 
            d+=v_dist[j].dist;
	      d /= (float)Params::novelty::k;
	      int obj_num = 1;
	      assert(obj_num<_pop[i]->fit().objs().size());
	      _pop[i]->fit().set_obj(obj_num, d);
	    }
	}
      };
    }

    SFERES_CLASS(BehaviorNov)
    {
    public:
      
      template<typename Ea>
	void apply(Ea& ea)
      {
        //std::cout<<"insidenov"<<std::endl;
        // parallel compute
        parallel::init();
        parallel::p_for(parallel::range_t(0, ea.pop().size()), 
                        novelty::_parallel_behavior_nov<typename Ea::phen_t, archive_t, Params>(ea.pop(),_archive));
        //std::cout<<"paralleldone"<<std::endl;
        //Update archive
        int obj_num = 1;
        int bestindiv = -1;
        float max_sparse = -1.0;
        
        for(size_t i = 0; i < ea.pop().size(); ++i) {
          float sparse = ea.pop()[i]->fit().obj(obj_num);	  
          if(max_sparse < sparse) {
            max_sparse =  sparse;
            bestindiv = i;
          }
        }
        if(bestindiv!=-1) 
          _archive.push_back(ea.pop()[bestindiv]->fit().end_pos);
        
      }
      int _size() const {return _archive.size();}
      float _prop_archive() const {return (float)_n_archive;}
	
    protected:      
      typedef fastsim::Posture behavior_t;
      typedef std::vector<behavior_t> archive_t;   

      archive_t _archive;
      int _n_archive;
    };
  }
}

#endif
