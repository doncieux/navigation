#ifndef STAT_BD_HPP
#define STAT_BD_HPP
#include <sferes/stat/stat.hpp>
namespace sferes
{
  namespace stat
  {


    /** \brief Stastistic used to save the behavior descriptor of the robot into
    * files for later analysis.
    */
    SFERES_STAT(BD, Stat)
    {
    public:
      template<typename E>
	void refresh(const E& ea)
      {

	static int bd_id=0;
	std::ostringstream ofbd_name;
	ofbd_name<<ea.res_dir()<<"/bd_"<<std::setfill('0')<<std::setw(6)<<bd_id<<".log";
	bd_id++;
	    
	std::ofstream outbd(ofbd_name.str());
	if (!outbd.is_open()) {
	  std::cerr<<"Can't open file to save behavior descriptors: "<<ofbd_name.str()<<std::endl;
	  exit(1);
	}

	for (size_t i = 0; i < ea.pop().size(); ++i)
	  {
	    for (int j=0;j<ea.pop()[i]->fit().pos_bd.size();j++) {
	      outbd<<ea.pop()[i]->fit().pos_bd[j].x()<<" ";
	      outbd<<ea.pop()[i]->fit().pos_bd[j].y()<<" ";
	      outbd<<ea.pop()[i]->fit().pos_bd[j].theta()<<" ";
	    }
	    outbd<<std::endl;	    
	  }
	outbd.close();
      }

    };
  }
}
#endif
