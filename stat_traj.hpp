#ifndef STAT_TRAJ_HPP
#define STAT_TRAJ_HPP
#include <sferes/stat/stat.hpp>
namespace sferes
{
  namespace stat
  {


    /** \brief Stastistic used to save the trajectory of the robot into
    * files for later analysis.

    * The SAVETRAJ label must have been defined. A fitness attribute (string) called "traj" contains the full trajectory. The stat does no more than saving it into a file.
    */
    SFERES_STAT(Traj, Stat)
    {
    public:
      template<typename E>
	void refresh(const E& ea)
      {

  static int traj_id=0;
	for (size_t i = 0; i < ea.pop().size(); ++i)
	  {
      std::ostringstream oftraj_name;
      oftraj_name<<ea.res_dir()<<"/traj_"<<std::setfill('0')<<std::setw(6)<<traj_id<<".traj";
      traj_id++;
      //std::cout<<"Saving traj to: "<<oftraj_name.str()<<std::endl;

       std::ofstream outtraj(oftraj_name.str());
       if (!outtraj.is_open()) {
         std::cerr<<"Can't open file to save trajectory: "<<oftraj_name.str()<<std::endl;
         exit(1);
       }
       outtraj<<"# fitness: ";
       for (int j=0;j<ea.pop()[i]->fit().objs().size();j++)
	 outtraj<<ea.pop()[i]->fit().obj(j)<<" ";
       outtraj<<std::endl;
       outtraj<<ea.pop()[i]->fit().traj;
       outtraj.close();

	  }
      }
    };
  }
}
#endif
