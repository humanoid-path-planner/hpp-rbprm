#ifndef HPP_RBPRM_ROADMAP_HH
#define HPP_RBPRM_ROADMAP_HH

#include <hpp/core/roadmap.hh>
#include <hpp/rbprm/planner/rbprm-node.hh>

namespace hpp {
  namespace core {

    HPP_PREDEF_CLASS (RbprmRoadmap);
    typedef boost::shared_ptr <RbprmRoadmap> RbprmRoadmapPtr_t;

    class HPP_CORE_DLLAPI RbprmRoadmap : public Roadmap
    {
    public:
      /// Return shared pointer to new instance.
      static RbprmRoadmapPtr_t create (const DistancePtr_t& distance, const DevicePtr_t& robot){
        RbprmRoadmap* ptr = new RbprmRoadmap (distance, robot);
        return RbprmRoadmapPtr_t (ptr);
      }

      virtual ~RbprmRoadmap ()
      {
        clear ();
      }

    protected:
      /// Constructor
      /// \param distance distance function for nearest neighbor computations
      RbprmRoadmap (const DistancePtr_t& distance, const DevicePtr_t& robot):
        Roadmap(distance,robot)
      {
      }



      /// Node factory
      /// Reimplement the function if you want to create an instance of a
      /// child class of Node
      virtual RbprmNodePtr_t createNode (const ConfigurationPtr_t& configuration) const  {
        return RbprmNodePtr_t (new RbprmNode (configuration));
      }

    }; // class

  }//core
}//hpp
#endif // HPP_RBPRM_ROADMAP_HH
