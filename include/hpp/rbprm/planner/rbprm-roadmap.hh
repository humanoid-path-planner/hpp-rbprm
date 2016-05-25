#ifndef HPP_RBPRM_ROADMAP_HH
#define HPP_RBPRM_ROADMAP_HH

#include <hpp/core/roadmap.hh>
#include <hpp/rbprm/planner/rbprm-node.hh>
#include <hpp/util/debug.hh>
#include <hpp/model/configuration.hh>

namespace hpp {
  namespace core {
    using model::displayConfig;
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
      
      virtual RbprmNodePtr_t addNode (const ConfigurationPtr_t& configuration)
      {
        value_type distance;
        if (nodes().size () != 0) {
          NodePtr_t nearest = nearestNode (configuration, distance);
          if (*(nearest->configuration ()) == *configuration) {
            return static_cast<core::RbprmNodePtr_t>(nearest);
          }
        }
        RbprmNodePtr_t node = createNode (configuration);
        hppDout (info, "Added node: " << displayConfig (*configuration));
        push_node (node);
        // Node constructor creates a new connected component. This new
        // connected component needs to be added in the roadmap and the
        // new node needs to be registered in the connected component.
        addConnectedComponent (node);
        return node;
      }
      
      
      
      virtual RbprmNodePtr_t addNodeAndEdges (const NodePtr_t from,
            const ConfigurationPtr_t& to,
            const PathPtr_t path)
      {
        NodePtr_t nodeTo = Roadmap::addNodeAndEdges(from,to,path);
        return static_cast<core::RbprmNodePtr_t>(nodeTo);
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
