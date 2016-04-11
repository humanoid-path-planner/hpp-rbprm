#ifndef HPP_RBPRM_NODE_HH
#define HPP_RBPRM_NODE_HH

#include <hpp/core/node.hh>
#include <hpp/rbprm/rbprm-validation-report.hh>
#include <polytope/stability_margin.h>

namespace hpp {
  namespace core {

    HPP_PREDEF_CLASS (RbprmNode);
    typedef RbprmNode* RbprmNodePtr_t;


    class HPP_CORE_DLLAPI RbprmNode : public Node
    {
    public :
      /// Constructor
      /// \param configuration configuration stored in the new node
      /// \note A new connected component is created. For consistency, the
      ///       new node is not registered in the connected component.
      RbprmNode (const ConfigurationPtr_t& configuration):
        Node(configuration)
      {}
      /// Constructor
      /// \param configuration configuration stored in the new node
      /// \param connectedComponent connected component the node belongs to.
      RbprmNode (const ConfigurationPtr_t& configuration,
      ConnectedComponentPtr_t connectedComponent):
        Node(configuration,connectedComponent)
      {}

      fcl::Vec3f getNormal(){
        return normal_;
      }

      void normal(double x, double y, double z){
        fcl::Vec3f n(x,y,z);
        normal_=n;
      }

      void normal(fcl::Vec3f n){
        normal_ = n;
      }

      RbprmValidationReportPtr_t getReport(){
        return collisionReport_;
      }

      void collisionReport(RbprmValidationReportPtr_t report){
        collisionReport_ = report;
      }
      
      void giwc(polytope::ProjectedCone* giwc){giwc_ = giwc;}
      
      polytope::ProjectedCone* giwc(){return giwc_;}

    private:
      fcl::Vec3f normal_;
      RbprmValidationReportPtr_t collisionReport_;
      polytope::ProjectedCone* giwc_;

    }; // class

  }//core
}//hpp


#endif // HPP_RBPRM_NODE_HH
