#ifndef HPP_RBPRM_NODE_HH
#define HPP_RBPRM_NODE_HH

#include <hpp/core/node.hh>
#include <hpp/rbprm/rbprm-validation-report.hh>
#include <robust-equilibrium-lib/static_equilibrium.hh>


namespace hpp {
  namespace core {

    HPP_PREDEF_CLASS (RbprmNode);
    typedef RbprmNode* RbprmNodePtr_t;

    typedef robust_equilibrium::MatrixXX MatrixXX;
    typedef robust_equilibrium::Matrix6X Matrix6X;
    typedef robust_equilibrium::Matrix63 Matrix63;
    typedef robust_equilibrium::Vector6 Vector6;
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
      

      
      void alpha(double alpha){alpha_=alpha;}
      
      double alpha(){return alpha_;}
      
      void alphaMin(double alphaMin){alphaMin_=alphaMin;}
      
      double alphaMin(){return alphaMin_;}
      
      void alphaMax(double alphaMax){alphaMax_=alphaMax;}
      
      double alphaMax(){return alphaMax_;}
      
      double xTheta(){return Xtheta_;}
      
      void xTheta(double xTheta){Xtheta_ = xTheta;}
      
      double Z(){return Z_;}
      
      void Z(double Z){Z_ = Z;}
      
      double v0(double alpha0){
        return sqrt((9.81*Xtheta_*Xtheta_)/(2*Xtheta_*tan(alpha0) - Z_));
      }
      
      void setAlphas(double alpha, double alphaMin, double alphaMax){
        alpha_=alpha;
        alphaMin_=alphaMin;
        alphaMax_=alphaMax;
      }


      void setV(MatrixXX V){V_ = V;}

      MatrixXX getV(){return V_;}

      void setIPHat(Matrix6X m){IP_hat_=m;}

      Matrix6X getIPhat(){return IP_hat_;}

      void setG(Matrix6X G){G_=G;}

      Matrix6X getG(){return G_;}

      void setH(Matrix63 H){H_=H;}

      Matrix63 getH(){return H_;}

      void seth(Vector6 h){h_=h;}

      Vector6 geth(){return h_;}

      void setNumberOfContacts(int n){numberOfContacts_ = n;}

      int getNumberOfContacts(){return numberOfContacts_;}

    private:
      fcl::Vec3f normal_;
      RbprmValidationReportPtr_t collisionReport_;
    //  const polytope::ProjectedCone* giwc_; // useless now ?
    //  polytope::T_rotation_t rotContact_;
    //  polytope::vector_t posContact_;
      MatrixXX V_;
      Matrix6X IP_hat_;
      Matrix6X G_; // not initialized yet
      Matrix63 H_;
      Vector6 h_;
      int numberOfContacts_;
      // remove this ? :
      double alpha_;    // for parent node ! not for this node
      double alphaMin_; // for parent node ! not for this node
      double alphaMax_; // for parent node ! not for this node
      double Xtheta_;   // for parent node ! not for this node
      double Z_;        // for parent node ! not for this node
      

    }; // class

  }//core
}//hpp


#endif // HPP_RBPRM_NODE_HH
