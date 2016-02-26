#ifndef HPP_RBPRM_NODE_HH
#define HPP_RBPRM_NODE_HH

#include <hpp/core/node.hh>

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
    }; // class

  }//core
}//hpp


#endif // HPP_RBPRM_NODE_HH
