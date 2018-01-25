#include <hpp/rbprm/contact_generation/reachability.hh>


namespace hpp {
  namespace rbprm {
   namespace reachability{

std::pair<MatrixXX, MatrixXX> stackConstraints(const std::pair<MatrixXX, MatrixXX>& Ab,const std::pair<MatrixXX, MatrixXX>& Cd){
    size_t numIneq = Ab.first.rows() + Cd.first.rows();
    MatrixXX M(numIneq,3);
    VectorX  N(numIneq);
    M.block(0,0,Ab.first.rows(),3) = Ab.first;
    M.block(Ab.first.rows(),0,Cd.first.rows(),3) = Cd.first;
    N.segment(0,Ab.first.rows()) = Ab.second;
    N.segment(Ab.first.rows(),Cd.first.rows()) = Cd.second;
    return std::make_pair(M,n);
}



}
}
}
