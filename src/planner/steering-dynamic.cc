# include <hpp/rbprm/planner/steering-dynamic.hh>


namespace hpp{
  namespace rbprm{


    core::PathPtr_t SteeringDynamic::impl_compute (core::ConfigurationIn_t q1,
                                    core::ConfigurationIn_t q2) const
    {
      core::value_type length = (*distance_) (q1, q2);
      core::PathPtr_t path = core::StraightPath::create (problem_->robot(), q1, q2, length,constraints ());
      return path;
    }

  }//rbprm
} // hpp
