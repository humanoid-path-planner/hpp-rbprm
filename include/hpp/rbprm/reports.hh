/// Copyright (c) 2017 CNRS
/// Authors: stonneau
///
///
// This file is part of hpp-rbprm.
// hpp-rbprm is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-wholebody-step-planner is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-wholebody-step-planner. If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_RBPRM_CONTACT_REPORT_HH
#define HPP_RBPRM_CONTACT_REPORT_HH

#include <hpp/rbprm/config.hh>
#include <hpp/rbprm/rbprm-state.hh>

namespace hpp {
namespace rbprm {

enum HPP_RBPRM_DLLAPI ContactComputationStatus {
  NO_CONTACT = 0,
  UNSTABLE_CONTACT = 1,
  STABLE_CONTACT = 2,
  REACHABLE_CONTACT =
      3  // in current implementation REACHABLE is always STABLE ... we might need to use mask for futur developpement
};

namespace projection {
struct HPP_RBPRM_DLLAPI ProjectionReport {
  ProjectionReport() : success_(false), status_(NO_CONTACT) {}
  ProjectionReport(const ProjectionReport&);
  ~ProjectionReport() {}
  bool success_;
  hpp::rbprm::State result_;
  ContactComputationStatus status_;
};

}  // namespace projection

namespace contact {

struct HPP_RBPRM_DLLAPI ContactReport : public projection::ProjectionReport {
  ContactReport();
  ContactReport(const projection::ProjectionReport&);
  bool contactMaintained_;
  bool multipleBreaks_;
  bool contactCreated_;
  bool repositionedInPlace_;
};
}  // namespace contact
}  // namespace rbprm
}  // namespace hpp
#endif  // HPP_RBPRM_ALGORITHM_HH
