#include "tau/pose.h"


namespace tau
{


template struct Pose<float>;
template struct Pose<double>;


} // end namespace tau


template struct pex::Group
    <
        tau::PoseFields,
        tau::PoseTemplate<float>::template Template,
        pex::PlainT<tau::Pose<float>>
    >;


template struct pex::Group
    <
        tau::PoseFields,
        tau::PoseTemplate<double>::template Template,
        pex::PlainT<tau::Pose<double>>
    >;
