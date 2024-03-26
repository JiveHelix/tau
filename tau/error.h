#pragma once


#include <jive/create_exception.h>


namespace tau
{


CREATE_EXCEPTION(TauError, std::runtime_error);
CREATE_EXCEPTION(NoIntersection, TauError);


} // end namespace tau
