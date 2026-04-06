#include "Body.h"

namespace Cacti
{
	Body::Body(Shape* shape, Vec3 position)
		:position(position), orientation(Quat(0,0,0,1)), shape(shape)
	{
	}
	Cacti::Body::~Body()
	{
	}
}

