#include "Body.h"

namespace Cacti
{
	Body::Body(Shape* shape, Vec3 position)
		:position(position), shape(shape)
	{
	}
	Cacti::Body::~Body()
	{
	}
}

