#include "Body.h"

namespace Cacti
{
	Body::Body(std::unique_ptr<Shape> shape, Vec3 position)
		:position(position), orientation(Quat(0,0,0,1)), shape(std::move(shape))
	{
	}

}

