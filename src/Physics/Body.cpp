#include "Body.h"
#include "Body.h"

namespace Cacti
{
	Body::Body(std::unique_ptr<Shape> shape, Vec3 position)
		:position(position), orientation(Quat(0,0,0,1)), shape(std::move(shape))
	{
	}

	Vec3 Body::WorldSpaceToLocalSpace(const Vec3 p)
	{
		Vec3 localPointTransformPosition = p - position;

		Quat localPointQuat(localPointTransformPosition.x, localPointTransformPosition.y, localPointTransformPosition.z, 0);

		Quat invertedRotation = orientation * localPointQuat * orientation.Inverse();

		return Vec3(invertedRotation.x, invertedRotation.y, invertedRotation.z);
	}

}

