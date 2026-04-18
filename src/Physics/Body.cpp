#include "Body.h"
#include "Body.h"

namespace Cacti
{
	Body::Body(std::unique_ptr<Shape> shape, Vec3 position, Vec3 angularVel)
		:position(position), orientation(Quaternion(0,0,0,1)), angularVelocity(angularVel), shape(std::move(shape))
	{
	}

	void Body::Update(float dt)
	{
		Vec3 rotateAxis = angularVelocity.Normalize() * dt;
		float angle = angularVelocity.GetMagnitude();
		Quaternion newOrientation = Quaternion(rotateAxis, angle);
		orientation = newOrientation * orientation;
		orientation.Normalize();
	}

	Vec3 Body::WorldSpaceToLocalSpace(const Vec3 p)
	{
		Vec3 localPointTransformPosition = p - position;

		Quaternion localPointQuat(localPointTransformPosition.x, localPointTransformPosition.y, localPointTransformPosition.z, 0);

		Quaternion invertedRotation = orientation * localPointQuat * orientation.Inverse();

		return Vec3(invertedRotation.x, invertedRotation.y, invertedRotation.z);
	}

}

