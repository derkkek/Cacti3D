#include "Body.h"
#include "Body.h"
#include "Body.h"

namespace Cacti
{
	Body::Body(std::unique_ptr<Shape> shape, Vec3 position, Vec3 linearVel ,Vec3 angularVel, float invMass)
		:position(position), orientation(Quaternion(0,0,0,1)), linearVelocity(linearVel), angularVelocity(angularVel), invMass(invMass), shape(std::move(shape))
	{
	}

	void Body::Update(float dt)
	{
		position += linearVelocity * dt;

		float angle = angularVelocity.GetMagnitude() * dt;

		Vec3 axis = angularVelocity.Normalized(); // this doesn't normalize velocity compeletly. Returns a copy, so we don't lose actual angular velocity.
		Quaternion deltaRotation(axis, angle);
		orientation = deltaRotation * orientation; // this order works for world space rotations.
		orientation.Normalize();
	}

	Vec3 Body::WorldSpaceToLocalSpace(const Vec3 p)
	{
		Vec3 localPointTransformPosition = p - shape->GetCenterOfMass();

		Quaternion localPointQuat(localPointTransformPosition.x, localPointTransformPosition.y, localPointTransformPosition.z, 0);

		Quaternion invertedRotation = orientation.Inverse() * localPointQuat * orientation;

		return Vec3(invertedRotation.x, invertedRotation.y, invertedRotation.z);
	}

	void Body::ApplyImpulse(const Vec3 J)
	{
		if (invMass == 0.0)
		{
			return;
		}

		linearVelocity += J * invMass;
	}

}

