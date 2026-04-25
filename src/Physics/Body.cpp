#include "Body.h"
#include <iostream>

namespace Cacti
{
	Body::Body(std::unique_ptr<Shape> shape, Vec3 position, Vec3 linearVel ,Vec3 angularVel, float e, float invMass)
		:position(position), orientation(Quaternion(0,0,0,1)), linearVelocity(linearVel), angularVelocity(angularVel), invMass(invMass), e(e), shape(std::move(shape))
	{
	}

	void Body::Update(float dt)
	{
		position += linearVelocity * dt;

		float angle = angularVelocity.GetMagnitude() * dt;

		Vec3 axis = angularVelocity.Normalize(); // this doesn't normalize velocity compeletly. Returns a copy, so we don't lose actual angular velocity.
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

	Vec3 Body::CenterOfMassWorldSpace()
	{
		Vec3 shapeCom = shape->GetCenterOfMass();
		
		Mat3 R = orientation.ToMat3();

		Vec3 rotation = R * shapeCom;

		Vec3 comWorld = position + rotation;
		return comWorld;
	}

	void Body::ApplyImpulse(const Vec3 impulsePoint, const Vec3 J)
	{
		if (invMass == 0.0)
		{
			return;
		}

		linearVelocity += J * invMass;
		const Vec3 r = impulsePoint - CenterOfMassWorldSpace();
		const Vec3 dL = r.Cross(J);
		Mat3 invInertia = GetInverseInertiaWorldSpace();

		Vec3 dAngular = invInertia * dL;
		const Vec3 com = CenterOfMassWorldSpace();
		angularVelocity += invInertia * dL;
		std::cout << "angular vel:" << angularVelocity.x << ", " << angularVelocity.y << ", " << angularVelocity.z << "\n";

	}
	Mat3 Body::GetInverseInertiaLocalSpace() const
	{
		Mat3 inertiaTensor = shape->GetInertiaTensor();
		Mat3 invInertiaTensor = inertiaTensor.Inverse() * invMass;
		return invInertiaTensor;
	}
	Mat3 Body::GetInverseInertiaWorldSpace() const
	{
		Mat3 inertiaTensor = shape->GetInertiaTensor();
		Mat3 invInertiaTensor = inertiaTensor.Inverse() * invMass;
		Mat3 orientationMatrix = orientation.ToMat3();
		Mat3 invInertiaWorldSpace = orientationMatrix * invInertiaTensor * orientationMatrix.Transpose();
		return invInertiaWorldSpace;
	}
}