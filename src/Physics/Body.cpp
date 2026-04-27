#include "Body.h"
#include "Body.h"
#include "Body.h"
#include <iostream>

namespace Cacti
{
	Body::Body(std::unique_ptr<Shape> shape, Vec3 position, Vec3 linearVel ,Vec3 angularVel, float e, float invMass)
		:position(position), orientation(Quaternion(0,0,0,1)), linearVelocity(linearVel), angularVelocity(angularVel), invMass(invMass), e(e), shape(std::move(shape))
	{
	}

	void Body::Update(const float dt)
	{
		position += linearVelocity * dt;

		Vec3 positionCM = CenterOfMassWorldSpace();
		Vec3 cmToPos = position - positionCM;

		Mat3 orientation = this->orientation.ToMat3();
		Mat3 inertiaTensor = orientation * shape->GetInertiaTensor() * orientation.Transpose();

		Vec3 alpha = inertiaTensor.Inverse() * (angularVelocity.Cross(inertiaTensor * angularVelocity));
		angularVelocity += alpha * dt;

		Vec3 dAngle = angularVelocity * dt;
		Quaternion dq = Quaternion(dAngle, dAngle.GetMagnitude());
		this->orientation = dq * this->orientation;
		this->orientation.Normalize();

		position = positionCM + dq.RotatePoint(cmToPos);
	}

	Vec3 Body::WorldSpaceToLocalSpace(const Vec3 p)
	{
		Vec3 temp = p - CenterOfMassWorldSpace();
		Quaternion inverseOrient = orientation.Inverse();
		Vec3 bodySpace = inverseOrient.RotatePoint(temp);
		return bodySpace;
	}

	Vec3 Body::LocalSpaceToWorldSpace(const Vec3 p)
	{
		Vec3 com = CenterOfMassWorldSpace();
		Vec3 rotatedP = orientation.RotatePoint(p);
		Vec3 worldSpace = com + rotatedP;
		return worldSpace;
	}

	Vec3 Body::CenterOfMassWorldSpace()
	{
		const Vec3 centerOfMass = shape->GetCenterOfMass();
		const Vec3 pos = position + orientation.RotatePoint(centerOfMass);
		return pos;
	}

	void Body::ApplyImpulseAngular(Vec3 impulse)
	{
		if (invMass == 0.0f)
		{
			return;
		}

		// L = I w = r x p
		// dL = I dw = r x J
		// = > dw = I ^ −1 * (r x J)

		angularVelocity += GetInverseInertiaWorldSpace() * impulse;
		const float maxAngularSpeed = 30.0f;
		if (angularVelocity.GetLengthSqr() > maxAngularSpeed * maxAngularSpeed)
		{
			angularVelocity.Normalize();
			angularVelocity *= maxAngularSpeed;
		}

	}
	void Body::ApplyImpulse(Vec3 impulsePoint, Vec3 impulse)
	{
		if (invMass == 0.0f)
		{
			return;
		}

		// impulse Point is the world space location of the application the impulse
		// impulse is the world space direction and magnitude of the impulse
		ApplyImpulseLinear(impulse);
		Vec3 position = CenterOfMassWorldSpace(); // applying impulses must produce to rques the center of mass
		Vec3 r = impulsePoint - position;
		Vec3 dL = r.Cross(impulse); // this is in world space.
		ApplyImpulseAngular(dL);
	}
	void Body::ApplyImpulseLinear(Vec3 impulse)
	{
		if (invMass == 0.0f)
		{
			return;
		}

		// p = mv
		// dp = m dv = J
		// = > dv = J / m
		linearVelocity += impulse * invMass;
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