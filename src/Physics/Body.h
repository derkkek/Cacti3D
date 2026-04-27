#pragma once
#include "../Math/Vector.h"
#include "../Math/Quaternion.h"
#include "Shapes.h"
#include <memory>
namespace Cacti
{
	class Body
	{
	public:
		Body(std::unique_ptr<Shape> shape, Vec3 position, Vec3 linearVel = Vec3(0,0,0), Vec3 angularVel = Vec3(0, 0, 0), float e = 0.5f,float invMass = 0.0);

		void Update(float dt);
		Vec3 position;

		float invMass;
		Vec3 linearVelocity;

		Vec3 angularVelocity;
		Quaternion orientation;
		std::unique_ptr<Shape> shape;

		Vec3 WorldSpaceToLocalSpace(const Vec3 p);
		Vec3 LocalSpaceToWorldSpace(const Vec3 p);

		Vec3 CenterOfMassWorldSpace();

		float e; // coef. of restitution.

		void ApplyImpulseAngular(Vec3 impulse);

		void ApplyImpulse(const Vec3 impulsePoint, const Vec3 J);

		void ApplyImpulseLinear(Vec3 impulse);

		Mat3 GetInverseInertiaLocalSpace() const;
		Mat3 GetInverseInertiaWorldSpace() const;
	private:
	};

}