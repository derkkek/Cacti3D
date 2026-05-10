#pragma once
#include "Math/Matrix.h"
#include <Math/AABB.h>
#include <Math/Quaternion.h>
#include <vector>

namespace Cacti
{
	enum ShapeType
	{
		SPHERE,
		BOX
	};

	class Shape
	{
	public:
		Shape(Vec3 com)
			:centerOfMass(com)
		{

		}
		~Shape() = default;

		const virtual ShapeType GetType() = 0;

		virtual Vec3 GetCenterOfMass() const
		{
			return centerOfMass;
		}

		virtual Mat3 GetInertiaTensor() const = 0;

		virtual AABB GetAABBWorldSpace(const Vec3& pos, const Quaternion& orient) = 0;

		virtual void Build()
		{

		}

	protected:
		Vec3 centerOfMass;
	};

	class Sphere : public Shape
	{
	public:
		Sphere(float radius)
			:radius(radius), Shape(Vec3(0, 0, 0))
		{

		}
		~Sphere() = default;

		const ShapeType GetType()
		{
			return SPHERE;
		}

		Vec3 GetCenterOfMass() const override
		{
			return Vec3(0, 0, 0);
		}

		Mat3 GetInertiaTensor() const override
		{
			Mat3 tensor{};
			tensor.Zero();
			tensor.rows[0][0] = 2.0f / 5.0f * radius * radius;
			tensor.rows[1][1] = 2.0f / 5.0f * radius * radius;
			tensor.rows[2][2] = 2.0f / 5.0f * radius * radius;
			return tensor;
		}

		AABB GetAABBWorldSpace(const Vec3& pos, const Quaternion& orient) override
		{
			AABB aabb{};
			aabb.min = Vec3(-radius) + pos;
			aabb.max = Vec3(radius) + pos;
			return aabb;
		}

		float radius;
	private:

	};

	class Box : public Shape
	{
	public:
		Box(float width, float height, float depth)
			:width(width), height(height), depth(depth), Shape(Vec3(width / 2.0f, height / 2.0f, depth / 2.0f))
		{
			Build();
		}
		~Box() = default;

		void Build() override
		{
			float halfWidth = width / 2.0f;
			float halfHeight = height / 2.0f;
			float halfDepth = depth / 2.0f;

			float x = centerOfMass.x - halfWidth;
			float y = centerOfMass.y - halfHeight;
			float z = centerOfMass.z - halfDepth;
			
			vertices.emplace_back(x, y, z); //near bottom left corner

			x = centerOfMass.x + halfWidth;
			vertices.emplace_back(x, y, z); //near bottom right 

			y = centerOfMass.y + halfHeight;
			vertices.emplace_back(x, y, z); //near top right;

			x = centerOfMass.x - halfWidth;
			vertices.emplace_back(x, y, z); //near top left;

			y = centerOfMass.y - halfHeight;
			z = centerOfMass.z + halfDepth;
			vertices.emplace_back(x, y, z); //far bottom left;

			x = centerOfMass.x + halfWidth;
			vertices.emplace_back(x, y, z); //far bottom right;

			y = centerOfMass.y + halfHeight;
			vertices.emplace_back(x, y, z); //far top right;

			x = centerOfMass.x - halfWidth;
			vertices.emplace_back(x, y, z);  //far top left;
		}
		const ShapeType GetType() override
		{
			return BOX;
		}

		Vec3 GetCenterOfMass() const override
		{
			return centerOfMass;
		}

		Mat3 GetInertiaTensor() const override
		{
			Mat3 t;
			t.Zero();
			t.rows[0][0] = 1.0f / 12.0f * (height * height + depth * depth);
			t.rows[1][1] = 1.0f / 12.0f * (width * width + depth * depth);
			t.rows[2][2] = 1.0f / 12.0f * (width * width + height * height);
			return t;
		}

		AABB GetAABBWorldSpace(const Vec3& pos, const Quaternion& orient)
		{
			AABB aabb;
			//aabb.min = pos - unit;
			//aabb.max = pos + unit;

			//orient.RotatePoint(aabb.min);
			//orient.RotatePoint(aabb.max);
			return aabb;
		}

		std::vector<Vec3> vertices;

		float width;
		float depth;
		float height;
	private:

	};

}