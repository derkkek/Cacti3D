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

		virtual float FastestLinearSpeed(const Vec3 linearVel, const Vec3 angularVel, const Vec3 dir)
		{
			return 0.0f;
		}

		virtual Vec3 Support(Vec3 dir, const Vec3 pos, const Quaternion quat)
		{
			return Vec3(0, 0, 0);
		}

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
			:width(width), height(height), depth(depth), Shape(Vec3(0,0,0))
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

			//for (int i = 0; i < 8; i++)
			//{
			//	Vec3 worldCoordVertex = orient.RotatePoint(vertices[i]) + pos;
			//	aabb.ExpandToContainPoint(worldCoordVertex);
			//}
			//return aabb;

			/* Below is optimized version for "boxes" */
			Mat3 R = orient.ToMat3();

			Vec3 e = Vec3(width * 0.5f, height * 0.5f, depth * 0.5f);

			float halfExtentsX = R.rows[0][0] * e.x + R.rows[0][1] * e.y + R.rows[0][2] * e.z;
			float halfExtentsY = R.rows[1][0] * e.x + R.rows[1][1] * e.y + R.rows[1][2] * e.z;
			float halfExtentsZ = R.rows[2][0] * e.x + R.rows[2][1] * e.y + R.rows[2][2] * e.z;

			Vec3 halfExtents(halfExtentsX, halfExtentsY, halfExtentsZ);

			aabb.min = pos - halfExtents;
			aabb.max = pos + halfExtents;

			return aabb;
		}

		/*Not the most performant Support function, consider box optimization or transform dir to local calculate in local then transform the result to world in future.*/
		Vec3 Support(Vec3 dir, const Vec3 pos, const Quaternion quat) override
		{
			float max = -INFINITY;
			int index = 0;
			Vec3 support(max, max, max);

			float bias = 0.025f;

			for (int i = 0; i < 8; i++)
			{
				Vec3 vertexWorldCoord = quat.RotatePoint(vertices[i]) + pos;
				//Imagine, carry the vertex position vector's starting point to the direction vector's and project it to the direction
				float dot = vertexWorldCoord.Dot(dir);
				
				//if projection is bigger than the current one, then this point is further from the direction.
				if (dot > max)
				{
					max = dot;
					index = i;
					support = vertexWorldCoord;
				}
			}
			return support + (dir.Normalize() * bias);
		}
		float FastestLinearSpeed(const Vec3 angularVel, const Vec3 dir)
		{
			float maxSpeed = 0.0f;
			for (int i = 0; i < 8; i++)
			{
				Vec3 r = vertices[i] - centerOfMass;
				Vec3 pointVelocity = angularVel.Cross(r);
				float speed = pointVelocity.Dot(dir);

				if (speed > maxSpeed)
				{
					maxSpeed = speed;
				}
			}
			return maxSpeed;
		}

		std::vector<Vec3> vertices;

		float width;
		float depth;
		float height;
	private:

	};

}