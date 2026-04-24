#pragma once
#include "Vector.h"

namespace Cacti
{
	class Mat3
	{
	public:
		Mat3() = default;
		~Mat3() = default;

		Mat3 Inverse();
		Mat3 Adjoint();
		float Determinant();
		Mat3 Transpose();

		Vec3 rows[3];

		Mat3 operator / (const float c);
		Mat3 operator * (const float c);
		Mat3 operator * (const Mat3& m);
		Vec3 operator * (const Vec3 v);
	};

	inline Mat3 Mat3::Adjoint()
	{
		Mat3 adj{};
		adj.rows[0][0] = (rows[1][1] * rows[2][2] - rows[2][1] * rows[1][2]);
		adj.rows[0][1] = -(rows[0][1] * rows[2][2] - rows[2][1] * rows[0][2]);
		adj.rows[0][2] = (rows[0][1] * rows[1][2] - rows[1][1] * rows[0][2]);

		// Row 1
		adj.rows[1][0] = -(rows[1][0] * rows[2][2] - rows[2][0] * rows[1][2]);
		adj.rows[1][1] = (rows[0][0] * rows[2][2] - rows[2][0] * rows[0][2]);
		adj.rows[1][2] = -(rows[0][0] * rows[1][2] - rows[1][0] * rows[0][2]);

		// Row 2
		adj.rows[2][0] = (rows[1][0] * rows[2][1] - rows[2][0] * rows[1][1]);
		adj.rows[2][1] = -(rows[0][0] * rows[2][1] - rows[2][0] * rows[0][1]);
		adj.rows[2][2] = (rows[0][0] * rows[1][1] - rows[1][0] * rows[0][1]);

		return adj;
	}

	inline float Mat3::Determinant()
	{
		float a = rows[0][0];
		float b = rows[0][1];
		float c = rows[0][2];

		float d = rows[1][0];
		float e = rows[1][1];
		float f = rows[1][2];

		float g = rows[2][0];
		float h = rows[2][1];
		float i = rows[2][2];

		return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
	}

	inline Mat3 Mat3::operator/ (const float c)
	{
		Mat3 result{};
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				result.rows[i][j] = rows[i][j] / c;
			}
		}
		return result;
	}

	inline Mat3 Mat3::operator*(const float c)
	{
		Mat3 result{};
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				result.rows[i][j] = rows[i][j] * c;
			}
		}
		return result;
	}
	inline Mat3 Mat3::operator * (const Mat3& rhs) {
		Mat3 tmp;
		for (int i = 0; i < 3; i++) {
			tmp.rows[i].x = rows[i].x * rhs.rows[0].x + rows[i].y * rhs.rows[1].x + rows[i].z * rhs.rows[2].x;
			tmp.rows[i].y = rows[i].x * rhs.rows[0].y + rows[i].y * rhs.rows[1].y + rows[i].z * rhs.rows[2].y;
			tmp.rows[i].z = rows[i].x * rhs.rows[0].z + rows[i].y * rhs.rows[1].z + rows[i].z * rhs.rows[2].z;
		}
		return tmp;
	}
	inline Vec3 Mat3::operator *(Vec3 v)
	{
		Vec3 result;
		result[0] = rows[0].Dot(v);
		result[1] = rows[1].Dot(v);
		result[2] = rows[2].Dot(v);
		return result;
	}

	inline Mat3 Mat3::Inverse()
	{
		Mat3 inv{};
		inv = Adjoint() / Determinant();
		return inv;
	}

	inline Mat3 Mat3::Transpose()
	{
		Mat3 t{};
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				t.rows[i][j] = t.rows[j][i];
			}
		}
		return t;
	}

}