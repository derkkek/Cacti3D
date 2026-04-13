#pragma once

#include "Math/Vector.h"
#include "Physics/Shapes.h"
#include "Physics/Body.h"
#include "World.h"
#include <TransformBuffer.h>

namespace Cacti
{
	class Engine
	{
	public:
		Engine();
		~Engine();

		void Init();
		void Update(float dt);

		World world;
		TransformBuffer transformBuffer;
		ContactBuffer contactBuffer;
	private:
		void UpdateTransformBuffer();
		void UpdateContactBuffer();
	};

}