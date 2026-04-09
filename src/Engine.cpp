#include "Engine.h"

namespace Cacti
{
	Engine::Engine()
		:world()
	{

	}
	Engine::~Engine()
	{
	}
	void Cacti::Engine::Init()
	{
		world.Init();
		transformBuffer.Init(world.MaxBodies);
	}

	void Cacti::Engine::UpdateTransformBuffer()
	{
		for (int i = 0; i < world.bodies.size(); i++)
		{
			transformBuffer.positions[i] = world.bodies[i].position;
			transformBuffer.orientations[i] = world.bodies[i].orientation;
		}
	}

	void Engine::Update(float dt)
	{
		world.Update(dt);
		UpdateTransformBuffer();
	}
}