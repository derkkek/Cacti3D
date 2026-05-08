#include "Engine.h"
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
		contactBuffer.Init(world.MaxBodies * world.MaxBodies);
		aabbBuffer.Init(world.MaxBodies);
	}

	void Cacti::Engine::UpdateTransformBuffer()
	{
		for (int i = 0; i < world.bodies.size(); i++)
		{
			transformBuffer.positions[i] = world.bodies[i].position;
			transformBuffer.orientations[i] = world.bodies[i].orientation;
		}
	}

	void Engine::UpdateContactBuffer()
	{
		contactBuffer.contacts.resize(world.numContacts);
		for (int i = 0; i < world.numContacts; i++)
		{
			contactBuffer.contacts[i] = world.contacts[i];
		}

		/*This can be removed in future. now Debug purposes.*/
		//if (world.contacts.size() >= contactBuffer.contacts.size())
		//{
		//	contactBuffer.contacts.resize(contactBuffer.contacts.size() * 2);
		//}
	}

	void Engine::UpdateAABBBuffer()
	{
		for (int i = 0; i < world.bodies.size(); i++)
		{
			aabbBuffer.aabbs[i] = world.bodies[i].shape->GetAABBWorldSpace(world.bodies[i].position, world.bodies[i].orientation);
		}
	}

	void Engine::Update(float dt)
	{
		world.Update(dt);
		UpdateTransformBuffer();
		UpdateContactBuffer();
		UpdateAABBBuffer();
	}
}