#pragma once
#include "raylib.h"
#include <vector>
#include "Renderer.h"
#include <Engine.h>

class Program
{
public:
	Program();
	~Program() = default;

	void Update();
	void Destroy();
	void InitScene();
private:
	void Init();
	RenderModel BuildRenderModelFromPhysicsGeometry(Cacti::Body& body, Cacti::Shape* shape);

	Renderer renderer;
	Cacti::Engine engine;
	bool running;

	ConvertedSceneData convertedSceneData;
};
