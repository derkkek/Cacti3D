#include "Program.h"

#if defined(PLATFORM_DESKTOP)
#define GLSL_VERSION            330
#else   // PLATFORM_ANDROID, PLATFORM_WEB
#define GLSL_VERSION            100
#endif

#include "raymath.h"
#include <random>

Program::Program()
	:running(true), convertedSceneData{}
{
	Init();
	engine.Init();
	renderer.Init();
	InitScene();
}

void Program::Init()
{
	const int screenWidth = 1800;
	const int screenHeight = 800;

	SetConfigFlags(FLAG_MSAA_4X_HINT);

	InitWindow(screenWidth, screenHeight, "raylib [core] example - basic window");

	DisableCursor();

}


void Program::InitScene()
{
	renderer.sceneObjects.reserve(engine.world.bodies.size());

	convertedSceneData.positions.resize(engine.transformBuffer.positions.size());
	convertedSceneData.orientations.resize(engine.transformBuffer.orientations.size());
	convertedSceneData.contacts.resize(20);

	for (int i = 0; i < engine.world.bodies.size(); i++) 
	{
		RenderModel sceneObject = BuildRenderModelFromPhysicsGeometry(engine.world.bodies[i], engine.world.bodies[i].shape.get());
		renderer.AddSceneObject(sceneObject);
	}

}

void Program::Update()
{
	float elapsed = 0.0f;
	const float startDelay = 2.0f;

	while (!WindowShouldClose())
	{
		float dt = GetFrameTime();
		engine.Update(dt);
		
		for (int i = 0; i < engine.world.bodies.size(); i++)
		{
			const Cacti::Vec3 p = engine.transformBuffer.positions[i];
			const Cacti::Quat q = engine.transformBuffer.orientations[i];

			convertedSceneData.positions[i] = { p.x, p.y, p.z };
			convertedSceneData.orientations[i] = { q.x, q.y , q.z, q.w };

		}

		for (int i = 0; i < engine.world.contacts.size(); i++)
		{
			const Cacti::Vec3 lpA = engine.contactBuffer.contacts[i].localPointA;
			const Cacti::Vec3 lpB = engine.contactBuffer.contacts[i].localPointB;

			convertedSceneData.contacts[i].localPointA = {lpA.x, lpA.y, lpA.z};
			convertedSceneData.contacts[i].localPointB = { lpB.x, lpB.y, lpB.z };
		}

		renderer.Update(convertedSceneData);
	}
}

void Program::Destroy()
{
	CloseWindow();
}

RenderModel Program::BuildRenderModelFromPhysicsGeometry(Cacti::Body& body, Cacti::Shape* shape)
{
	static const Color colorList[] = {
	LIGHTGRAY, GRAY, DARKGRAY, YELLOW, GOLD, ORANGE, PINK, RED, MAROON,
	GREEN, LIME, DARKGREEN, SKYBLUE, BLUE, DARKBLUE, PURPLE, VIOLET,
	DARKPURPLE, BEIGE, BROWN, DARKBROWN, WHITE, BLACK, MAGENTA, RAYWHITE
	};
	static const size_t colorCount = sizeof(colorList) / sizeof(colorList[0]);

	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::uniform_int_distribution<size_t> dist(0, colorCount - 1);

	if (shape->GetType() == Cacti::ShapeType::SPHERE)
	{
		Cacti::Sphere* sphereShape = (Cacti::Sphere*)shape;
		Model sphere = LoadModelFromMesh(GenMeshSphere(sphereShape->radius, 50, 50));
		Vector3 raylibPos = { body.position.x, body.position.y, body.position.z };
		Color randomColor = colorList[dist(gen)];
		RenderModel sphereObj{ sphere, randomColor};

		return sphereObj;
	}
	return RenderModel();
}