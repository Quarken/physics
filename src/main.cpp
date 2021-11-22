
static app_state g_AppState;

arena *TemporaryArena()
{
    return &g_AppState.FrameArena;
}

arena *PersistentArena()
{
    return &g_AppState.PersistentArena;
}

world *GetWorld()
{
    return &g_AppState.World;
}

rigid_body *GetEntityByHandle(entity_handle Handle)
{
    ASSERT(Handle != 0);
    return GetWorld()->Entities + Handle;
}

entity_handle CreateEntity(rigid_body **Out)
{
    world *World = GetWorld();
    entity_handle EntityID = World->EntityCount++;
    rigid_body *Entity = World->Entities + EntityID;
    *Entity = {};
    if (Out)
    {
        *Out = Entity;
    }
    return EntityID;
}


void InitApp()
{
    app_state *State = &g_AppState;
    world *World = GetWorld();

    State->FrameArena = CreateArena();
    State->PersistentArena = CreateArena();

    World->HullArena = CreateArena();
    World->SolverIterations = 5;
    World->Camera.FocusPosition = V3(0,0,0);
    World->Camera.LatAngle = 0;
    World->Camera.LngAngle = 0;
    World->Camera.Radius = 300.f;
}

void DEBUGCreateBoxRigidBody(arena *Arena, rigid_body *Body, float Width, float Depth, float Height, float Mass)
{
    memset(Body, 0, sizeof(rigid_body));
    Body->Type = RigidBodyType_Dynamic;

    v3 Max = V3(Width/2.f, Depth/2.f, Height/2.f);
    v3 Min = -Max;

    Body->Hull = DEBUGCreateBoxHull(Arena, Min, Max);
    Body->DEBUGModel.Min = Min;
    Body->DEBUGModel.Max = Max;
    Body->BoundingVolume.Min = Min;
    Body->BoundingVolume.Max = Max;
    Body->Orientation = Rotation(V3(1,0,0), 0);

    Body->Mass = Mass;

    if (Mass == 0)
    {
        Body->InverseMass = 0;
    }
    else
    {
        Body->InverseMass = 1.f / Mass;
    }

    if (Mass != 0)
    {
        float Ix = (1.f / 12.f) * Mass * (Square(Height) + Square(Depth));
        float Iy = (1.f / 12.f) * Mass * (Square(Width)  + Square(Depth));
        float Iz = (1.f / 12.f) * Mass * (Square(Width)  + Square(Height));

        Body->Inertia[0][0] = Ix;
        Body->Inertia[1][1] = Iy;
        Body->Inertia[2][2] = Iz;

        Body->InverseInertia[0][0] = 1.f / Ix;
        Body->InverseInertia[1][1] = 1.f / Iy;
        Body->InverseInertia[2][2] = 1.f / Iz;
    }
}

entity_handle DEBUGCreateRigidBody(float Width, float Depth, float Height, float Mass,
                                   v3 Position = V3(0,0,0),
                                   quaternion Orientation = Rotation(V3(1,0,0),0))
{
    world *World = GetWorld();

    rigid_body *Entity;
    i32 EntityID = CreateEntity(&Entity);
    DEBUGCreateBoxRigidBody(&World->HullArena, Entity, Width, Depth, Height, Mass);
    Entity->Position = Position;
    Entity->Orientation = Orientation;
    Entity->Recalculate();
    Entity->RecalculateModelMatrix();
    return EntityID;
}

void Simulate(float dt)
{
    world *World = GetWorld();
    World->DEBUG_SATCalls = 0;
    World->DEBUG_DetectedCollisions = 0;
    World->DEBUG_ReusedContacts = 0;
    World->DEBUG_NewContacts = 0;

    // 1 unit = 1cm
    float UnitsPerMeter = 100.f;
    v3 Gravity = V3(0, 0, -9.8f * UnitsPerMeter);

    // Apply forces.
    for (i32 i = 1; i < GetWorld()->EntityCount; ++i)
    {
        rigid_body *Entity = GetEntityByHandle(i);
        if (Entity->Type == RigidBodyType_Dynamic)
        {
            Entity->LinearMomentum += Gravity * Entity->Mass * dt;
        }
        Entity->Recalculate();
        Entity->RecalculateModelMatrix();
    }

    Broadphase(World, PersistentArena());

    for (i32 Iteration = 0;
         Iteration < World->SolverIterations;
         ++Iteration)
    {
        for (i32 i = 0; i < World->CollisionPairCount; ++i)
        {
            i32 EntityA = World->CollisionPairs[i].EntityA;
            i32 EntityB = World->CollisionPairs[i].EntityB;
            arbiter *Arbiter = GetArbiter(World, EntityA, EntityB);

            if (!Arbiter || Arbiter->EntityA == 0)
            {
                continue;
            }

            ASSERT(Arbiter->EntityB != 0);
            ApplyImpulses(Arbiter, dt);
        }
    }

    quaternion AngularVelocity;
    AngularVelocity.w = 0;
    for (i32 i = 1; i < GetWorld()->EntityCount; ++i)
    {
        rigid_body *Entity = GetEntityByHandle(i);
        if (Entity->Type == RigidBodyType_Dynamic)
        {
            Entity->Position += Entity->LinearVelocity * dt;
            AngularVelocity.v = Entity->AngularVelocity;
            Entity->Orientation += 0.5f * AngularVelocity * Entity->Orientation * dt;
        }
    }
}

void ClearAllEntities(world *World)
{
    ClearArena(&World->HullArena);
    World->EntityCount = 1;

    for (i32 i = 1; i < World->EntityCount; ++i)
    {
        // j = i+1 so we only loop over distinct pairs, otherwise we may eject an arbiter when we didnt mean to
        for (i32 j = i+1; j < World->EntityCount; ++j)
        {
            arbiter *Arbiter = GetArbiter(World, i, j);
            if (Arbiter)
            {
                *Arbiter = {};
            }
        }
    }
}

void ReinitSimulationState()
{
    world *World = GetWorld();

    ClearAllEntities(World);

    // @TODO: More demos
    i32 Entity = DEBUGCreateRigidBody(512, 512, 16, 0, V3(0,0,0), Rotation(V3(1,0,0), DegreesToRadians(20.f)));
    GetEntityByHandle(Entity)->Type = RigidBodyType_Static;
    for (i32 i = 0; i < 4; ++i)
    {
        for (i32 j = 0; j < 4; ++j)
        {
            DEBUGCreateRigidBody(32, 32, 24, 16, V3(i*48,j*48,256));
        }
    }

    for (i32 i = 0; i < 4; ++i)
    {
        for (i32 j = 0; j < 4; ++j)
        {
            DEBUGCreateRigidBody(32, 40, 32, 16, V3(16+i*48,j*48,320));
        }
    }

    for (i32 i = 1; i < GetWorld()->EntityCount; ++i)
    {
        GetEntityByHandle(i)->LinearMomentum = {};
        GetEntityByHandle(i)->AngularMomentum = {};
        GetEntityByHandle(i)->Recalculate();
        GetEntityByHandle(i)->RecalculateModelMatrix();
    }
}

void UpdateAndRender(float FrameTimeInSeconds, app_input *Input)
{
    if (!g_AppState.IsInitialized)
    {
        InitApp();
        g_AppState.IsInitialized = true;
        g_AppState.Accumulator = 0.f;


        world *World = GetWorld();
        World->MaxEntities = 512;
        World->EntityCount = 1; // Leave a 'null' entity at index 0.
        World->Entities = ArenaPushArray(PersistentArena(), World->MaxEntities, rigid_body);

        World->MaxArbiters = 2048;
        World->Arbiters = ArenaPushArray(PersistentArena(), World->MaxArbiters, arbiter);

        ReinitSimulationState();
    }
    ClearArena(TemporaryArena());

    world *World = GetWorld();
    render_group RenderGroup = {};
    camera *Camera = &World->Camera;

    {
        float Sensitivity = 0.001f;
        if (Input->MouseDown)
        {
            Camera->LatAngle -= Input->MouseDeltaX * Sensitivity;
            Camera->LngAngle += Input->MouseDeltaY * Sensitivity;
        }

        float ZoomSensitivity = 0.1f;
        float NewRadius = Camera->Radius - Input->MouseWheelDelta * ZoomSensitivity;
        if (NewRadius > 0)
        {
            Camera->Radius = NewRadius;
        }
    }

    float dt = 0.01f;
    if (g_AppState.IsSimulating)
    {
        g_AppState.Accumulator += FrameTimeInSeconds;
        while (g_AppState.Accumulator >= dt)
        {
            g_AppState.Accumulator -= dt;
            Simulate(dt);
        }
    }

    // @TODO: Save physics state from last frame and lerp between them here. 
    for (i32 i = 1; i < World->EntityCount; ++i)
    {
        rigid_body *Entity = GetEntityByHandle(i);
        PushAABB(&RenderGroup, Entity->DEBUGModel, Entity->Transform, i == 1 ? V3(0,0.5,1) : V3(1,0.5,0.2*i));
    }

    ImGui::NewFrame();

    ImGui::Begin("Controls", NULL, ImGuiWindowFlags_AlwaysAutoResize);
    const char *Text = g_AppState.IsSimulating ? "Pause Simulation" : "Unpause Simulation";
    if (ImGui::Button(Text))
    {
        g_AppState.IsSimulating = !g_AppState.IsSimulating;
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset Simulation State"))
    {
        ReinitSimulationState();
    }
    ImGui::SameLine();
    if (ImGui::Button("Step Forward"))
    {
        Simulate(dt);
    }

    ImGui::Separator();

    ImGui::SliderInt("Sequential Impulses Iterations", &World->SolverIterations, 1, 20);
    // ImGui::Checkbox("Show BVH visualization", &World->DEBUG_ShowBVH);
    ImGui::Text("SAT calls: %d", World->DEBUG_SATCalls);
    ImGui::Text("Collisions: %d", World->DEBUG_DetectedCollisions);

    ImGui::End();

    // @TODO: Pass a pointer instead
    RenderGroup.Camera = Camera;
    RendererDrawFrame(&RenderGroup);
}

