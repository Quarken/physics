enum render_command_type
{
    RenderCommandType_None,
    RenderCommandType_Sphere,
    RenderCommandType_AABB,

    DEBUG_RenderCommandType_Hull,
    DEBUG_RenderCommandType_BVH,

    RenderCommandType_MAX,
};

struct render_command_aabb
{
    aabb AABB;
    transform Transform;
    v3 Color;
};

struct render_command
{
    render_command_type Type;
    union
    {
        sphere Sphere;
        render_command_aabb AABB;

        hull *DEBUGHull;
        bvh_tree *DEBUGBVHTree;
    };
};

struct render_group
{
    i32 CmdsCount;
    render_command Cmds[128];
    camera *Camera;
};

inline render_command* PushRenderCommand(render_group *Group, render_command_type CommandType)
{
    i32 Index = Group->CmdsCount++;
    ASSERT(Index < ARRAY_SIZE(Group->Cmds));
    render_command *Cmd = Group->Cmds + Index;
    Cmd->Type = CommandType;
    return Cmd;
}

inline void PushSphere(render_group *Group, sphere Sphere)
{
    render_command *Cmd = PushRenderCommand(Group, RenderCommandType_Sphere);
    Cmd->Sphere = Sphere;
}

inline void PushAABB(render_group *Group, aabb AABB, transform Transform, v3 Color = V3(1.0f, 0.5f, 0.0f))
{
    render_command *Cmd = PushRenderCommand(Group, RenderCommandType_AABB);
    Cmd->AABB.AABB = AABB;
    Cmd->AABB.Transform = Transform;
    Cmd->AABB.Color = Color;
}

inline void DEBUGPushHull(render_group *Group, hull *Hull)
{
    render_command *Cmd = PushRenderCommand(Group, DEBUG_RenderCommandType_Hull);
    Cmd->DEBUGHull = Hull;
}

inline void DEBUGPushBVHVis(render_group *Group, bvh_tree *Tree)
{
    render_command *Cmd = PushRenderCommand(Group, DEBUG_RenderCommandType_BVH);
    Cmd->DEBUGBVHTree = Tree;
}

m4x4 CalculateCameraViewMatrix(camera *Camera)
{
    v3 CamX, CamY, CamZ;
    CamZ.x = cosf(Camera->LngAngle) * cosf(Camera->LatAngle);
    CamZ.y = cosf(Camera->LngAngle) * sinf(Camera->LatAngle);
    CamZ.z = sinf(Camera->LngAngle);

    float LngAnglePerp = Camera->LngAngle + M_PI_2;
    CamY.x = cosf(LngAnglePerp) * cosf(Camera->LatAngle);
    CamY.y = cosf(LngAnglePerp) * sinf(Camera->LatAngle);
    CamY.z = sinf(LngAnglePerp);

    CamX = Normalized(Cross(CamY, CamZ));
    Camera->Direction = -CamZ;
    Camera->Position = Camera->FocusPosition + CamZ * Camera->Radius;
    return CameraMatrix(Camera->Position, CamX, CamY, CamZ);
}

void RendererGetBackbufferSize(i32 *Width, i32 *Height);
void RendererInitialize();
void RendererDrawFrame(render_group *RenderGroup);

