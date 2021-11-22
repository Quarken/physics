typedef i32 entity_handle;

struct bvh_node
{
    i32 NextFree;
    aabb BoundingVolume;
    i32 Parent;
    i32 LeftChild;
    i32 RightChild;

    // @TODO: We could use 0 as "no entity" and then Entity and IsLeaf can be merged into one.
    entity_handle Entity;
    bool IsLeaf;
};

struct bvh_tree
{
    float GrowFactor;
    i32 Root;
    i32 FreeList;
    i32 MaxNodes;
    i32 NodeCount;
    bvh_node *Nodes;
};

struct bvh_queue_item
{
    i32 NodeIndex;
    float InheritedCost;
};

struct bvh_node_pair
{
    i32 A, B;
};

enum rigid_body_type
{
    RigidBodyType_Static = 0,
    RigidBodyType_Dynamic 
};

struct rigid_body
{
    i32 Type;
    hull *Hull;
    aabb DEBUGModel;

    // State variables
    union
    {
        struct
        {
            v3 Position;
            quaternion Orientation;
        };
        transform Transform;
    };
    m4x4 ModelMatrix;
    aabb BoundingVolume;

    v3 LinearMomentum;
    v3 AngularMomentum;

    // Derived variables
    v3 LinearVelocity;
    v3 AngularVelocity;

    // Constants
    float Mass;
    float InverseMass;
    m3x3 Inertia;
    m3x3 InverseInertia;
    float Friction;

    void Recalculate()
    {
        LinearVelocity = InverseMass * LinearMomentum;
        m3x3 R = RotationMatrix3(Orientation);
        AngularVelocity = 
            R *
            InverseInertia *
            Transpose(R) *
            AngularMomentum;
        Orientation = Normalized(Orientation);
    }

    inline void RecalculateModelMatrix()
    {
        ModelMatrix = Translation(Position) * RotationMatrix(Orientation);
    }
};

struct collision_pair
{
    i32 EntityA;
    i32 EntityB;
};

enum
{
    ContactType_None = 0,
    ContactType_Edge,
    ContactType_Face
};

struct contact_point
{
    v3 Position;
    float Penetration;

    float NormalImpulse;
    float FrictionImpulse;
    float TangentImpulse;
    float BiasImpulse;

    struct
    {
        i32 Type;
        union
        {
            struct
            {
                i32 EdgeA;
                i32 EdgeB;
            };

            struct
            {
                i32 FaceA;
                i32 FaceB;
            };

            u64 Value;
        };
    } ID;
};

struct contact_manifold
{
    i32 PointCount;
    contact_point Points[4];
    v3 Normal;
};

struct arbiter
{
    bool WasUpdated;
    i32 EntityA;
    i32 EntityB;

    contact_manifold Manifold;

    arbiter *NextArbiter;
};

struct camera
{
    v3 FocusPosition;
    float LatAngle;
    float LngAngle;
    float Radius;

    v3 Position;
    v3 Direction;
};

struct world
{
    arena HullArena;
    camera Camera;

    bvh_tree BVH;
    i32 MaxEntities;
    i32 EntityCount;
    rigid_body *Entities;

    i32 CollisionPairCount;
    collision_pair *CollisionPairs;
    i32 MaxArbiters;
    arbiter *Arbiters;

    i32 SolverIterations;

    bool DEBUG_ShowBVH;
    i32 DEBUG_SATCalls;
    i32 DEBUG_DetectedCollisions;
    i32 DEBUG_ReusedContacts;
    i32 DEBUG_NewContacts;
};

struct app_state
{
    bool IsInitialized;
    bool IsSimulating;
    float Accumulator;
    arena PersistentArena;
    arena FrameArena;
    world World;
};

arena* TemporaryArena();
arena* PersistentArena();
world* GetWorld();
rigid_body* GetEntityByHandle(entity_handle Handle);

