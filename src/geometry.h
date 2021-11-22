#ifndef GEOMETRY_H
#define GEOMETRY_H

#define PLANE_THICKNESS_EPSILON 0.001f

struct transform
{
    v3 Position;
    quaternion Rotation;
};

struct plane
{
    v3 Normal;
    float Distance;
};

struct aabb
{
    v3 Min;
    v3 Max;
};

struct sphere
{
    v3 Center;
    float Radius;
};

struct half_edge
{
    u16 Next;
    u16 Twin;
    u16 Origin;
    u16 Face;
};

struct face
{
    u16 Edge;
};

struct hull
{
    v3 Centroid;
    i32 VertexCount;
    v3 *Vertices;
    i32 EdgeCount;
    half_edge *Edges;
    i32 FaceCount;
    face *Faces;
    plane *Planes;
};

struct polygon_vertex
{
    v3 Position;
    i32 FaceIndex;
};

struct polygon
{
    i32 VertexCount;
    polygon_vertex *Vertices;
};

struct face_query
{
    i32 Index;
    v3 Normal;
    float Separation;
};

struct edge_query
{
    i32 EdgeA;
    i32 EdgeB;
    v3 Normal;
    float Separation;
};

inline plane PlaneFromPoints(v3 a, v3 b, v3 c)
{
    plane Result;
    Result.Normal = Normalized(Cross(b-a, c-a));
    Result.Distance = Dot(Result.Normal, a);
    return Result;
}

inline plane PlaneFromVectorAndPoint(v3 a, v3 p)
{
    plane Result;
    Result.Normal = Normalized(a);
    Result.Distance = Dot(Result.Normal, p);
    return Result;
}

inline v3 ProjectPointOnPlane(plane Plane, v3 P)
{
    return P - (Dot(Plane.Normal, P) - Plane.Distance) * Plane.Normal;
}

inline float SignedDistance(plane Plane, v3 P)
{
    return Dot(Plane.Normal, P) - Plane.Distance;
}

enum
{
    PointPlane_Inside,
    PointPlane_Front,
    PointPlane_Back
};

int ClassifyPointToPlane(v3 Point, plane Plane)
{
    int Result = PointPlane_Inside;

    float Distance = SignedDistance(Plane, Point);
    if (Distance > PLANE_THICKNESS_EPSILON)
    {
        Result = PointPlane_Front;
    }
    else if (Distance < -PLANE_THICKNESS_EPSILON)
    {
        Result = PointPlane_Back;
    }

    return Result;
    
}

inline transform TransformCombine(transform A, transform B)
{
    transform Result;
    Result.Position = A.Position - B.Position;
    Result.Rotation = A.Rotation * Inverse(B.Rotation);
    return Result;
}

bool IntersectAABBAABB(aabb A, aabb B)
{
    if (A.Max[0] < B.Min[0] || A.Min[0] > B.Max[0]) return false;
    if (A.Max[1] < B.Min[1] || A.Min[1] > B.Max[1]) return false;
    if (A.Max[2] < B.Min[2] || A.Min[2] > B.Max[2]) return false;
    return true;
}

// Is A contained inside B?
bool InsideAABBAABB(aabb A, aabb B)
{
    if (A.Min[0] > B.Min[0] && A.Max[0] < B.Max[0] &&
        A.Min[1] > B.Min[1] && A.Max[1] < B.Max[1] &&
        A.Min[2] > B.Min[2] && A.Max[2] < B.Max[2])
    {
        return true;
    }
    else
    {
        return false;
    }
}

// @TODO: This should be replaced with a quickhull implementation eventually.
inline hull* DEBUGCreateBoxHull(arena *Arena, v3 Min, v3 Max)
{
    hull *Result = ArenaPushType(Arena, hull);
    for (i32 i = 0; i < 3; ++i)
    {
        Result->Centroid = Max*0.5f + Min*0.5f;
    }
    Result->VertexCount = 8;
    Result->EdgeCount = 24;
    Result->FaceCount = 6;
    Result->Vertices = ArenaPushArray(Arena, Result->VertexCount, v3);
    Result->Edges = ArenaPushArray(Arena, Result->EdgeCount, half_edge);
    Result->Faces = ArenaPushArray(Arena, Result->FaceCount, face);
    Result->Planes = ArenaPushArray(Arena, Result->FaceCount, plane);

    Result->Vertices[0] = Min;
    Result->Vertices[1] = V3(Min.x, Max.y, Min.z);
    Result->Vertices[2] = V3(Max.x, Max.y, Min.z);
    Result->Vertices[3] = V3(Max.x, Min.y, Min.z);
    Result->Vertices[4] = V3(Min.x, Max.y, Max.z);
    Result->Vertices[5] = V3(Min.x, Min.y, Max.z);
    Result->Vertices[6] = V3(Max.x, Min.y, Max.z);
    Result->Vertices[7] = Max;

    i32 FaceEdges[][4] = {
        {0, 2, 4, 6}, {11, 23, 19, 15},
        {7, 8, 10, 12}, {5, 21, 22, 9},
        {3, 17, 18, 20}, {1, 13, 14, 16}
    };

    i32 Origins[24] = {
        0, 1, 1, 2, 2, 3, 3, 0,
        3, 6, 6, 5, 5, 0, 5, 4,
        4, 1, 4, 7, 7, 2, 7, 6
    };

    for (i32 FaceIndex = 0;
         FaceIndex < Result->FaceCount;
         ++FaceIndex)
    {
        for (i32 i = 0; i < 4; ++i)
        {
            i32 EdgeIndex = FaceEdges[FaceIndex][i];
            i32 Delta = EdgeIndex % 2 == 0 ? 1 : -1;
            i32 TwinIndex = EdgeIndex + Delta;
            half_edge *Edge = Result->Edges + EdgeIndex;
            Edge->Next = FaceEdges[FaceIndex][(i+1)%4];
            Edge->Face = FaceIndex;
            Edge->Twin = TwinIndex;
            Edge->Origin = Origins[EdgeIndex];
        }

        Result->Faces[FaceIndex].Edge = FaceEdges[FaceIndex][0];
        half_edge *Edge = Result->Edges + Result->Faces[FaceIndex].Edge;
        v3 A = Result->Vertices[Edge->Origin];
        Edge = Result->Edges + Edge->Next;
        v3 B = Result->Vertices[Edge->Origin];
        Edge = Result->Edges + Edge->Next;
        v3 C = Result->Vertices[Edge->Origin];
        Result->Planes[FaceIndex] = PlaneFromPoints(A, B, C);
    }

    return Result;
}

#endif
