inline v3 PointToWorldSpace(v3 Point, transform T)
{
    Point = RotateVector(Point, T.Rotation);
    Point += T.Position;
    return Point;
}

inline v3 PointToLocalSpace(v3 Point, transform T)
{
    Point -= T.Position;
    Point = RotateVector(Point, Inverse(T.Rotation));
    return Point;
}

inline v3 DirectionToLocalSpaceOfB(v3 Direction, transform A, transform B)
{
    Direction = RotateVector(Direction, A.Rotation);
    Direction = RotateVector(Direction, Inverse(B.Rotation));
    return Direction;
}

inline v3 PointToLocalSpaceOfB(v3 Point, transform A, transform B)
{
    Point = PointToWorldSpace(Point, A);
    Point = PointToLocalSpace(Point, B);
    return Point;
}

inline v3 EdgePlaneIntersection(v3 A, v3 B, plane Plane)
{
    v3 AB = B - A;
    float t = (Plane.Distance - Dot(Plane.Normal, A)) / Dot(Plane.Normal, AB);
    ASSERT(t >= 0.0f && t <= 1.0f);
    return A + AB * t;
}

polygon ClipPolygonBack(polygon Polygon, plane ClipPlane, i32 FaceIndex)
{
    i32 OutIndex = 0;
    polygon_vertex *Output = ArenaPushArray(TemporaryArena(), Polygon.VertexCount+2, polygon_vertex);

    polygon_vertex A = Polygon.Vertices[Polygon.VertexCount-1];
    int ASide = ClassifyPointToPlane(A.Position, ClipPlane);
    for (i32 i = 0; i < Polygon.VertexCount; ++i)
    {
        polygon_vertex B = Polygon.Vertices[i];
        int BSide = ClassifyPointToPlane(B.Position, ClipPlane);

        if (BSide == PointPlane_Front)
        {
            if (ASide == PointPlane_Back)
            {
                v3 Intersection = EdgePlaneIntersection(A.Position, B.Position, ClipPlane);
                ASSERT(ClassifyPointToPlane(Intersection, ClipPlane) == PointPlane_Inside);
                Output[OutIndex++] = { Intersection, FaceIndex };
            }
        }
        else if (BSide == PointPlane_Back)
        {
            if (ASide == PointPlane_Front)
            {
                v3 Intersection = EdgePlaneIntersection(A.Position, B.Position, ClipPlane);
                ASSERT(ClassifyPointToPlane(Intersection, ClipPlane) == PointPlane_Inside);
                Output[OutIndex++] = { Intersection, FaceIndex };
            }
            else if (ASide == PointPlane_Inside)
            {
                Output[OutIndex++] = A;
            }
            Output[OutIndex++] = B;
        }
        else if (ASide == PointPlane_Back)
        {
            Output[OutIndex++] = B;
        }

        A = B;
        ASide = BSide;
    }

    polygon Result;
    Result.VertexCount = OutIndex;
    Result.Vertices = Output;
    return Result;
}

v3 HullSupport(hull *Hull, v3 Direction)
{
    v3 Result = {};
    float Max = -FLT_MAX;
    for (i32 i = 0; i < Hull->VertexCount; ++i)
    {
        v3 Vertex = Hull->Vertices[i];
        float Projection = Dot(Vertex, Direction);
        if (Projection > Max)
        {
            Max = Projection;
            Result = Vertex;
        }
    }

    return Result;
}

face_query SATQueryFaces(hull *A, transform TA, hull *B, transform TB)
{
    face_query Result;
    Result.Index = -1;
    Result.Normal = {};
    Result.Separation = -FLT_MAX;

    for (i32 Index = 0;
         Index < A->FaceCount;
         ++Index)
    {
        plane Plane = A->Planes[Index];
        v3 BSpaceNormal = DirectionToLocalSpaceOfB(Plane.Normal, TA, TB);

        i32 VertexAIndex = A->Edges[A->Faces[Index].Edge].Origin;
        v3 VertexA = PointToLocalSpaceOfB(A->Vertices[VertexAIndex], TA, TB);
        v3 VertexB = HullSupport(B, -BSpaceNormal);
        float Separation = Dot(BSpaceNormal, VertexB - VertexA);

        if (Separation > Result.Separation)
        {
            Result.Separation = Separation;
            Result.Index = Index;
            Result.Normal = RotateVector(Plane.Normal, TA.Rotation);
        }
    }

    return Result;
}

// @TODO: This can be improved by using gauss maps for edge pruning (as per Valve GDC paper)
edge_query SATQueryEdges(hull *A, transform TA, hull *B, transform TB)
{
    edge_query Result;
    Result.EdgeA = -1;
    Result.EdgeB = -1;
    Result.Normal = {};
    Result.Separation = -FLT_MAX;

    v3 C1 = PointToLocalSpaceOfB(A->Centroid, TA, TB);

    for (i32 Index1 = 0; Index1 < A->EdgeCount; Index1 += 2)
    {
        half_edge *Edge1 = A->Edges + Index1;
        half_edge *Twin1 = A->Edges + Index1 + 1;

        ASSERT(Edge1->Twin == (Index1 + 1));
        ASSERT(Twin1->Twin == Index1);

        v3 P1 = PointToLocalSpaceOfB(A->Vertices[Edge1->Origin], TA, TB);
        v3 Q1 = PointToLocalSpaceOfB(A->Vertices[Twin1->Origin], TA, TB);
        v3 E1 = Q1 - P1;

        for (i32 Index2 = 0; Index2 < B->EdgeCount; Index2 += 2)
        {
            half_edge *Edge2 = B->Edges + Index2;
            half_edge *Twin2 = B->Edges + Index2 + 1;

            ASSERT(Edge2->Twin == (Index2 + 1));
            ASSERT(Twin2->Twin == Index2);

            v3 P2 = B->Vertices[Edge2->Origin];
            v3 Q2 = B->Vertices[Twin2->Origin];
            v3 E2 = Q2 - P2;

            v3 Axis = Cross(E1, E2);
            if (IsZeroVector(Axis))
            {
                continue;
            }

            if (Dot(Axis, P1 - C1) < 0.0f)
            {
                Axis = -Axis;
            }

            Axis = Normalized(Axis);

            // Transform the axis into A's local space so we can find the support.
            v3 AxisA = DirectionToLocalSpaceOfB(Axis, TB, TA);
            v3 VertexA = PointToLocalSpaceOfB(HullSupport(A, AxisA), TA, TB);

            v3 VertexB = HullSupport(B, -Axis);
            float Separation = Dot(Axis, VertexB - VertexA);

            if (Separation > Result.Separation)
            {
                Result.EdgeA = Index1;
                Result.EdgeB = Index2;
                Result.Normal = RotateVector(Axis, TB.Rotation);
                Result.Separation = Separation;
            }
        }
    }

    return Result;
}

void BuildFaceContact(face_query FaceQuery, hull *HullA, transform TA,
                      hull *HullB, transform TB, contact_manifold *Manifold)
{
    Manifold->Normal = FaceQuery.Normal;

    i32 IncidentFace = -1;
    float MinProj = FLT_MAX;
    plane ReferenceFacePlane = HullA->Planes[FaceQuery.Index];
    v3 ReferenceNormal = DirectionToLocalSpaceOfB(ReferenceFacePlane.Normal, TA, TB);
    for (i32 i = 0; i < HullB->FaceCount; ++i)
    {
        v3 Normal = HullB->Planes[i].Normal;
        float Proj = Dot(Normal, ReferenceNormal);
        if (Proj < MinProj)
        {
            IncidentFace = i;
            MinProj = Proj;
        }
    }

    // @TODO: The following clipping part can probably be improved.
    i32 FaceEdgeCount = 1;
    i32 StartEdge = HullB->Faces[IncidentFace].Edge;
    {
        half_edge *Edge = HullB->Edges + StartEdge;
        while (Edge->Next != StartEdge)
        {
            FaceEdgeCount++;
            Edge = HullB->Edges + Edge->Next;
        }
    }

    polygon Polygon;
    Polygon.VertexCount = FaceEdgeCount;
    Polygon.Vertices = ArenaPushArray(TemporaryArena(), FaceEdgeCount, polygon_vertex);

    half_edge *Edge = HullB->Edges + StartEdge;
    for (i32 i = 0; i < FaceEdgeCount; ++i)
    {
        // Move every vertex to the local space of A so we dont have to transform the clip planes.
        Polygon.Vertices[i] = { PointToLocalSpaceOfB(HullB->Vertices[Edge->Origin], TB, TA), FaceQuery.Index };
        Edge = HullB->Edges + Edge->Next;
    }

    StartEdge = HullA->Faces[FaceQuery.Index].Edge;
    i32 CurrentEdge = StartEdge;
    Edge = HullA->Edges + CurrentEdge;
    do
    {
        i32 ClipFace = HullA->Edges[Edge->Twin].Face;
        Polygon = ClipPolygonBack(Polygon, HullA->Planes[ClipFace], ClipFace);
        CurrentEdge = Edge->Next;
        Edge = HullA->Edges + CurrentEdge;
    }
    while (CurrentEdge != StartEdge);

    // Pick a direction for contact point reduction.
    v3 EdgeA = HullA->Vertices[Edge->Origin];
    v3 EdgeB = HullA->Vertices[HullA->Edges[Edge->Next].Origin];
    v3 Direction = EdgeB - EdgeA;

    i32 PointCount = 0;
    contact_point *Points = ArenaPushArray(TemporaryArena(), Polygon.VertexCount, contact_point);

    float MaxDepth = FLT_MAX;
    i32 ContactAIndex;

    // Cull contact points above reference face and project onto plane.
    for (i32 i = 0; i < Polygon.VertexCount; ++i)
    {
        polygon_vertex Point = Polygon.Vertices[i];
        float Depth = SignedDistance(ReferenceFacePlane, Point.Position);
        if (Depth <= 0.f)
        {
            i32 Index = PointCount++;
            Points[Index].Position = ProjectPointOnPlane(ReferenceFacePlane, Point.Position);
            Points[Index].Penetration = Depth;
            Points[Index].ID.Type = ContactType_Face;
            Points[Index].ID.FaceA = Point.FaceIndex;
            Points[Index].ID.FaceB = IncidentFace;
            if (Depth < MaxDepth)
            {
                MaxDepth = Depth;
                ContactAIndex = Index;
            }
        }
    }

    if (PointCount > 4)
    {
        Manifold->PointCount = 4;

        Manifold->Points[0] = Points[ContactAIndex];
        Points[ContactAIndex] = Points[PointCount-1];
        PointCount--;

        v3 A = Manifold->Points[0].Position;

        i32 ContactBIndex;
        float MaxABLength = -FLT_MAX;
        for (i32 i = 0; i < PointCount; ++i)
        {
            v3 AB = Points[i].Position - A;
            float ABLength = LengthSquared(AB);
            if (ABLength > MaxABLength)
            {
                ContactBIndex = i;
                MaxABLength = ABLength;
            }
        }

        Manifold->Points[1] = Points[ContactBIndex];
        Points[ContactBIndex] = Points[PointCount-1];
        PointCount--;

        v3 B = Manifold->Points[1].Position;

        i32 ContactCIndex;
        float MaxArea = -FLT_MAX;
        for (i32 i = 0; i < PointCount; ++i)
        {
            v3 C = Points[i].Position;
            v3 CA = A - C;
            v3 CB = B - C;
            float Area = 0.5f * Dot(Cross(CA, CB), Manifold->Normal);
            if (Area > MaxArea)
            {
                ContactCIndex = i;
                MaxArea = Area;
            }
        }

        Manifold->Points[2] = Points[ContactCIndex];
        Points[ContactCIndex] = Points[PointCount-1];
        PointCount--;

        v3 C = Manifold->Points[2].Position;

        i32 ContactDIndex;
        MaxArea = FLT_MAX;
        for (i32 i = 0; i < PointCount; ++i)
        {
            v3 D = Points[i].Position;
            
            v3 DA = A - D;
            v3 DB = B - D;
            v3 DC = C - D;

            float AreaABD = 0.5f * Dot(Cross(DA, DB), Manifold->Normal);
            float AreaBCD = 0.5f * Dot(Cross(DB, DC), Manifold->Normal);
            float AreaCAD = 0.5f * Dot(Cross(DC, DA), Manifold->Normal);

            if (AreaABD < MaxArea)
            {
                MaxArea = AreaABD;
                ContactDIndex = i;
            }

            if (AreaBCD < MaxArea)
            {
                MaxArea = AreaBCD;
                ContactDIndex = i;
            }

            if (AreaCAD < MaxArea)
            {
                MaxArea = AreaCAD;
                ContactDIndex = i;
            }
        }

        Manifold->Points[3] = Points[ContactDIndex];

        ASSERT(Manifold->PointCount == 4);
        for (i32 i = 0; i < Manifold->PointCount; ++i)
        {
            Manifold->Points[i].Position = PointToWorldSpace(Manifold->Points[i].Position, TA);
        }
    }
    else
    {
        Manifold->PointCount = PointCount;
        for (i32 i = 0; i < PointCount; ++i)
        {
            contact_point *Point = Manifold->Points + i;
            Point->Position = PointToWorldSpace(Points[i].Position, TA);
            Point->Penetration = Points[i].Penetration;
        }
    }
}

// @TODO: Optimization as per Ericson page 151.
void ClosestPointsSegmentSegment(v3 P1, v3 Q1, v3 P2, v3 Q2,
                                 v3 *C1, v3 *C2)
{
    v3 E1 = Q1 - P1;
    v3 E2 = Q2 - P2;
    v3 r = P1 - P2;

    float a = Dot(E1, E1);
    float e = Dot(E2, E2);
    float f = Dot(E2, r);

    if (a <= ML_EPSILON && e <= ML_EPSILON)
    {
        // Squared lengths are close to zero, both segments are just points.
        *C1 = P1;
        *C2 = P2;
        return;
    }

    float s, t;
    if (a <= ML_EPSILON)
    {
        s = 0;
        t = f / e;
        t = Clamp(t, 0.f, 1.f);
    }
    else
    {
        float c = Dot(E1, r);
        if (e <= ML_EPSILON)
        {
            t = 0;
            s = Clamp(-c / a, 0.f, 1.f);
        }
        else
        {
            // Neither segment is degenerate here.
            float b = Dot(E1, E2);
            float Denom = a*e - b*b;
            if (Denom != 0.f)
            {
                s = Clamp((b*f - c*e) / Denom, 0.f, 1.f);
            }
            else
            {
                s = 0.f;
            }

            t = (b*s + f) / e;
            if (t < 0.f)
            {
                t = 0.f;
                s = Clamp(-c / a, 0.f, 1.f);
            }
            else if (t > 1.f)
            {
                t = 1.f;
                s = Clamp((b - c) / a, 0.f, 1.f);
            }
        }
    }

    *C1 = P1 + E1 * s;
    *C2 = P2 + E2 * t;
}

void BuildEdgeContact(edge_query EdgeQuery, hull *A, transform  TA, hull *B, transform TB, contact_manifold *Manifold)
{
    half_edge *EdgeA = A->Edges + EdgeQuery.EdgeA;
    half_edge *EdgeANext = A->Edges + EdgeA->Next;

    half_edge *EdgeB = B->Edges + EdgeQuery.EdgeB;
    half_edge *EdgeBNext = B->Edges + EdgeB->Next;

    v3 P1 = PointToLocalSpaceOfB(A->Vertices[EdgeA->Origin], TA, TB);
    v3 Q1 = PointToLocalSpaceOfB(A->Vertices[EdgeANext->Origin], TA, TB);

    v3 P2 = B->Vertices[EdgeB->Origin];
    v3 Q2 = B->Vertices[EdgeBNext->Origin];

    v3 C1, C2;
    ClosestPointsSegmentSegment(P1, Q1, P2, Q2, &C1, &C2);

    v3 Direction = C2 - C1;
    v3 MidPoint = C1 + Direction * 0.5f;

    Manifold->PointCount = 1;
    Manifold->Points[0].Penetration = EdgeQuery.Separation;
    Manifold->Points[0].Position = PointToWorldSpace(MidPoint, TB);
    Manifold->Points[0].ID.Type = ContactType_Edge;
    Manifold->Points[0].ID.EdgeA = EdgeQuery.EdgeA;
    Manifold->Points[0].ID.EdgeB = EdgeQuery.EdgeB;
    Manifold->Normal = EdgeQuery.Normal;
}

bool CollideHulls(hull *A, transform TA, hull *B, transform TB, contact_manifold *Manifold)
{
    memset(Manifold, 0, sizeof(*Manifold));

    face_query FaceQueryA = SATQueryFaces(A, TA, B, TB);
    if (FaceQueryA.Separation > 0.0f)
    {
        return false;
    }

    face_query FaceQueryB = SATQueryFaces(B, TB, A, TA);
    if (FaceQueryB.Separation > 0.0f)
    {
        return false;
    }

    edge_query EdgeQuery = SATQueryEdges(A, TA, B, TB);
    if (EdgeQuery.Separation > 0.0f)
    {
        return false;
    }

    Manifold->PointCount = 1;
    float Bias = 1.f; // Bias to prefer building face contacts.
    bool IsEdgeContact = 
        FaceQueryA.Separation < (EdgeQuery.Separation - Bias) &&
        FaceQueryB.Separation < (EdgeQuery.Separation - Bias);
    if (IsEdgeContact)
    {
        BuildEdgeContact(EdgeQuery, A, TA, B, TB, Manifold);
    }
    else if (FaceQueryA.Separation > FaceQueryB.Separation)
    {
        BuildFaceContact(FaceQueryA, A, TA, B, TB, Manifold);
    }
    else
    {
        BuildFaceContact(FaceQueryB, B, TB, A, TA, Manifold);
        // Negate such that the normal consistently points towards B.
        Manifold->Normal = -Manifold->Normal;
    }

    return true;
}

bool CollideSpheres(sphere a, sphere b, contact_manifold *Manifold)
{
    bool Result = false;
    v3 Diff = b.Center - a.Center;
    float Distance = Length(Diff);
    float PenetrationDepth = Distance - a.Radius - b.Radius;
    if (PenetrationDepth <= 0)
    {
        Result = true;
        v3 Normal = Diff / Distance;
        Manifold->PointCount = 1;
        Manifold->Points[0].Position = a.Center + Normal *  (a.Radius - fabs(PenetrationDepth)/2.f);
        Manifold->Points[0].Penetration = PenetrationDepth;
        Manifold->Normal = Normal;
    }
    return Result;
}

v3 ClosestPointAABB(v3 Point, aabb Box)
{
    v3 Result = {};
    for (int i = 0; i < 3; i++)
    {
        float Value = Point[i];
        if (Value < Box.Min[i])
        {
            Value = Box.Min[i];
        }
        if (Value > Box.Max[i])
        {
            Value = Box.Max[i];
        }
        Result[i] = Value;
    }
    return Result;
}

bool CollideSphereAABB(sphere Sphere, aabb Box, contact_manifold *Manifold)
{
    bool Result = false;
    Box = Box;
    v3 ClosestPoint = ClosestPointAABB(Sphere.Center, Box);
    v3 Diff = Sphere.Center - ClosestPoint;
    float Distance = Length(Diff);
    // @TODO: This function doesn't handle the case where sphere center is inside the AABB.
    float PenetrationDepth = Distance - Sphere.Radius;
    if (PenetrationDepth <= 0)
    {
        Result = true;
        Manifold->PointCount = 1;
        Manifold->Points[0].Position = ClosestPoint;
        Manifold->Points[0].Penetration = PenetrationDepth;
        Manifold->Normal = -(Diff / Distance);
    }
    return Result;
}



