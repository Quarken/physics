v3 VelocityAtPoint(rigid_body *Body, v3 P)
{
    v3 X = PointToWorldSpace(Body->Hull->Centroid, Body->Transform);
    return Body->LinearVelocity + Cross(Body->AngularVelocity, P - X);
}

// @TODO: Maybe center of mass should just be the body's position (would simplify code)
v3 CenterOfMass(rigid_body *Body)
{
    v3 Centroid = Body->Hull->Centroid;
    return PointToWorldSpace(Centroid, Body->Transform);
}

arbiter *GetArbiter(world *World, i32 EntityA, i32 EntityB, arena *Arena = NULL)
{
    if (EntityA > EntityB)
    {
        // Ensure EntityA has the smallest ID
        i32 Temp = EntityA;
        EntityA = EntityB;
        EntityB = Temp;
    }

    // @TODO: Temporary basic hash function
    i32 HashIndex = (53 * EntityA + 97 * EntityB) & (World->MaxArbiters - 1);
    arbiter *Arbiter = World->Arbiters + HashIndex;
    while (Arbiter)
    {
        if ((Arbiter->EntityA == EntityA) &&
            (Arbiter->EntityB == EntityB))
        {
            break;
        }

        if (Arena)
        {
            // @TODO: Potentially very bad!!
            // In the chain of arbiters we might have alive -> dead -> alive
            // By mistake we will use the dead one in the chain even if the last alive one is the correct one!!
            
            // NOTE: An arbiter is invalid if EntityA is 0 (pointing to the null entity)
            if (Arbiter->EntityA == 0)
            {
                arbiter *Temp = Arbiter->NextArbiter;
                memset(Arbiter, 0, sizeof(*Arbiter));
                Arbiter->EntityA = EntityA;
                Arbiter->EntityB = EntityB;
                Arbiter->NextArbiter = Temp;
                break;
            }
            else if (!Arbiter->NextArbiter)
            {
                Arbiter->NextArbiter = ArenaPushType(Arena, arbiter);
                Arbiter->NextArbiter->EntityA = 0;
            }
        }

        Arbiter = Arbiter->NextArbiter;
    }
    return Arbiter;
}

void MergeContacts(arbiter *Arbiter, contact_manifold *NewManifold)
{
    float WarmStartingFactor = 1.f;
    world *World = GetWorld();

    contact_manifold MergedManifold = *NewManifold;
    contact_point *MergedContacts = MergedManifold.Points;

    for (int i = 0; i < NewManifold->PointCount; ++i)
    {
        contact_point *NewContact = NewManifold->Points + i;
        int k = -1;
        for (int j = 0; j < Arbiter->Manifold.PointCount; ++j)
        {
            contact_point *OldContact = Arbiter->Manifold.Points + j;
            if (NewContact->ID.Type == OldContact->ID.Type &&
                NewContact->ID.Value == OldContact->ID.Value)
            {
                k = j;
                break;
            }
        }

        if (k > -1)
        {
            contact_point *Contact = MergedContacts + i;
            contact_point *OldContact = Arbiter->Manifold.Points + k;
            World->DEBUG_ReusedContacts++;

            Contact->NormalImpulse = OldContact->NormalImpulse * WarmStartingFactor;
            Contact->FrictionImpulse = OldContact->FrictionImpulse * WarmStartingFactor;
            Contact->TangentImpulse = OldContact->TangentImpulse * WarmStartingFactor;
            Contact->BiasImpulse = OldContact->BiasImpulse * WarmStartingFactor;
        }
        else
        {
            World->DEBUG_NewContacts++;
            MergedContacts[i] = *NewContact;
        }
    }

    Arbiter->Manifold = MergedManifold;
}

void Broadphase(world *World, arena *Arena)
{
    i32 MaxPairs = 512;
    i32 PairCount = 0;
    collision_pair *Pairs = ArenaPushArray(TemporaryArena(), MaxPairs, collision_pair);

    // O(n^2) broadphase, @TODO: Replace with dynamic "fat" AABB tree
    for (i32 i = 1; i < World->EntityCount; ++i)
    {
        for (i32 j = i+1; j < World->EntityCount; ++j)
        {
            rigid_body *A = GetEntityByHandle(i);
            rigid_body *B = GetEntityByHandle(j);

            contact_manifold Manifold;
            bool Collision = CollideHulls(A->Hull, A->Transform, B->Hull, B->Transform, &Manifold);
            World->DEBUG_SATCalls++;

            if (Collision)
            {
                World->DEBUG_DetectedCollisions++;
                arbiter *Arbiter = GetArbiter(World, i, j, Arena);
                MergeContacts(Arbiter, &Manifold);
                Arbiter->WasUpdated = true;
                Pairs[PairCount++] = {i,j};
            }
        }
    }

    World->CollisionPairs = Pairs;
    World->CollisionPairCount = PairCount;

    for (i32 i = 1; i < World->EntityCount; ++i)
    {
        // j = i+1 so we only loop over distinct pairs, otherwise we may eject an arbiter when we didnt mean to
        for (i32 j = i+1; j < World->EntityCount; ++j)
        {
            arbiter *Arbiter = GetArbiter(World, i, j);
            if (Arbiter)
            {
                if (!Arbiter->WasUpdated)
                {
                    // This is an outdated arbiter, clear its contents so the memory can be reused in the future.
                    *Arbiter = {};
                }

                Arbiter->WasUpdated = false;
            }
        }
    }
}

void ApplyImpulses(arbiter *Arbiter, float dt)
{
    rigid_body *A = GetEntityByHandle(Arbiter->EntityA);
    rigid_body *B = GetEntityByHandle(Arbiter->EntityB);

    float Friction = 0.5f;
    float Slop = 0.01f;
    float BiasFactor = 0.2f;
    float inv_dt = 1.f / dt;
    v3 Normal = Normalized(Arbiter->Manifold.Normal);

    for (i32 i = 0; i < Arbiter->Manifold.PointCount; ++i)
    {
        contact_point *Contact = Arbiter->Manifold.Points + i;
        v3 CA = CenterOfMass(A);
        v3 CB = CenterOfMass(B);
        
        v3 RA = Contact->Position - CA;
        v3 RB = Contact->Position - CB;

        {
            v3 RelativeVelocity = 
                B->LinearVelocity + Cross(B->AngularVelocity, RB) -
                A->LinearVelocity - Cross(A->AngularVelocity, RA);

            float MassNormal = A->InverseMass + B->InverseMass;
            MassNormal += Dot(A->InverseInertia * Cross(Cross(RA, Normal), RA) +
                              B->InverseInertia * Cross(Cross(RB, Normal), RB),
                              Normal);
            MassNormal = 1.f / MassNormal;

            float Vn = Dot(RelativeVelocity, Normal);
            float Bias = -BiasFactor * inv_dt * Min(0.0f, Contact->Penetration + Slop);
            float NormalImpulse = MassNormal * (-Vn + Bias);
            float NormalImpulse0 = Contact->NormalImpulse;
            Contact->NormalImpulse = Max(NormalImpulse + NormalImpulse0, 0.0f);
            NormalImpulse = Contact->NormalImpulse - NormalImpulse0;
            v3 P = NormalImpulse * Normal;

            if (A->Type == RigidBodyType_Dynamic)
            {
                A->LinearMomentum -= P;
                A->AngularMomentum -= Cross(RA, P);
                A->Recalculate();
            }

            if (B->Type == RigidBodyType_Dynamic)
            {
                B->LinearMomentum += P;
                B->AngularMomentum += Cross(RB, P);
                B->Recalculate();
            }
        }

        {
            v3 RelativeVelocity = 
                B->LinearVelocity + Cross(B->AngularVelocity, RB) -
                A->LinearVelocity - Cross(A->AngularVelocity, RA);

            v3 Tangent = RelativeVelocity - Dot(RelativeVelocity, Normal) * Normal;
            if (!IsZeroVector(Tangent))
            {
                Tangent = Normalized(Tangent);
                float MassTangent = A->InverseMass + B->InverseMass;
                MassTangent += Dot(A->InverseInertia * Cross(Cross(RA, Tangent), RA) +
                                   B->InverseInertia * Cross(Cross(RB, Tangent), RB),
                                   Tangent);
                MassTangent = 1.f / MassTangent;
                float Vt = Dot(RelativeVelocity, Tangent);
                float TangentImpulse = MassTangent * (-Vt);
                float ImpulseClamp = Friction * Contact->NormalImpulse;
                float TangentImpulse0 = Contact->FrictionImpulse;
                Contact->FrictionImpulse = Clamp(TangentImpulse0 + TangentImpulse, -ImpulseClamp, ImpulseClamp);
                TangentImpulse = Contact->FrictionImpulse - TangentImpulse0;
                v3 P = TangentImpulse * Tangent;

                if (A->Type == RigidBodyType_Dynamic)
                {
                    A->LinearMomentum -= P;
                    A->AngularMomentum -= Cross(RA, P);
                    A->Recalculate();
                }

                if (B->Type == RigidBodyType_Dynamic)
                {
                    B->LinearMomentum += P;
                    B->AngularMomentum += Cross(RB, P);
                    B->Recalculate();
                }
            }
        }


        {
            v3 RelativeVelocity = 
                B->LinearVelocity + Cross(B->AngularVelocity, RB) -
                A->LinearVelocity - Cross(A->AngularVelocity, RA);

            v3 Tangent = RelativeVelocity - Dot(RelativeVelocity, Normal) * Normal;
            Tangent = Cross(Tangent, Normal);
            if (!IsZeroVector(Tangent))
            {
                Tangent = Normalized(Tangent);
                float MassTangent = A->InverseMass + B->InverseMass;
                MassTangent += Dot(A->InverseInertia * Cross(Cross(RA, Tangent), RA) +
                                   B->InverseInertia * Cross(Cross(RB, Tangent), RB),
                                   Tangent);
                MassTangent = 1.f / MassTangent;
                float Vt = Dot(RelativeVelocity, Tangent);
                float TangentImpulse = MassTangent * (-Vt);
                float ImpulseClamp = Friction * Contact->NormalImpulse;
                float TangentImpulse0 = Contact->FrictionImpulse;
                Contact->FrictionImpulse = Clamp(TangentImpulse0 + TangentImpulse, -ImpulseClamp, ImpulseClamp);
                TangentImpulse = Contact->FrictionImpulse - TangentImpulse0;
                v3 P = TangentImpulse * Tangent;

                if (A->Type == RigidBodyType_Dynamic)
                {
                    A->LinearMomentum -= P;
                    A->AngularMomentum -= Cross(RA, P);
                    A->Recalculate();
                }

                if (B->Type == RigidBodyType_Dynamic)
                {
                    B->LinearMomentum += P;
                    B->AngularMomentum += Cross(RB, P);
                    B->Recalculate();
                }
            }
        }
    }
}


