//
//
// @TODO: This implementation was written a while ago and needs to be redone as it is buggy
//        For now we use the O(n^2) naive broadphase algorithm
//
//

inline aabb Union(aabb A, aabb B)
{
    aabb Result;
    for (int i = 0; i < 3; ++i)
    {
        Result.Min[i] = Min(A.Min[i], B.Min[i]);
        Result.Max[i] = Max(A.Max[i], B.Max[i]);
    }
    return Result;
}

inline float SurfaceArea(aabb AABB)
{
    v3 D = AABB.Min - AABB.Max;
    return 2.f * (D.x * D.y + D.y * D.z + D.z * D.x);
}

void CreateBVH(bvh_tree *Tree, arena *Arena, i32 MaxNodes)
{
    // Fat AABBs are 20% bigger than the tight AABBs
    Tree->GrowFactor = 1.2f;
    // Store an extra node at the 0th index
    Tree->MaxNodes = MaxNodes+1;
    Tree->NodeCount = 1;
    Tree->Nodes = ArenaPushArray(Arena, Tree->MaxNodes, bvh_node);
}

i32 AllocateNode(bvh_tree *Tree, aabb BoundingVolume)
{
    i32 NodeIndex;
    bvh_node *Node;
    if (Tree->FreeList != 0)
    {
        NodeIndex = Tree->FreeList;
        Node = Tree->Nodes + NodeIndex;
        Tree->FreeList = Node->NextFree;
    }
    else
    {
        ASSERT((Tree->NodeCount+1) < Tree->MaxNodes);
        NodeIndex = Tree->NodeCount++;
        Node = Tree->Nodes + NodeIndex;
    }

    *Node = {};
    Node->BoundingVolume = BoundingVolume;
    return NodeIndex;
}

inline i32 AllocateLeafNode(bvh_tree *Tree, aabb BoundingVolume)
{
    i32 Index = AllocateNode(Tree, BoundingVolume);
    Tree->Nodes[Index].IsLeaf = true;
    return Index;
}

// @TODO: Looping over NodeCount wont work since we use a freelist.
float SurfaceAreaHeuristicCost(bvh_tree *Tree)
{
    float Sum = 0.f;
    for (i32 i = 0; i < Tree->NodeCount; ++i)
    {
        bvh_node *Node = Tree->Nodes + i;
        if (!Node->IsLeaf)
        {
            Sum += SurfaceArea(Node->BoundingVolume);
        }
    }
    return Sum;
}

i32 GetNodeSibling(bvh_tree *Tree, i32 NodeIndex)
{
    bvh_node *Node = Tree->Nodes + NodeIndex;
    bvh_node *Parent = Tree->Nodes + Node->Parent;
    if (Parent->RightChild == NodeIndex)
    {
        return Parent->LeftChild;
    }
    else
    {
        return Parent->RightChild;
    }
}

void Rotate(bvh_tree *Tree, i32 NodeIndexA, i32 NodeIndexB)
{
    ASSERT(NodeIndexA > 0 && NodeIndexB > 0);
    bvh_node *NodeA = Tree->Nodes + NodeIndexA;
    bvh_node *NodeB = Tree->Nodes + NodeIndexB;

    bvh_node *NodeAParent = Tree->Nodes + NodeA->Parent;
    if (NodeAParent->LeftChild == NodeIndexA)
    {
        NodeAParent->LeftChild = NodeIndexB;
    }
    else
    {
        NodeAParent->RightChild = NodeIndexB;
    }

    bvh_node *NodeBParent = Tree->Nodes + NodeB->Parent;
    if (NodeBParent->LeftChild == NodeIndexB)
    {
        NodeBParent->LeftChild = NodeIndexA;
    }
    else
    {
        NodeBParent->RightChild = NodeIndexA;
    }
}

void RefitAncestorsAndRotate(bvh_tree *Tree, i32 LeafIndex)
{
    i32 WalkIndex = Tree->Nodes[LeafIndex].Parent;
    while (WalkIndex != 0)
    {
        bvh_node *WalkNode = Tree->Nodes + WalkIndex;

        bvh_node *LeftChild = Tree->Nodes + WalkNode->LeftChild;
        bvh_node *RightChild = Tree->Nodes + WalkNode->RightChild;
        WalkNode->BoundingVolume = Union(LeftChild->BoundingVolume, RightChild->BoundingVolume);

        WalkIndex = WalkNode->Parent;
    }
}

void RefitAncestors(bvh_tree *Tree, i32 LeafIndex)
{
    i32 WalkIndex = Tree->Nodes[LeafIndex].Parent;
    while (WalkIndex != 0)
    {
        bvh_node *WalkNode = Tree->Nodes + WalkIndex;

        bvh_node *LeftChild = Tree->Nodes + WalkNode->LeftChild;
        bvh_node *RightChild = Tree->Nodes + WalkNode->RightChild;
        WalkNode->BoundingVolume = Union(LeftChild->BoundingVolume, RightChild->BoundingVolume);

        WalkIndex = WalkNode->Parent;
    }
}

i32 PickSibling(bvh_tree *Tree, aabb BoundingVolume, i32 NI)
{
    i32 Best = Tree->Root;
    float BestCost = SurfaceArea(Tree->Nodes[Best].BoundingVolume);
    float Area = SurfaceArea(BoundingVolume);

    // @TODO: Unsure if a priority queue is _really_ needed?
    // Could just do with a plain array instead.
    bvh_queue_item *PriorityQueue = ArenaPushList(TemporaryArena(), Tree->NodeCount, bvh_queue_item);
    bvh_queue_item Item;
    Item.NodeIndex = Tree->Root;
    Item.InheritedCost = 0.f;
    ListPush(PriorityQueue, Item);

    i32 Index = ListLength(PriorityQueue) - 1;
    while (ListLength(PriorityQueue) != 0)
    {
        bvh_queue_item CurrentItem = PriorityQueue[Index];
        bvh_node *CurrentNode = Tree->Nodes + CurrentItem.NodeIndex;

        ASSERT(CurrentItem.NodeIndex != NI);
        ASSERT(CurrentNode->RightChild != NI);
        ASSERT(CurrentNode->LeftChild != NI);

        float OldArea = SurfaceArea(CurrentNode->BoundingVolume);
        float NewArea = SurfaceArea(Union(CurrentNode->BoundingVolume, BoundingVolume));
        float Delta = NewArea - OldArea;

        float Cost = NewArea + CurrentItem.InheritedCost;
        if (Cost < BestCost)
        {
            Best = CurrentItem.NodeIndex;
            BestCost = Cost;
        }

        ListPop(PriorityQueue);

        float LowerBound = Area + Delta + CurrentItem.InheritedCost;
        if (LowerBound < BestCost && !CurrentNode->IsLeaf)
        {
            float ChildInheritedCost = Delta + CurrentItem.InheritedCost;

            i32 i;
            for (i = 0; i < ListLength(PriorityQueue); ++i)
            {
                if (PriorityQueue[i].InheritedCost < ChildInheritedCost)
                {
                    break;
                }
            }
            ListInsertElements(PriorityQueue, i, 2);
            PriorityQueue[i].NodeIndex = CurrentNode->LeftChild;
            PriorityQueue[i].InheritedCost = ChildInheritedCost;
            PriorityQueue[i+1].NodeIndex = CurrentNode->RightChild;
            PriorityQueue[i+1].InheritedCost = ChildInheritedCost;
        }
        
        Index = ListLength(PriorityQueue) - 1;
    }

    return Best;
}

void InsertLeaf(bvh_tree *Tree, aabb BoundingVolume, i32 EntityIndex)
{
    i32 LeafIndex = AllocateLeafNode(Tree, BoundingVolume);
    bvh_node *Leaf = Tree->Nodes + LeafIndex;
    Leaf->BoundingVolume = BoundingVolume;
    Leaf->Entity = EntityIndex;
    if (Tree->Root == 0)
    {
        Tree->Root = LeafIndex;
        return;
    }

    // Find the best sibling for this new leaf.
    i32 SiblingIndex = PickSibling(Tree, BoundingVolume, LeafIndex);

    // Create a new parent
    bvh_node *Sibling = Tree->Nodes + SiblingIndex;
    i32 OldParentIndex = Sibling->Parent;
    aabb NewParentVolume = Union(BoundingVolume, Sibling->BoundingVolume);
    i32 NewParentIndex = AllocateNode(Tree, NewParentVolume);

    bvh_node *OldParent = Tree->Nodes + OldParentIndex;
    bvh_node *NewParent = Tree->Nodes + NewParentIndex;

    if (OldParentIndex != 0)
    {
        if (OldParent->LeftChild == SiblingIndex)
        {
            OldParent->LeftChild = NewParentIndex;
        }
        else
        {
            OldParent->RightChild = NewParentIndex;
        }
    }
    else
    {
        // Chosen sibling was root.
        Tree->Root = NewParentIndex;
    }

    NewParent->Parent = OldParentIndex;
    NewParent->LeftChild = SiblingIndex;
    NewParent->RightChild = LeafIndex;
    Sibling->Parent = NewParentIndex;
    Leaf->Parent = NewParentIndex;

    // Walk up the tree to refit bounding volumes.
    RefitAncestorsAndRotate(Tree, LeafIndex);
}

void RemoveLeaf(bvh_tree *Tree, i32 NodeIndex)
{
    bvh_node *Node = Tree->Nodes + NodeIndex;

    ASSERT(NodeIndex != 0);
    ASSERT(Node->IsLeaf);
    ASSERT(Node->Entity != 0);

    if (Node->Parent != 0)
    {
        bvh_node *Parent = Tree->Nodes + Node->Parent;
        i32 SiblingIndex;
        ASSERT(Parent->LeftChild == NodeIndex ||
               Parent->RightChild == NodeIndex);

        if (Parent->LeftChild == NodeIndex)
        {
            SiblingIndex = Parent->RightChild;
        }
        else
        {
            SiblingIndex = Parent->LeftChild;
        }

        bvh_node *Sibling = Tree->Nodes + SiblingIndex;
        Sibling->Parent = Parent->Parent;
        if (Parent->Parent != 0)
        {
            bvh_node *GrandParent = Tree->Nodes + Parent->Parent;
            ASSERT(GrandParent->LeftChild == Node->Parent ||
                   GrandParent->RightChild == Node->Parent);
            if (GrandParent->LeftChild == Node->Parent)
            {
                GrandParent->LeftChild = SiblingIndex;
            }
            else
            {
                GrandParent->RightChild = SiblingIndex;
            }
        }
        else
        {
            Tree->Root = SiblingIndex;
        }

        i32 NextFree = Node->Parent;
        *Node = {};
        *Parent = {};
        Node->NextFree = NextFree;
        Parent->NextFree = Tree->FreeList;

        RefitAncestors(Tree, SiblingIndex);
    }
    else
    {
        Tree->Root = 0;
        *Node = {};
        Node->NextFree = Tree->FreeList;
    }

    Tree->FreeList = NodeIndex;
}

aabb TransformAABB(aabb A, m4x4 ModelMatrix)
{
    aabb Result;
    for (i32 i = 0; i < 3; ++i)
    {
        Result.Min[i] = ModelMatrix[i][3];
        Result.Max[i] = ModelMatrix[i][3];
        for (i32 j = 0; j < 3; ++j)
        {
            float e = ModelMatrix[i][j] * A.Min[j];
            float f = ModelMatrix[i][j] * A.Max[j];
            Result.Min[i] += (e < f ? e : f);
            Result.Max[i] += (e < f ? f : e);
        }
    }
    return Result;
}

aabb GrowAABB(aabb A, float Factor)
{
    aabb Result;
    for (i32 i = 0; i < 3; ++i)
    {
        float Length = A.Max[i] - A.Min[i];
        float AddLength = (Length * Factor - Length) * 0.5f;
        Result.Min[i] = A.Min[i] - AddLength;
        Result.Max[i] = A.Max[i] + AddLength;
    }
    return Result;
}

void UpdateBVH(bvh_tree *Tree)
{
    for (i32 NodeIndex = 0;
         NodeIndex < Tree->NodeCount;
         ++NodeIndex)
    {
        bvh_node *Node = Tree->Nodes + NodeIndex;
        if (!Node->IsLeaf) continue;

        // Save a local copy of entindex since we may remove this leaf and it gets cleared to zero.
        entity_handle EntityIndex = Node->Entity;
        rigid_body *Entity = GetEntityByHandle(EntityIndex);
        aabb TransformedBoundingVolume = TransformAABB(Entity->BoundingVolume, Entity->ModelMatrix);
        ASSERT(!IsZeroVector(TransformedBoundingVolume.Min) ||
               !IsZeroVector(TransformedBoundingVolume.Max));
        
        if (!InsideAABBAABB(TransformedBoundingVolume, Node->BoundingVolume))
        {
            // The tight AABB has gone outside the loose AABB, we need to reinsert.
            RemoveLeaf(Tree, NodeIndex);
            InsertLeaf(Tree, GrowAABB(TransformedBoundingVolume, Tree->GrowFactor), EntityIndex);
        }
    }
}

void InsertEntity(bvh_tree *Tree, entity_handle EntityIndex)
{
    rigid_body *Entity = GetEntityByHandle(EntityIndex);
    Entity->RecalculateModelMatrix();
    aabb BV = TransformAABB(Entity->BoundingVolume, Entity->ModelMatrix);
    InsertLeaf(Tree, GrowAABB(BV, Tree->GrowFactor), EntityIndex);
}

void QueryBVHForCollidingPairs(bvh_tree *Tree, collision_pair **Pairs, i32 *PairCount)
{
    // @TODO: Unsure of how big the stack should be, come back and fix this so it's not hardcoded!!
    bvh_node_pair *Stack = ArenaPushList(TemporaryArena(), 512, bvh_node_pair);
    bvh_node *Root = Tree->Nodes + Tree->Root;

    // @TODO: This may not be the nicest way to use the arena, maybe make a proper API?
    *Pairs = (collision_pair*)GetArenaEnd(TemporaryArena());
    i32 CollisionCount = 0;

    // Only need to check one child
    if (Root->LeftChild == 0)
    {
        return;
    }

    bvh_node_pair Pair = {Root->LeftChild, Root->RightChild};
    ListPush(Stack, Pair);

    while (ListLength(Stack) != 0)
    {
        Pair = ListPop(Stack);

        bvh_node *NodeA = Tree->Nodes + Pair.A;
        bvh_node *NodeB = Tree->Nodes + Pair.B;

        if (IntersectAABBAABB(NodeA->BoundingVolume, NodeB->BoundingVolume))
        {
            if (NodeA->IsLeaf && NodeB->IsLeaf)
            {
                // @TODO: Check the tight AABBs
                // (currently the list of colliding pairs is built from the fat AABBs)
                CollisionCount++;
                collision_pair *Pair = ArenaPushType(TemporaryArena(), collision_pair);
                Pair->EntityA = NodeA->Entity;
                Pair->EntityB = NodeB->Entity;
            }
            else if (NodeA->IsLeaf && !NodeB->IsLeaf)
            {
                ListPush(Stack, (bvh_node_pair{Pair.A, NodeB->LeftChild}));
                ListPush(Stack, (bvh_node_pair{Pair.A, NodeB->RightChild}));
                ListPush(Stack, (bvh_node_pair{NodeB->LeftChild, NodeB->RightChild}));
            }
            else if (!NodeA->IsLeaf && NodeB->IsLeaf)
            {
                ListPush(Stack, (bvh_node_pair{Pair.B, NodeA->LeftChild}));
                ListPush(Stack, (bvh_node_pair{Pair.B, NodeA->RightChild}));
                ListPush(Stack, (bvh_node_pair{NodeA->LeftChild, NodeA->RightChild}));
            }
            else
            {
                ListPush(Stack, (bvh_node_pair{NodeA->LeftChild,  NodeB->LeftChild}));
                ListPush(Stack, (bvh_node_pair{NodeA->LeftChild,  NodeB->RightChild}));
                ListPush(Stack, (bvh_node_pair{NodeA->RightChild, NodeB->LeftChild}));
                ListPush(Stack, (bvh_node_pair{NodeA->RightChild, NodeB->RightChild}));

                ListPush(Stack, (bvh_node_pair{NodeA->LeftChild, NodeA->RightChild}));
                ListPush(Stack, (bvh_node_pair{NodeB->LeftChild, NodeB->RightChild}));
            }
        }
    }

    *PairCount = CollisionCount;
}
