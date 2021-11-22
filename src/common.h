#define ASSERT(_expr) (!(_expr) ? __debugbreak() : (void)0)
#define ARRAY_SIZE(_array) (sizeof(_array)/sizeof(*(_array)))
#define KILOBYTES(_x) ((_x) << 10)
#define MEGABYTES(_x) ((_x) << 20)
#define GIGABYTES(_x) ((u64)(_x) << 30)

#define ARENA_COMMIT_ALIGNMENT KILOBYTES(4)

typedef uint8_t  u8;
typedef uint16_t u16; 
typedef uint32_t u32;
typedef uint64_t u64;

typedef int8_t  i8;
typedef int16_t i16; 
typedef int32_t i32;
typedef int64_t i64;

typedef size_t    usize;
typedef ptrdiff_t isize;

struct app_input
{
    bool MouseDown;
    float MouseDeltaX;
    float MouseDeltaY;
    float MouseWheelDelta;
};

struct arena
{
    void *Base;
    u64 Size;
    u64 AllocPosition;
    u64 CommitPosition;
};

struct list_header
{
    usize Capacity;
    usize Length;
};

#define ListHeader(_list) ((list_header*)(_list) - 1)

#define ListLength(_list)     ((_list) ? ListHeader(_list)->Length : 0)
#define ListCapacity(_list)   ((_list) ? ListHeader(_list)->Capacity : 0)

#define ListLast(_list)       ((_list)[ListHeader(_list)->Length - 1])

#define ListPush(_list, _val) \
    do { \
        ASSERT(ListLength(_list) < ListCapacity(_list)); \
        (_list)[ListHeader(_list)->Length++] = (_val); \
    } while(0)
#define ListPop(_list) (ListHeader(_list)->Length--, (_list)[ListHeader(_list)->Length])

#define ListInsertElements(_list, _index, _num) \
    do { \
        ListHeader(_list)->Length += (_num); \
        usize MoveLen = ListLength(_list) - (_index) - (_num); \
        memmove((_list)+(_index)+(_num), (_list)+(_index), sizeof *(_list) * MoveLen); \
    } while (0)

#define ListRemove(_list, _index) ListRemoveElements(_list, _index, 1)
#define ListRemoveElements(_list, _index, _num) \
    do { \
        usize MoveLen = ListLength(_list) - (_index) - (_num); \
        memmove((_list)+(_index), (_list)+(_index)+(_num), sizeof *(_list) * MoveLen); \
        ListHeader(_list)->Length -= (_num); \
    } while (0)

void* PlatformReserveMemory(u64 Size);
void PlatformCommitMemory(void *Memory, u64 Size);
void PlatformReleaseMemory(void *Memory);

inline arena CreateArena(u64 Size = GIGABYTES(1))
{
    arena Arena;
    Arena.Size = Size;
    Arena.Base = PlatformReserveMemory(Arena.Size);
    Arena.AllocPosition = 0;
    Arena.CommitPosition = 0;
    return Arena;
}

inline void ClearArena(arena *Arena)
{
    Arena->AllocPosition = 0;
}

inline void* GetArenaEnd(arena *Arena)
{
    void *Result = (u8*)Arena->Base + Arena->AllocPosition;
    return Result;
}

#define ArenaPushType(_arena, _type) (_type *)ArenaPushSize(_arena, sizeof(_type))
#define ArenaPushArray(_arena, _count, _type) (_type *)ArenaPushSize(_arena, sizeof(_type) * (_count))
#define ArenaPushList(_arena, _count, _type) (_type *)ArenaPushList_(_arena, sizeof(_type), (_count))

inline void *ArenaPushSize(arena *Arena, u64 Size)
{
    void *Result = NULL;
    if ((Arena->AllocPosition + Size) >= Arena->CommitPosition)
    {
        u64 CommitSize = (Arena->AllocPosition + Size) - Arena->CommitPosition;
        CommitSize += ARENA_COMMIT_ALIGNMENT - 1;
        CommitSize -= CommitSize % ARENA_COMMIT_ALIGNMENT;
        PlatformCommitMemory((u8*)Arena->Base + Arena->CommitPosition, CommitSize);
        Arena->CommitPosition += CommitSize;
    }
    Result = (u8*)Arena->Base + Arena->AllocPosition;
    Arena->AllocPosition += Size;
    memset(Result, 0, Size);
    return Result;
}

inline void *ArenaPushList_(arena *Arena, usize ElementSize, usize Capacity)
{
    void *Array = ArenaPushSize(Arena, sizeof(list_header) + ElementSize * Capacity);
    list_header *Header = (list_header*)Array;
    Header->Capacity = Capacity;
    return (void*)(Header+1);
}

