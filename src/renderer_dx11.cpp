#include <d3d11.h>

#include "generated/vertex_shader.h"
#include "generated/pixel_shader.h"
#include "generated/dvertex_shader.h"
#include "generated/dpixel_shader.h"

#define DX_MAP(_ctx, _var, ...) for (int _index_ = ((_ctx)->Map(_var, __VA_ARGS__), 0); !_index_; ++_index_, (_ctx)->Unmap(_var, 0))

struct dx_shader
{
    ID3D11VertexShader *VertexShader;
    ID3D11PixelShader *PixelShader;
    ID3D11InputLayout *InputLayout;
};

struct vertex
{
    v3 Position;
    v3 Normal;
    v3 Color;
};

struct debug_vertex
{
    v3 Position;
    v3 Color;
};

#pragma pack(push,4)
struct alignas(16) DXConstantBuffer
{
    m4x4 Model;
    m4x4 ViewProjection;
    v3 CameraDirection;
};
#pragma pack(pop)

static struct
{
    HWND Window;
    IDXGISwapChain *SwapChain;
    ID3D11Device *Device;
    ID3D11DeviceContext *Context;
    ID3D11Texture2D *Backbuffer;
    ID3D11RenderTargetView *BackbufferView;
    ID3D11DepthStencilView *DepthStencilView;
    ID3D11DepthStencilState *DepthStencilState;

    ID3D11Buffer *ConstantBuffer;
    dx_shader Shader;
    dx_shader DebugShader;

    ID3D11Buffer *BoxVertexBuffer;
    ID3D11Buffer *HullVertexBuffer;
    ID3D11Buffer *ScratchVertexBuffer;

    i32 SphereIndexCount;
    ID3D11Buffer *SphereVertexBuffer;
    ID3D11Buffer *SphereIndexBuffer;
} Renderer;

static inline void DXCreateShader(dx_shader *Shader,
                                  const void *VertexShaderCode, SIZE_T VertexShaderCodeSize,
                                  const void *PixelShaderCode, SIZE_T PixelShaderCodeSize,
                                  D3D11_INPUT_ELEMENT_DESC *InputElementDesc, UINT InputElementDescSize)
{
    Renderer.Device->CreateVertexShader(VertexShaderCode, VertexShaderCodeSize, 0, &Shader->VertexShader);
    Renderer.Device->CreatePixelShader(PixelShaderCode, PixelShaderCodeSize, 0, &Shader->PixelShader);
    Renderer.Device->CreateInputLayout(InputElementDesc, InputElementDescSize,
                                       VertexShaderCode, VertexShaderCodeSize,
                                       &Shader->InputLayout);

    ASSERT(Shader->VertexShader);
    ASSERT(Shader->PixelShader);
    ASSERT(Shader->InputLayout);
}

static inline void DXUseShader(dx_shader Shader)
{
    Renderer.Context->VSSetShader(Shader.VertexShader, 0, 0);
    Renderer.Context->PSSetShader(Shader.PixelShader, 0, 0);
    Renderer.Context->IASetInputLayout(Shader.InputLayout);
}

static void DXInitShaders()
{
    D3D11_INPUT_ELEMENT_DESC InputElementDesc[] = {
        {"POSITION",0,DXGI_FORMAT_R32G32B32_FLOAT,0,D3D11_APPEND_ALIGNED_ELEMENT,D3D11_INPUT_PER_VERTEX_DATA,0},
        {"NORMAL",0,DXGI_FORMAT_R32G32B32_FLOAT,0,D3D11_APPEND_ALIGNED_ELEMENT,D3D11_INPUT_PER_VERTEX_DATA,0},
        {"COLOR",0,DXGI_FORMAT_R32G32B32_FLOAT,0,D3D11_APPEND_ALIGNED_ELEMENT,D3D11_INPUT_PER_VERTEX_DATA,0}
    };

    DXCreateShader(&Renderer.Shader,
                   g_VertexShaderCode, ARRAY_SIZE(g_VertexShaderCode),
                   g_PixelShaderCode, ARRAY_SIZE(g_PixelShaderCode),
                   InputElementDesc, ARRAY_SIZE(InputElementDesc));

    D3D11_INPUT_ELEMENT_DESC DebugInputElementDesc[] = {
        {"POSITION",0,DXGI_FORMAT_R32G32B32_FLOAT,0,D3D11_APPEND_ALIGNED_ELEMENT,D3D11_INPUT_PER_VERTEX_DATA,0},
        {"COLOR",0,DXGI_FORMAT_R32G32B32_FLOAT,0,D3D11_APPEND_ALIGNED_ELEMENT,D3D11_INPUT_PER_VERTEX_DATA,0}
    };

    DXCreateShader(&Renderer.DebugShader,
                   g_DebugVertexShaderCode, ARRAY_SIZE(g_DebugVertexShaderCode),
                   g_DebugPixelShaderCode, ARRAY_SIZE(g_DebugPixelShaderCode),
                   DebugInputElementDesc, ARRAY_SIZE(DebugInputElementDesc));
}

static void DXCreateSphere(i32 Longitude, i32 Latitude, float Radius,
                           ID3D11Buffer **VertexBuffer, ID3D11Buffer **IndexBuffer, i32 *OutIndexCount)
{
    i32 VertexCount = (Longitude+1) * (Latitude+1);
    i32 IndexCount = Longitude * (Latitude-1) * 6;
    void *Memory = VirtualAlloc(0,
                                VertexCount * sizeof(vertex) + IndexCount * sizeof(u32),
                                MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE);

    vertex *Vertices = (vertex*)Memory;
    u32 *Indices = (u32*)(Vertices+VertexCount);

    i32 Idx = 0;
    for (i32 i = 0; i <= Longitude; ++i)
    {
        float LongitudeAngle = M_PI / 2.f - i * (M_PI / Longitude);
        for (i32 j = 0; j <= Latitude; ++j)
        {
            float LatitudeAngle = j * (2 * M_PI / Latitude);

            v3 Normal;
            Normal.x = cosf(LongitudeAngle) * cosf(LatitudeAngle);
            Normal.y = cosf(LongitudeAngle) * sinf(LatitudeAngle);
            Normal.z = sinf(LongitudeAngle);

            v3 Position = Radius * Normal;

            Vertices[Idx++] = {Position, Normal, V3(0.f, 0.5f, 0.7f)};
        }
    }

    Idx = 0;
    for (i32 i = 0; i < Longitude; ++i)
    {
        for (i32 j = 0; j < Latitude; ++j)
        {
            i32 k1 = i * (Latitude+1) + j;
            i32 k2 = k1 + (Latitude+1);
            if (i != 0)
            {
                Indices[Idx++] = k1;
                Indices[Idx++] = k2;
                Indices[Idx++] = k1+1;
            }

            if (i != Longitude-1)
            {
                Indices[Idx++] = k1+1;
                Indices[Idx++] = k2;
                Indices[Idx++] = k2+1;
            }
        }
    }

    D3D11_SUBRESOURCE_DATA InitData = {};
    D3D11_BUFFER_DESC VertexBufferDesc = {};
    VertexBufferDesc.Usage = D3D11_USAGE_DEFAULT;
    VertexBufferDesc.ByteWidth = VertexCount * sizeof(vertex);
    VertexBufferDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
    InitData.pSysMem = Vertices;
    Renderer.Device->CreateBuffer(&VertexBufferDesc, &InitData, VertexBuffer);
    ASSERT(*VertexBuffer);

    D3D11_BUFFER_DESC IndexBufferDesc = {};
    IndexBufferDesc.Usage = D3D11_USAGE_DEFAULT;
    IndexBufferDesc.ByteWidth = IndexCount * sizeof(u32);
    IndexBufferDesc.BindFlags = D3D11_BIND_INDEX_BUFFER;
    InitData.pSysMem = Indices;
    Renderer.Device->CreateBuffer(&IndexBufferDesc, &InitData, IndexBuffer);
    ASSERT(*IndexBuffer);

    *OutIndexCount = IndexCount;
    VirtualFree(Memory, 0, MEM_RELEASE);
}

static inline void DXLoadConstantBuffer(DXConstantBuffer *Buffer)
{
    D3D11_MAPPED_SUBRESOURCE Resource;
    DX_MAP(Renderer.Context, Renderer.ConstantBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &Resource)
    {
        memcpy(Resource.pData, Buffer, sizeof(DXConstantBuffer));
    }
}

void RendererGetBackbufferSize(i32 *Width, i32 *Height)
{
    D3D11_TEXTURE2D_DESC Desc;
    Renderer.Backbuffer->GetDesc(&Desc);
    *Width = Desc.Width;
    *Height = Desc.Height;
}

void RendererInitialize()
{
    Renderer.Window = GetActiveWindow();
    D3D_FEATURE_LEVEL FeatureLevels[] = {D3D_FEATURE_LEVEL_11_0};

    DXGI_SWAP_CHAIN_DESC SwapChainDesc = {};
    SwapChainDesc.BufferCount = 2;
    SwapChainDesc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    SwapChainDesc.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
    SwapChainDesc.OutputWindow = Renderer.Window;
    SwapChainDesc.Windowed = true;
    SwapChainDesc.SampleDesc.Count = 1;
    SwapChainDesc.SampleDesc.Quality = 0;
    SwapChainDesc.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;

    D3D11CreateDeviceAndSwapChain(NULL, D3D_DRIVER_TYPE_HARDWARE,
                                  NULL, D3D11_CREATE_DEVICE_DEBUG,
                                  FeatureLevels, ARRAY_SIZE(FeatureLevels),
                                  D3D11_SDK_VERSION,
                                  &SwapChainDesc,
                                  &Renderer.SwapChain,
                                  &Renderer.Device,
                                  NULL,
                                  &Renderer.Context);

    ASSERT(Renderer.SwapChain);
    ASSERT(Renderer.Device);
    ASSERT(Renderer.Context);

    ID3D11Texture2D *Backbuffer;
    Renderer.SwapChain->GetBuffer(0, IID_ID3D11Texture2D, (void**)&Backbuffer);
    D3D11_TEXTURE2D_DESC BackbufferDesc;
    Backbuffer->GetDesc(&BackbufferDesc);
    {
        Renderer.Device->CreateRenderTargetView(Backbuffer, 0, &Renderer.BackbufferView);

        D3D11_VIEWPORT Viewport = {};
        Viewport.Width = BackbufferDesc.Width;
        Viewport.Height = BackbufferDesc.Height;
        Viewport.MinDepth = 0.f;
        Viewport.MaxDepth = 1.f;
        Viewport.TopLeftX = 0;
        Viewport.TopLeftY = 0;
        Renderer.Context->RSSetViewports(1, &Viewport);

        ID3D11Texture2D *DepthStencil;
        D3D11_TEXTURE2D_DESC DepthStencilBufferDesc = {};
        DepthStencilBufferDesc.Width = BackbufferDesc.Width;
        DepthStencilBufferDesc.Height = BackbufferDesc.Height;
        DepthStencilBufferDesc.MipLevels = 1;
        DepthStencilBufferDesc.ArraySize = 1;
        DepthStencilBufferDesc.Format = DXGI_FORMAT_D32_FLOAT_S8X24_UINT;
        DepthStencilBufferDesc.SampleDesc.Count = 1;
        DepthStencilBufferDesc.SampleDesc.Quality = 0;
        DepthStencilBufferDesc.Usage = D3D11_USAGE_DEFAULT;
        DepthStencilBufferDesc.BindFlags = D3D11_BIND_DEPTH_STENCIL;
        DepthStencilBufferDesc.CPUAccessFlags = 0;
        DepthStencilBufferDesc.MiscFlags = 0;
        Renderer.Device->CreateTexture2D(&DepthStencilBufferDesc, NULL, &DepthStencil);
        
        D3D11_DEPTH_STENCIL_VIEW_DESC DepthStencilViewDesc;
        DepthStencilViewDesc.Format = DepthStencilBufferDesc.Format;
        DepthStencilViewDesc.ViewDimension = D3D11_DSV_DIMENSION_TEXTURE2D;
        DepthStencilViewDesc.Texture2D.MipSlice = 0;
        DepthStencilViewDesc.Flags = 0;
        Renderer.Device->CreateDepthStencilView(DepthStencil, &DepthStencilViewDesc, &Renderer.DepthStencilView);
        ASSERT(Renderer.DepthStencilView);
        DepthStencil->Release();
    }
    Renderer.Backbuffer = Backbuffer;

    D3D11_DEPTH_STENCIL_DESC DepthStencilDesc = {};
    DepthStencilDesc.DepthEnable = true;
    DepthStencilDesc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
    DepthStencilDesc.DepthFunc = D3D11_COMPARISON_LESS;
    Renderer.Device->CreateDepthStencilState(&DepthStencilDesc, &Renderer.DepthStencilState);
    ASSERT(Renderer.DepthStencilState);

    ID3D11RasterizerState *RasterState;
    D3D11_RASTERIZER_DESC RasterDesc = {};
    RasterDesc.FillMode = D3D11_FILL_SOLID;
    RasterDesc.CullMode = D3D11_CULL_NONE;
    RasterDesc.FrontCounterClockwise = true;
    RasterDesc.DepthClipEnable = true;
    Renderer.Device->CreateRasterizerState(&RasterDesc, &RasterState);
    Renderer.Context->RSSetState(RasterState);

    DXInitShaders();

    // @TODO: Just have one scratch vertex buffer.
    {
        D3D11_BUFFER_DESC VertexBufferDesc = {};
        VertexBufferDesc.Usage = D3D11_USAGE_DYNAMIC;
        VertexBufferDesc.ByteWidth = 36 * sizeof(vertex);
        VertexBufferDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
        VertexBufferDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
        Renderer.Device->CreateBuffer(&VertexBufferDesc, NULL, &Renderer.BoxVertexBuffer);
        ASSERT(Renderer.BoxVertexBuffer);

        VertexBufferDesc.ByteWidth = 8192 * sizeof(vertex);
        Renderer.Device->CreateBuffer(&VertexBufferDesc, NULL, &Renderer.HullVertexBuffer);
        ASSERT(Renderer.HullVertexBuffer);

        VertexBufferDesc.ByteWidth = 16384 * sizeof(debug_vertex);
        Renderer.Device->CreateBuffer(&VertexBufferDesc, NULL, &Renderer.ScratchVertexBuffer);
        ASSERT(Renderer.ScratchVertexBuffer);
    }

    {
        D3D11_BUFFER_DESC ConstantBufferDesc = {};
        ConstantBufferDesc.ByteWidth = sizeof(DXConstantBuffer);
        ConstantBufferDesc.Usage = D3D11_USAGE_DYNAMIC;
        ConstantBufferDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
        ConstantBufferDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
        Renderer.Device->CreateBuffer(&ConstantBufferDesc, NULL, &Renderer.ConstantBuffer);
        Renderer.Context->VSSetConstantBuffers(0, 1, &Renderer.ConstantBuffer);
        Renderer.Context->PSSetConstantBuffers(0, 1, &Renderer.ConstantBuffer);
    }

    DXCreateSphere(16, 32, 1,
                   &Renderer.SphereVertexBuffer,
                   &Renderer.SphereIndexBuffer,
                   &Renderer.SphereIndexCount);

    ImGui_ImplDX11_Init(Renderer.Device, Renderer.Context);
}

void RendererDrawFrame(render_group *RenderGroup)
{
    float ClearColor[] = {0.3f, 0.3f, 0.3f, 1.0f};
    Renderer.Context->OMSetRenderTargets(1, &Renderer.BackbufferView, Renderer.DepthStencilView);
    Renderer.Context->ClearDepthStencilView(Renderer.DepthStencilView,
                                            D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL,
                                            1.f, 0.f);
    Renderer.Context->ClearRenderTargetView(Renderer.BackbufferView, ClearColor);

    float AspectRatio = 1.f;
    {
        RECT ClientRect;
        GetClientRect(Renderer.Window, &ClientRect);
        float Width = (float)(ClientRect.right - ClientRect.left);
        float Height = (float)(ClientRect.bottom - ClientRect.top);
        AspectRatio = Width / Height;
    }

    m4x4 ViewProjection =
            PerspectiveProjection(60.f, AspectRatio, 0.01f, 10000.f) *
            CalculateCameraViewMatrix(RenderGroup->Camera);

    Renderer.Context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    Renderer.Context->OMSetDepthStencilState(Renderer.DepthStencilState, 1);
    DXUseShader(Renderer.Shader);

    DXConstantBuffer ConstantBuffer;
    ConstantBuffer.CameraDirection = RenderGroup->Camera->Direction;
    D3D11_MAPPED_SUBRESOURCE Subresource;
    u32 Stride = sizeof(vertex);
    u32 Offset = 0;
    for (i32 CmdIndex = 0;
         CmdIndex < RenderGroup->CmdsCount;
         ++CmdIndex)
    {
        render_command *Cmd = RenderGroup->Cmds + CmdIndex;
        switch (Cmd->Type)
        {
            case RenderCommandType_Sphere:
            {

                ConstantBuffer.ViewProjection =
                    ViewProjection *
                    Translation(Cmd->Sphere.Center) *
                    UniformScale(Cmd->Sphere.Radius);
                DXLoadConstantBuffer(&ConstantBuffer);

                Renderer.Context->IASetVertexBuffers(0, 1, &Renderer.SphereVertexBuffer, &Stride, &Offset);
                Renderer.Context->IASetIndexBuffer(Renderer.SphereIndexBuffer, DXGI_FORMAT_R32_UINT, 0);
                Renderer.Context->DrawIndexed(Renderer.SphereIndexCount, 0, 0);
            } break;

            case RenderCommandType_AABB:
            {
                v3 Max = Cmd->AABB.AABB.Max;
                v3 Min = Cmd->AABB.AABB.Min;
                v3 Color = Cmd->AABB.Color;
                DX_MAP(Renderer.Context,Renderer.BoxVertexBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &Subresource)
                {
                    vertex *Buffer = (vertex*)Subresource.pData;
                    v3 Corners[] = {
                        Min,
                        V3(Max.x, Min.y, Min.z),
                        V3(Max.x, Max.y, Min.z),
                        V3(Min.x, Max.y, Min.z),
                        Max,
                        V3(Min.x, Max.y, Max.z),
                        V3(Min.x, Min.y, Max.z),
                        V3(Max.x, Min.y, Max.z)
                    };
                    i32 Indices[] = {
                        0, 1, 2,  0, 2, 3,
                        6, 5, 4,  6, 4, 7,
                        1, 7, 4,  1, 4, 2,
                        0, 3, 5,  0, 5, 6,
                        0, 6, 7,  0, 7, 1,
                        2, 4, 5,  2, 5, 3
                    };
                    v3 Normals[] = {
                        V3( 0,  0, -1),
                        V3( 0,  0,  1),
                        V3( 1,  0,  0),
                        V3(-1,  0,  0),
                        V3( 0, -1,  0),
                        V3( 0,  1,  0)
                    };
                    for (i32 i = 0; i < ARRAY_SIZE(Indices); ++i)
                    {
                        i32 n = i / 6;
                        Buffer[i] = {Corners[Indices[i]], Normals[n], Color};
                    }
                }

                ConstantBuffer.Model =
                    Translation(Cmd->AABB.Transform.Position) *
                    RotationMatrix(Cmd->AABB.Transform.Rotation);
                ConstantBuffer.ViewProjection = ViewProjection;
                DXLoadConstantBuffer(&ConstantBuffer);

                Renderer.Context->IASetVertexBuffers(0, 1, &Renderer.BoxVertexBuffer, &Stride, &Offset);
                Renderer.Context->Draw(36, 0);
            } break;

            case DEBUG_RenderCommandType_BVH:
            {
                DXUseShader(Renderer.DebugShader);
                bvh_tree *Tree = Cmd->DEBUGBVHTree;
                Renderer.Context->Map(Renderer.ScratchVertexBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &Subresource);
                debug_vertex *Buffer = (debug_vertex*)Subresource.pData;
                i32 VertexCount = 0;
                for (i32 NodeIndex = 0;
                     NodeIndex < Tree->NodeCount;
                     ++NodeIndex)
                {
                    v3 Color = V3(1,0,0);
                    if (Tree->Nodes[NodeIndex].IsLeaf)
                    {
                        Color = V3(1,1,1);
                    }
                    if (Tree->Root == NodeIndex)
                    {
                        Color = V3(1,1,0);
                    }
                    v3 Min = Tree->Nodes[NodeIndex].BoundingVolume.Min;
                    v3 Max = Tree->Nodes[NodeIndex].BoundingVolume.Max;
                    {
                        v3 Corners[] = {
                            Min,
                            V3(Max.x, Min.y, Min.z),
                            V3(Max.x, Max.y, Min.z),
                            V3(Min.x, Max.y, Min.z),
                            Max,
                            V3(Min.x, Max.y, Max.z),
                            V3(Min.x, Min.y, Max.z),
                            V3(Max.x, Min.y, Max.z)
                        };
                        i32 Indices[] = {
                            0, 1,  0, 3,  0, 6,
                            4, 5,  4, 7,  4, 2,
                            2, 1,  2, 3,  6, 5,
                            6, 7,  3, 5,  1, 7
                        };
                        VertexCount += ARRAY_SIZE(Indices);
                        for (i32 i = 0; i < ARRAY_SIZE(Indices); ++i)
                        {
                            *Buffer++ = {Corners[Indices[i]], Color};
                        }
                    }
                }
                Renderer.Context->Unmap(Renderer.ScratchVertexBuffer, 0);
                ConstantBuffer.Model = IdentityMatrix();
                ConstantBuffer.ViewProjection = ViewProjection;
                DXLoadConstantBuffer(&ConstantBuffer);


                u32 DebugStride = sizeof(debug_vertex);
                Renderer.Context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
                Renderer.Context->IASetVertexBuffers(0, 1, &Renderer.ScratchVertexBuffer, &DebugStride, &Offset);
                Renderer.Context->Draw(VertexCount, 0);
                Renderer.Context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
                DXUseShader(Renderer.Shader);
            } break;

            case DEBUG_RenderCommandType_Hull:
            {
                Renderer.Context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);

                ConstantBuffer.ViewProjection = ViewProjection;
                DXLoadConstantBuffer(&ConstantBuffer);

                i32 BufIndex = 0;
                DX_MAP(Renderer.Context, Renderer.HullVertexBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &Subresource)
                {
                    vertex *Buffer = (vertex*)Subresource.pData;
                    hull *Hull = Cmd->DEBUGHull;

                    for (i32 i = 0; i < Hull->FaceCount; ++i)
                    {
                        i32 StartEdge = Hull->Faces[i].Edge;
                        i32 CurrentEdge = -1;
                        half_edge *Edge = Hull->Edges + StartEdge;
                        v3 NormalStart = V3(0,0,0);
                        i32 FaceVertexCount = 0;
                        while (CurrentEdge != StartEdge)
                        {
                            FaceVertexCount++;
                            v3 A = Hull->Vertices[Edge->Origin];
                            NormalStart.x += A.x;
                            NormalStart.y += A.y;
                            NormalStart.z += A.z;
                            CurrentEdge = Edge->Next;
                            Edge = Hull->Edges + Edge->Next;
                            v3 B = Hull->Vertices[Edge->Origin];
                            Buffer[BufIndex++] = {A, V3(0,0,1), V3(1,1,1)};
                            Buffer[BufIndex++] = {B, V3(0,0,1), V3(1,1,1)};
                        }
                        for (i32 j = 0; j < 3; ++j)
                        {
                            NormalStart[j] = NormalStart[j] / (float)FaceVertexCount;
                        }
                        v3 NormalEnd = NormalStart + Hull->Planes[i].Normal * 16.f;
                        Buffer[BufIndex++] = {NormalStart, V3(0,0,1), V3(1,1,1)};
                        Buffer[BufIndex++] = {NormalEnd, V3(0,0,1), V3(1,1,1)};
                    }
                }

                Renderer.Context->IASetVertexBuffers(0, 1, &Renderer.HullVertexBuffer, &Stride, &Offset);
                Renderer.Context->Draw(BufIndex, 0);
                Renderer.Context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
            } break;

            default: ASSERT(false);
        }
    }

    ImGui::Render();
    ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
    Renderer.SwapChain->Present(1, 0);
}

