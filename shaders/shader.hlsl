cbuffer ConstantBuffer : register(b0)
{
    float4x4 Model;
    float4x4 ViewProjection;
    float3 CameraDirection;
};

struct DEBUGVertexShaderInput
{
    float3 Position : POSITION;
    float3 Color : COLOR;
};

struct DEBUGPixelShaderInput
{
    float4 Position : SV_POSITION;
    float3 Color : COLOR;
};

DEBUGPixelShaderInput DEBUGVertexShaderMain(DEBUGVertexShaderInput Input)
{
    DEBUGPixelShaderInput Result;
    Result.Position = mul(float4(Input.Position, 1.0f), mul(Model, ViewProjection));
    Result.Color = Input.Color;
    return Result;
}

float4 DEBUGPixelShaderMain(DEBUGPixelShaderInput Input) : SV_TARGET
{
    return float4(Input.Color, 1.0f);
};

struct VertexShaderInput
{
    float3 Position : POSITION;
    float3 Normal : NORMAL;
    float3 Color : COLOR;
};

struct PixelShaderInput
{
    float4 Position : SV_POSITION;
    float3 Normal : NORMAL;
    float3 Color : COLOR;
};

PixelShaderInput VertexShaderMain(VertexShaderInput Input)
{
    PixelShaderInput Result;
    float4 Position = float4(Input.Position, 1.0f);
    Result.Position = mul(Position, mul(Model, ViewProjection));
    float3x3 NormalMatrix = (float3x3)Model;
    Result.Normal = mul(Input.Normal, NormalMatrix);
    Result.Color = Input.Color;
    return Result;
}

float4 PixelShaderMain(PixelShaderInput Input) : SV_TARGET
{
    float LightStrength = max(dot(-CameraDirection, Input.Normal), 0);
    LightStrength += 0.2f;
    float3 add = abs(Input.Normal * 3.f);
    return float4(Input.Color * LightStrength, 1.0f);
    // return float4((Input.Normal + add) * 0.5f, 1.0f);
}

