#if 0
//
// Generated by Microsoft (R) HLSL Shader Compiler 10.1
//
//
// Buffer Definitions: 
//
// cbuffer ConstantBuffer
// {
//
//   float4x4 Model;                    // Offset:    0 Size:    64 [unused]
//   float4x4 ViewProjection;           // Offset:   64 Size:    64 [unused]
//   float3 CameraDirection;            // Offset:  128 Size:    12
//
// }
//
//
// Resource Bindings:
//
// Name                                 Type  Format         Dim      HLSL Bind  Count
// ------------------------------ ---------- ------- ----------- -------------- ------
// ConstantBuffer                    cbuffer      NA          NA            cb0      1 
//
//
//
// Input signature:
//
// Name                 Index   Mask Register SysValue  Format   Used
// -------------------- ----- ------ -------- -------- ------- ------
// SV_POSITION              0   xyzw        0      POS   float       
// NORMAL                   0   xyz         1     NONE   float   xyz 
// COLOR                    0   xyz         2     NONE   float   xyz 
//
//
// Output signature:
//
// Name                 Index   Mask Register SysValue  Format   Used
// -------------------- ----- ------ -------- -------- ------- ------
// SV_TARGET                0   xyzw        0   TARGET   float   xyzw
//
ps_4_0
dcl_constantbuffer CB0[9], immediateIndexed
dcl_input_ps linear v1.xyz
dcl_input_ps linear v2.xyz
dcl_output o0.xyzw
dcl_temps 1
dp3 r0.x, -cb0[8].xyzx, v1.xyzx
max r0.x, r0.x, l(0.000000)
add r0.x, r0.x, l(0.200000)
mul o0.xyz, r0.xxxx, v2.xyzx
mov o0.w, l(1.000000)
ret 
// Approximately 6 instruction slots used
#endif

const BYTE g_PixelShaderCode[] =
{
     68,  88,  66,  67,  22, 110, 
     26,  49, 102,   7, 182,  53, 
    199,  81, 141, 255, 181,  71, 
    122, 148,   1,   0,   0,   0, 
     88,   3,   0,   0,   5,   0, 
      0,   0,  52,   0,   0,   0, 
     88,   1,   0,   0, 204,   1, 
      0,   0,   0,   2,   0,   0, 
    220,   2,   0,   0,  82,  68, 
     69,  70,  28,   1,   0,   0, 
      1,   0,   0,   0,  76,   0, 
      0,   0,   1,   0,   0,   0, 
     28,   0,   0,   0,   0,   4, 
    255, 255,   0,   1,   0,   0, 
    244,   0,   0,   0,  60,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   1,   0, 
      0,   0,   1,   0,   0,   0, 
     67, 111, 110, 115, 116,  97, 
    110, 116,  66, 117, 102, 102, 
    101, 114,   0, 171,  60,   0, 
      0,   0,   3,   0,   0,   0, 
    100,   0,   0,   0, 144,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0, 172,   0, 
      0,   0,   0,   0,   0,   0, 
     64,   0,   0,   0,   0,   0, 
      0,   0, 180,   0,   0,   0, 
      0,   0,   0,   0, 196,   0, 
      0,   0,  64,   0,   0,   0, 
     64,   0,   0,   0,   0,   0, 
      0,   0, 180,   0,   0,   0, 
      0,   0,   0,   0, 211,   0, 
      0,   0, 128,   0,   0,   0, 
     12,   0,   0,   0,   2,   0, 
      0,   0, 228,   0,   0,   0, 
      0,   0,   0,   0,  77, 111, 
    100, 101, 108,   0, 171, 171, 
      3,   0,   3,   0,   4,   0, 
      4,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,  86, 105, 
    101, 119,  80, 114, 111, 106, 
    101,  99, 116, 105, 111, 110, 
      0,  67,  97, 109, 101, 114, 
     97,  68, 105, 114, 101,  99, 
    116, 105, 111, 110,   0, 171, 
      1,   0,   3,   0,   1,   0, 
      3,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,  77, 105, 
     99, 114, 111, 115, 111, 102, 
    116,  32,  40,  82,  41,  32, 
     72,  76,  83,  76,  32,  83, 
    104,  97, 100, 101, 114,  32, 
     67, 111, 109, 112, 105, 108, 
    101, 114,  32,  49,  48,  46, 
     49,   0,  73,  83,  71,  78, 
    108,   0,   0,   0,   3,   0, 
      0,   0,   8,   0,   0,   0, 
     80,   0,   0,   0,   0,   0, 
      0,   0,   1,   0,   0,   0, 
      3,   0,   0,   0,   0,   0, 
      0,   0,  15,   0,   0,   0, 
     92,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      3,   0,   0,   0,   1,   0, 
      0,   0,   7,   7,   0,   0, 
     99,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      3,   0,   0,   0,   2,   0, 
      0,   0,   7,   7,   0,   0, 
     83,  86,  95,  80,  79,  83, 
     73,  84,  73,  79,  78,   0, 
     78,  79,  82,  77,  65,  76, 
      0,  67,  79,  76,  79,  82, 
      0, 171, 171, 171,  79,  83, 
     71,  78,  44,   0,   0,   0, 
      1,   0,   0,   0,   8,   0, 
      0,   0,  32,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   3,   0,   0,   0, 
      0,   0,   0,   0,  15,   0, 
      0,   0,  83,  86,  95,  84, 
     65,  82,  71,  69,  84,   0, 
    171, 171,  83,  72,  68,  82, 
    212,   0,   0,   0,  64,   0, 
      0,   0,  53,   0,   0,   0, 
     89,   0,   0,   4,  70, 142, 
     32,   0,   0,   0,   0,   0, 
      9,   0,   0,   0,  98,  16, 
      0,   3, 114,  16,  16,   0, 
      1,   0,   0,   0,  98,  16, 
      0,   3, 114,  16,  16,   0, 
      2,   0,   0,   0, 101,   0, 
      0,   3, 242,  32,  16,   0, 
      0,   0,   0,   0, 104,   0, 
      0,   2,   1,   0,   0,   0, 
     16,   0,   0,   9,  18,   0, 
     16,   0,   0,   0,   0,   0, 
     70, 130,  32, 128,  65,   0, 
      0,   0,   0,   0,   0,   0, 
      8,   0,   0,   0,  70,  18, 
     16,   0,   1,   0,   0,   0, 
     52,   0,   0,   7,  18,   0, 
     16,   0,   0,   0,   0,   0, 
     10,   0,  16,   0,   0,   0, 
      0,   0,   1,  64,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   7,  18,   0,  16,   0, 
      0,   0,   0,   0,  10,   0, 
     16,   0,   0,   0,   0,   0, 
      1,  64,   0,   0, 205, 204, 
     76,  62,  56,   0,   0,   7, 
    114,  32,  16,   0,   0,   0, 
      0,   0,   6,   0,  16,   0, 
      0,   0,   0,   0,  70,  18, 
     16,   0,   2,   0,   0,   0, 
     54,   0,   0,   5, 130,  32, 
     16,   0,   0,   0,   0,   0, 
      1,  64,   0,   0,   0,   0, 
    128,  63,  62,   0,   0,   1, 
     83,  84,  65,  84, 116,   0, 
      0,   0,   6,   0,   0,   0, 
      1,   0,   0,   0,   0,   0, 
      0,   0,   3,   0,   0,   0, 
      4,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      1,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      1,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0
};