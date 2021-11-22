#ifndef MATHLIB_H
#define MATHLIB_H

#include "math.h"
#include "float.h"

// @TODO: Rename.
#define M_PI   3.14159265358979323846
#define M_PI_2 1.57079632679489661923

#define ML_EPSILON FLT_EPSILON

union v3
{
    struct
    {
        float x, y, z;
    };
    float e[3];

    inline float operator[](int i) const
    {
        return e[i];
    }

    inline float& operator[](int i)
    {
        return e[i];
    }

    inline v3& operator+=(v3 o);
    inline v3& operator-=(v3 o);
    inline v3& operator*=(float s);
};

struct quaternion
{
    float w;
    union
    {
        v3 v;
        struct { float x, y, z; };
    };

    inline quaternion& operator+=(quaternion o);
};

struct m4x4
{
    float e[4][4];

    float *operator[](int i)
    {
        return e[i];
    }
};

struct m3x3
{
    float e[3][3];

    float *operator[](int i)
    {
        return e[i];
    }
};

// Some of the operators need to use these two functions.
inline v3 Cross(v3 a, v3 b);
inline float Dot(v3 a, v3 b);

#include "math_operators.inl"

inline float DegreesToRadians(float value)
{
    return value * (M_PI / 180.f);
}

inline float Square(float a) 
{
    return a*a;
}

inline float Max(float a, float b)
{
    return a > b ? a : b;
}

inline float Min(float a, float b)
{
    return a < b ? a : b;
}

inline float Clamp(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

inline float Dot(v3 a, v3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline float LengthSquared(v3 v)
{
    return v.x*v.x + v.y*v.y + v.z*v.z;
}

inline float Length(v3 v)
{
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

bool IsZeroVector(v3 v)
{
    return Dot(v, v) <= (ML_EPSILON * ML_EPSILON);
}

inline v3 Normalized(v3 v)
{
    v3 result;
    float len = Length(v);
    if (len == 0.f)
    {
        return V3(0,0,0);
    }
    float inv_len = 1.f / Length(v);
    result.x = v.x * inv_len;
    result.y = v.y * inv_len;
    result.z = v.z * inv_len;
    return result;
}

inline v3 Cross(v3 a, v3 b)
{
    v3 result;
    result.x = a.y * b.z - a.z * b.y;
    result.y = a.z * b.x - a.x * b.z;
    result.z = a.x * b.y - a.y * b.x;
    return result;
}

inline float Length(quaternion q)
{
    return sqrtf(q.w * q.w +
                 q.x * q.x +
                 q.y * q.y +
                 q.z * q.z);
}

inline quaternion Conjugate(quaternion q)
{
    q.x = -q.x;
    q.y = -q.y;
    q.z = -q.z;
    return q;
}

inline quaternion Inverse(quaternion q)
{
    float len = Length(q);
    return Conjugate(q) * (1.f / (len * len));
}

inline quaternion Normalized(quaternion q)
{
    quaternion result;
    float inv_len = 1.f / Length(q);
    result.w = q.w * inv_len;
    result.x = q.x * inv_len;
    result.y = q.y * inv_len;
    result.z = q.z * inv_len;
    return result;
}

inline quaternion Rotation(v3 axis, float theta)
{
    quaternion q;
    float theta_half = theta / 2.f;
    q.w = cosf(theta_half);
    q.v = axis * sinf(theta_half);
    return q;
}

inline v3 RotateVector(v3 vector, v3 axis, float theta)
{
    quaternion v;
    v.w = 0;
    v.v = vector;

    quaternion p = Rotation(axis, theta);
    quaternion q = Conjugate(p);

    quaternion result = p*v*q;
    return result.v;
}

inline v3 RotateVector(v3 vector, quaternion q)
{
    quaternion v;
    v.w = 0;
    v.v = vector;
    quaternion conj = Conjugate(q);
    quaternion result = q*v*conj;
    return result.v;
}

inline m3x3 Transpose(m3x3 m)
{
    m3x3 result;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            result[i][j] = m[j][i];
        }
    }
    return result;
}

inline m4x4 IdentityMatrix()
{
    m4x4 result = {};
    result[0][0] = 1.f;
    result[1][1] = 1.f;
    result[2][2] = 1.f;
    result[3][3] = 1.f;
    return result;
}

inline m4x4 Transpose(m4x4 m)
{
    m4x4 result;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            result[i][j] = m[j][i];
        }
    }
    return result;
}

inline m4x4 PerspectiveProjection(float fov, float ar, float n, float f)
{
    float distance = 1.f / tanf(DegreesToRadians(fov) / 2.f);
    m4x4 result = {};
    result[0][0] = distance / ar;
    result[1][1] = distance;
    result[2][2] = -(f / (f-n));
    result[2][3] = -((f*n) / (f-n));
    result[3][2] = -1.f;
    return result;
}

inline m3x3 RotationMatrix3(quaternion q)
{
    m3x3 result = {};
    result[0][0] = 1 - 2*q.y*q.y - 2*q.z*q.z;
    result[0][1] = 2*q.x*q.y - 2*q.w*q.z;
    result[0][2] = 2*q.z*q.x + 2*q.w*q.y;

    result[1][0] = 2*q.x*q.y + 2*q.w*q.z;
    result[1][1] = 1 - 2*q.x*q.x - 2*q.z*q.z;
    result[1][2] = 2*q.y*q.z - 2*q.w*q.x;
    
    result[2][0] = 2*q.z*q.x - 2*q.w*q.y;
    result[2][1] = 2*q.y*q.z + 2*q.w*q.x;
    result[2][2] = 1 - 2*q.x*q.x - 2*q.y*q.y;
    return result;
}

inline m4x4 RotationMatrix(quaternion q)
{
    m4x4 result = {};
    result[0][0] = 1 - 2*q.y*q.y - 2*q.z*q.z;
    result[0][1] = 2*q.x*q.y - 2*q.w*q.z;
    result[0][2] = 2*q.z*q.x + 2*q.w*q.y;

    result[1][0] = 2*q.x*q.y + 2*q.w*q.z;
    result[1][1] = 1 - 2*q.x*q.x - 2*q.z*q.z;
    result[1][2] = 2*q.y*q.z - 2*q.w*q.x;
    
    result[2][0] = 2*q.z*q.x - 2*q.w*q.y;
    result[2][1] = 2*q.y*q.z + 2*q.w*q.x;
    result[2][2] = 1 - 2*q.x*q.x - 2*q.y*q.y;
    
    result[3][3] = 1.f;
    return result;
}

inline m4x4 Translation(float x, float y, float z)
{
    m4x4 result = IdentityMatrix();
    result[0][3] = x;
    result[1][3] = y;
    result[2][3] = z;
    return result;
}

inline m4x4 Translation(v3 v)
{
    return Translation(v.x, v.y, v.z);
}

inline m4x4 Scale(float x, float y, float z)
{
    m4x4 result = {};
    result[0][0] = x;
    result[1][1] = y;
    result[2][2] = z;
    result[3][3] = 1;
    return result;
}

inline m4x4 UniformScale(float s)
{
    return Scale(s, s, s);
}

inline m4x4 CameraMatrix(v3 pos, v3 x, v3 y, v3 z)
{
    m4x4 result = {};
    for (int i = 0; i < 3; i++)
    {
        result.e[0][i] = x.e[i];
        result.e[1][i] = y.e[i];
        result.e[2][i] = z.e[i];
    }

    result.e[0][3] = Dot(x, -pos);
    result.e[1][3] = Dot(y, -pos);
    result.e[2][3] = Dot(z, -pos);
    result.e[3][3] = 1.f;

    return result;
}

inline m4x4 LookAt(v3 from, v3 at, v3 up)
{
    v3 z = Normalized(from - at);
    v3 x = Normalized(Cross(up, z));
    v3 y = Normalized(Cross(z, x));
    return CameraMatrix(from, x,y,z);
}

#endif

