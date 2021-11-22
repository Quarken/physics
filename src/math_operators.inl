inline v3 Cross(v3 a, v3 b);

inline v3 V3(float x, float y, float z)
{
    v3 result;
    result.x = x;
    result.y = y;
    result.z = z;
    return result;
}

inline v3 operator+(v3 a, v3 b)
{
    v3 result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;
    return result;
}

inline v3 operator-(v3 v)
{
    v3 result;
    result.x = -v.x;
    result.y = -v.y;
    result.z = -v.z;
    return result;
}

inline v3 operator-(v3 a, v3 b)
{
    v3 result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;
    return result;
}

inline v3 operator*(float s, v3 v)
{
    v3 result;
    result.x = s * v.x;
    result.y = s * v.y;
    result.z = s * v.z;
    return result;
}

inline v3 operator*(v3 v, float s)
{
    v3 result;
    result.x = s * v.x;
    result.y = s * v.y;
    result.z = s * v.z;
    return result;
}

inline v3 operator/(v3 v, float s)
{
    v3 result;
    float s_inv = 1.f / s;
    result.x = v.x * s_inv;
    result.y = v.y * s_inv;
    result.z = v.z * s_inv;
    return result;
}

inline v3 operator/(float s, v3 v)
{
    v3 result;
    float s_inv = 1.f / s;
    result.x = v.x * s_inv;
    result.y = v.y * s_inv;
    result.z = v.z * s_inv;
    return result;
}

inline v3& v3::operator+=(v3 o)
{
    *this = *this + o;
    return *this;
}

inline v3& v3::operator-=(v3 o)
{
    *this = *this - o;
    return *this;
}

inline v3& v3::operator*=(float s)
{
    *this = *this * s;
    return *this;
}

inline quaternion operator+(quaternion a, quaternion b)
{
    quaternion q;
    q.w = a.w  + b.w;
    q.v = a.v + b.v;
    return q;
}

inline quaternion operator-(quaternion a, quaternion b)
{
    quaternion q;
    q.w = a.w - b.w;
    q.v = a.v - b.v;
    return q;
}

inline quaternion operator*(quaternion a, quaternion b)
{
    quaternion q;
    q.w = a.w * b.w - Dot(a.v, b.v);
    q.v = a.w * b.v + b.w * a.v + Cross(a.v, b.v);
    return q;
}

inline quaternion operator*(float s, quaternion q)
{
    quaternion result;
    result.w = s * q.w;
    result.x = s * q.x;
    result.y = s * q.y;
    result.z = s * q.z;
    return result;
}

inline quaternion operator*(quaternion q, float s)
{
    quaternion result;
    result.w = s * q.w;
    result.x = s * q.x;
    result.y = s * q.y;
    result.z = s * q.z;
    return result;
}

inline quaternion& quaternion::operator+=(quaternion o)
{
    *this = *this + o;
    return *this;
}

inline m3x3 operator*(m3x3 a, m3x3 b)
{
    m3x3 result;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            float sum = 0;
            for (int k = 0; k < 3; ++k)
            {
                sum += a[i][k] * b[k][j];
            }
            result.e[i][j] = sum;
        }
    }
    return result;
}

inline v3 operator*(m3x3 a, v3 v)
{
    v3 result;
    result.x = a[0][0] * v.x + a[0][1] * v.y + a[0][2] * v.z;
    result.y = a[1][0] * v.x + a[1][1] * v.y + a[1][2] * v.z;
    result.z = a[2][0] * v.x + a[2][1] * v.y + a[2][2] * v.z;
    return result;
}

inline m4x4 operator*(m4x4 a, m4x4 b)
{
    m4x4 result;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            float sum = 0;
            for (int k = 0; k < 4; ++k)
            {
                sum += a[i][k] * b[k][j];
            }
            result.e[i][j] = sum;
        }
    }
    return result;
}

