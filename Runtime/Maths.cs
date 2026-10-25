using Unity.Mathematics;

namespace Avbd
{
    // The demo's maths.h, operation for operation, so the port rounds like the reference does.
    // Matrices hold the same entries as the demo's row-major ones: demo m[row][col] is m[col][row] here.
    internal static class Maths
    {
        public const float PenaltyMin = 1f;
        public const float PenaltyMax = 1e10f;
        public const float CollisionMargin = 0.01f;
        public const float StickThreshold = 0.00001f;

        public static quaternion Mul(quaternion qa, quaternion qb)
        {
            float4 a = qa.value, b = qb.value;
            return new quaternion(
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);
        }

        public static quaternion Conjugate(quaternion q) => new(-q.value.x, -q.value.y, -q.value.z, q.value.w);

        public static float LengthSq(quaternion q) => math.dot(q.value.xyz, q.value.xyz) + q.value.w * q.value.w;

        public static quaternion Inverse(quaternion q) => new(Conjugate(q).value / LengthSq(q));

        public static quaternion Normalize(quaternion q) => new(q.value / math.sqrt(LengthSq(q)));

        // Demo: quat - quat — the rotation from b to a as a small-angle vector.
        public static float3 Difference(quaternion a, quaternion b) => Mul(a, Inverse(b)).value.xyz * 2f;

        // Demo: quat + float3 — integrates an angular step.
        public static quaternion Add(quaternion a, float3 b)
        {
            var spin = Mul(new quaternion(b.x, b.y, b.z, 0f), a).value * 0.5f;
            return Normalize(new quaternion(a.value + spin));
        }

        public static float3 Rotate(quaternion q, float3 v)
        {
            var u = q.value.xyz;
            var t = math.cross(u, v) * 2f;
            return v + t * q.value.w + math.cross(u, t);
        }

        public static float3 Transform(float3 position, quaternion rotation, float3 v) => Rotate(rotation, v) + position;

        public static float3x3 Rows(float3 r0, float3 r1, float3 r2) => math.transpose(new float3x3(r0, r1, r2));

        public static float3x3 Diagonal(float3 d) => new(d.x, 0f, 0f, 0f, d.y, 0f, 0f, 0f, d.z);

        public static float3x3 Diagonal(float d) => Diagonal(new float3(d));

        public static float3x3 Skew(float3 r) => new(0f, -r.z, r.y, r.z, 0f, -r.x, -r.y, r.x, 0f);

        public static float3x3 Outer(float3 a, float3 b) => Rows(b * a.x, b * a.y, b * a.z);

        public static float3x3 Diagonalize(float3x3 m) => Diagonal(new float3(math.length(m.c0), math.length(m.c1), math.length(m.c2)));

        public static float3x3 Orthonormal(float3 normal)
        {
            var t1 = math.abs(normal.x) > math.abs(normal.z) ? new float3(-normal.y, normal.x, 0f) : new float3(0f, -normal.z, normal.y);
            t1 /= math.length(t1);
            return Rows(normal, t1, math.cross(normal, t1));
        }

        // Second-order term of a ball-socket row k: −v[k]·I with v added to column k.
        public static float3x3 BallSocketStiffness(int k, float3 v)
        {
            var m = Diagonal(-v[k]);
            m[k] += v;
            return m;
        }

        public static float3 Min(float3 a, float b) => new(math.min(a.x, b), math.min(a.y, b), math.min(a.z, b));

        public static float3 Clamp(float3 v, float a, float b) => new(Clamp(v.x, a, b), Clamp(v.y, a, b), Clamp(v.z, a, b));

        public static float Clamp(float x, float a, float b) => math.max(a, math.min(b, x));

        // The demo's 6×6 LDLᵀ over the lower triangle of [lin, crossᵀ; cross, ang].
        public static void Solve(float3x3 aLin, float3x3 aAng, float3x3 aCross, float3 bLin, float3 bAng, out float3 xLin, out float3 xAng)
        {
            var a11 = aLin.c0.x;
            float a21 = aLin.c0.y, a22 = aLin.c1.y;
            float a31 = aLin.c0.z, a32 = aLin.c1.z, a33 = aLin.c2.z;
            float a41 = aCross.c0.x, a42 = aCross.c1.x, a43 = aCross.c2.x, a44 = aAng.c0.x;
            float a51 = aCross.c0.y, a52 = aCross.c1.y, a53 = aCross.c2.y, a54 = aAng.c0.y, a55 = aAng.c1.y;
            float a61 = aCross.c0.z, a62 = aCross.c1.z, a63 = aCross.c2.z, a64 = aAng.c0.z, a65 = aAng.c1.z, a66 = aAng.c2.z;

            var l21 = a21 / a11;
            var l31 = a31 / a11;
            var l41 = a41 / a11;
            var l51 = a51 / a11;
            var l61 = a61 / a11;
            var d1 = a11;
            var d2 = a22 - l21 * l21 * d1;
            var l32 = (a32 - l21 * l31 * d1) / d2;
            var l42 = (a42 - l21 * l41 * d1) / d2;
            var l52 = (a52 - l21 * l51 * d1) / d2;
            var l62 = (a62 - l21 * l61 * d1) / d2;
            var d3 = a33 - (l31 * l31 * d1 + l32 * l32 * d2);
            var l43 = (a43 - l31 * l41 * d1 - l32 * l42 * d2) / d3;
            var l53 = (a53 - l31 * l51 * d1 - l32 * l52 * d2) / d3;
            var l63 = (a63 - l31 * l61 * d1 - l32 * l62 * d2) / d3;
            var d4 = a44 - (l41 * l41 * d1 + l42 * l42 * d2 + l43 * l43 * d3);
            var l54 = (a54 - l41 * l51 * d1 - l42 * l52 * d2 - l43 * l53 * d3) / d4;
            var l64 = (a64 - l41 * l61 * d1 - l42 * l62 * d2 - l43 * l63 * d3) / d4;
            var d5 = a55 - (l51 * l51 * d1 + l52 * l52 * d2 + l53 * l53 * d3 + l54 * l54 * d4);
            var l65 = (a65 - l51 * l61 * d1 - l52 * l62 * d2 - l53 * l63 * d3 - l54 * l64 * d4) / d5;
            var d6 = a66 - (l61 * l61 * d1 + l62 * l62 * d2 + l63 * l63 * d3 + l64 * l64 * d4 + l65 * l65 * d5);

            var y1 = bLin.x;
            var y2 = bLin.y - l21 * y1;
            var y3 = bLin.z - l31 * y1 - l32 * y2;
            var y4 = bAng.x - l41 * y1 - l42 * y2 - l43 * y3;
            var y5 = bAng.y - l51 * y1 - l52 * y2 - l53 * y3 - l54 * y4;
            var y6 = bAng.z - l61 * y1 - l62 * y2 - l63 * y3 - l64 * y4 - l65 * y5;

            var z1 = y1 / d1;
            var z2 = y2 / d2;
            var z3 = y3 / d3;
            var z4 = y4 / d4;
            var z5 = y5 / d5;
            var z6 = y6 / d6;

            xAng.z = z6;
            xAng.y = z5 - l65 * xAng.z;
            xAng.x = z4 - l54 * xAng.y - l64 * xAng.z;
            xLin.z = z3 - l43 * xAng.x - l53 * xAng.y - l63 * xAng.z;
            xLin.y = z2 - l32 * xLin.z - l42 * xAng.x - l52 * xAng.y - l62 * xAng.z;
            xLin.x = z1 - l21 * xLin.y - l31 * xLin.z - l41 * xAng.x - l51 * xAng.y - l61 * xAng.z;
        }
    }
}
