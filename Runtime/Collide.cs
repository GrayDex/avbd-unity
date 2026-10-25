using Unity.Collections;
using Unity.Mathematics;

namespace Avbd
{
    // Box against box, the demo's collide.cpp: separating axes, then a clipped face or the closest edge points.
    internal static class Collide
    {
        public const int Most = 8;
        private const int MostPolygon = 16;
        private const float AxisEpsilon = 1e-6f;
        private const float PlaneEpsilon = 1e-5f;
        private const float MergeDistanceSq = 1e-6f;

        private const int FaceA = 0, FaceB = 1, Edge = 2;

        private struct Obb
        {
            public float3 Center;
            public float3 Half;
            public float3x3 Axis; // column i — local axis i in world space
        }

        private struct Candidate
        {
            public int Type, IndexA, IndexB;
            public float Separation;
            public float3 Normal;
            public bool Valid;
        }

        private struct Face
        {
            public float3 Normal, Center, U, V;
            public float ExtentU, ExtentV;
        }

        public static int Boxes(in Body a, in Body b, ref FixedList4096Bytes<Contact> contacts, out float3x3 basis)
        {
            basis = default;
            contacts.Clear();
            var boxA = Make(a);
            var boxB = Make(b);
            var delta = boxB.Center - boxA.Center;

            var bestFace = new Candidate { Separation = -float.MaxValue };
            var bestEdge = new Candidate { Separation = -float.MaxValue };
            for (var i = 0; i < 3; i++)
                if (!Test(boxA, boxB, delta, boxA.Axis[i], FaceA, i, -1, ref bestFace)) return 0;
            for (var i = 0; i < 3; i++)
                if (!Test(boxA, boxB, delta, boxB.Axis[i], FaceB, -1, i, ref bestFace)) return 0;
            for (var i = 0; i < 3; i++)
            for (var j = 0; j < 3; j++)
                if (!Test(boxA, boxB, delta, math.cross(boxA.Axis[i], boxB.Axis[j]), Edge, i, j, ref bestEdge)) return 0;
            if (!bestFace.Valid) return 0;

            var best = bestFace;
            if (bestEdge.Valid && 0.95f * bestEdge.Separation > bestFace.Separation + 0.01f) best = bestEdge;

            basis = Maths.Orthonormal(-best.Normal);
            if (best.Type == Edge) return EdgeContact(a, b, boxA, boxB, best.IndexA, best.IndexB, best.Normal, ref contacts);
            return best.Type == FaceA
                ? FaceManifold(a, b, boxA, boxB, true, best.IndexA, best.Normal, ref contacts)
                : FaceManifold(a, b, boxA, boxB, false, best.IndexB, best.Normal, ref contacts);
        }

        private static Obb Make(in Body body) => new()
        {
            Center = body.Position,
            Half = body.Size * 0.5f,
            Axis = new float3x3(
                Maths.Rotate(body.Rotation, new float3(1f, 0f, 0f)),
                Maths.Rotate(body.Rotation, new float3(0f, 1f, 0f)),
                Maths.Rotate(body.Rotation, new float3(0f, 0f, 1f)))
        };

        private static float AbsDot(float3 a, float3 b) => math.abs(math.dot(a, b));

        private static float3 Support(in Obb box, float3 direction)
        {
            var sx = math.dot(direction, box.Axis[0]) >= 0f ? 1f : -1f;
            var sy = math.dot(direction, box.Axis[1]) >= 0f ? 1f : -1f;
            var sz = math.dot(direction, box.Axis[2]) >= 0f ? 1f : -1f;
            return box.Center + box.Axis[0] * (box.Half.x * sx) + box.Axis[1] * (box.Half.y * sy) + box.Axis[2] * (box.Half.z * sz);
        }

        private static void FaceAxes(in Obb box, int axis, out float3 u, out float3 v, out float extentU, out float extentV)
        {
            u = axis == 0 ? box.Axis[1] : box.Axis[0];
            v = axis == 2 ? box.Axis[1] : box.Axis[2];
            extentU = axis == 0 ? box.Half.y : box.Half.x;
            extentV = axis == 2 ? box.Half.y : box.Half.z;
        }

        private static Face Reference(in Obb box, int axis, float3 outward)
        {
            var sign = math.dot(outward, box.Axis[axis]) >= 0f ? 1f : -1f;
            var face = new Face { Normal = box.Axis[axis] * sign };
            face.Center = box.Center + face.Normal * box.Half[axis];
            FaceAxes(box, axis, out face.U, out face.V, out face.ExtentU, out face.ExtentV);
            return face;
        }

        private static int IncidentAxis(in Obb box, float3 normal)
        {
            var axis = 0;
            var best = -float.MaxValue;
            for (var i = 0; i < 3; i++)
            {
                var d = AbsDot(box.Axis[i], normal);
                if (d <= best) continue;
                best = d;
                axis = i;
            }

            return axis;
        }

        private static void Incident(in Obb box, int axis, float3 normal, ref FixedList512Bytes<float3> polygon)
        {
            var sign = math.dot(box.Axis[axis], normal) > 0f ? -1f : 1f;
            var faceNormal = box.Axis[axis] * sign;
            var center = box.Center + faceNormal * box.Half[axis];
            FaceAxes(box, axis, out var u, out var v, out var extentU, out var extentV);
            polygon.Clear();
            polygon.Add(center + u * extentU + v * extentV);
            polygon.Add(center - u * extentU + v * extentV);
            polygon.Add(center - u * extentU - v * extentV);
            polygon.Add(center + u * extentU - v * extentV);
        }

        private static void Clip(in FixedList512Bytes<float3> input, float3 normal, float offset, ref FixedList512Bytes<float3> output)
        {
            output.Clear();
            if (input.Length == 0) return;
            var a = input[^1];
            var da = math.dot(normal, a) - offset;
            for (var i = 0; i < input.Length; i++)
            {
                var b = input[i];
                var db = math.dot(normal, b) - offset;
                var aInside = da <= PlaneEpsilon;
                var bInside = db <= PlaneEpsilon;
                if (aInside != bInside)
                {
                    var t = 0f;
                    var denominator = da - db;
                    if (math.abs(denominator) > AxisEpsilon) t = Maths.Clamp(da / denominator, 0f, 1f);
                    if (output.Length < MostPolygon) output.Add(a + (b - a) * t);
                }

                if (bInside && output.Length < MostPolygon) output.Add(b);
                a = b;
                da = db;
            }
        }

        private static void Add(in Body a, in Body b, ref FixedList4096Bytes<Contact> contacts, ref FixedList128Bytes<float3> midpoints,
            float3 xA, float3 xB, int feature)
        {
            var midpoint = (xA + xB) * 0.5f;
            for (var i = 0; i < contacts.Length; i++)
                if (math.lengthsq(midpoint - midpoints[i]) < MergeDistanceSq) return;
            if (contacts.Length >= Most) return;
            contacts.Add(new Contact
            {
                Feature = feature,
                AnchorA = Maths.Rotate(Maths.Conjugate(a.Rotation), xA - a.Position),
                AnchorB = Maths.Rotate(Maths.Conjugate(b.Rotation), xB - b.Position)
            });
            midpoints.Add(midpoint);
        }

        private static bool Test(in Obb boxA, in Obb boxB, float3 delta, float3 axis, int type, int indexA, int indexB, ref Candidate best)
        {
            var lengthSq = math.lengthsq(axis);
            if (lengthSq < AxisEpsilon) return true;
            var n = axis * (1f / math.sqrt(lengthSq));
            if (math.dot(n, delta) < 0f) n = -n;

            var distance = math.abs(math.dot(delta, n));
            var rA = boxA.Half.x * AbsDot(n, boxA.Axis[0]) + boxA.Half.y * AbsDot(n, boxA.Axis[1]) + boxA.Half.z * AbsDot(n, boxA.Axis[2]);
            var rB = boxB.Half.x * AbsDot(n, boxB.Axis[0]) + boxB.Half.y * AbsDot(n, boxB.Axis[1]) + boxB.Half.z * AbsDot(n, boxB.Axis[2]);
            var separation = distance - (rA + rB);
            if (separation > 0f) return false;
            if (best.Valid && separation <= best.Separation) return true;
            best = new Candidate { Valid = true, Type = type, IndexA = indexA, IndexB = indexB, Separation = separation, Normal = n };
            return true;
        }

        private static void SupportEdge(in Obb box, int axis, float3 direction, out float3 from, out float3 to)
        {
            var axis1 = (axis + 1) % 3;
            var axis2 = (axis + 2) % 3;
            var sign1 = math.dot(direction, box.Axis[axis1]) >= 0f ? 1f : -1f;
            var sign2 = math.dot(direction, box.Axis[axis2]) >= 0f ? 1f : -1f;
            var center = box.Center + box.Axis[axis1] * (box.Half[axis1] * sign1) + box.Axis[axis2] * (box.Half[axis2] * sign2);
            from = center - box.Axis[axis] * box.Half[axis];
            to = center + box.Axis[axis] * box.Half[axis];
        }

        private static void ClosestOnSegments(float3 p0, float3 p1, float3 q0, float3 q1, out float3 c0, out float3 c1)
        {
            var d1 = p1 - p0;
            var d2 = q1 - q0;
            var r = p0 - q0;
            var a = math.dot(d1, d1);
            var e = math.dot(d2, d2);
            var f = math.dot(d2, r);
            float s = 0f, t = 0f;
            if (a <= AxisEpsilon && e <= AxisEpsilon)
            {
                c0 = p0;
                c1 = q0;
                return;
            }

            if (a <= AxisEpsilon)
            {
                t = Maths.Clamp(f / e, 0f, 1f);
            }
            else
            {
                var c = math.dot(d1, r);
                if (e <= AxisEpsilon)
                {
                    s = Maths.Clamp(-c / a, 0f, 1f);
                }
                else
                {
                    var b = math.dot(d1, d2);
                    var denominator = a * e - b * b;
                    if (math.abs(denominator) > AxisEpsilon) s = Maths.Clamp((b * f - c * e) / denominator, 0f, 1f);
                    t = (b * s + f) / e;
                    if (t < 0f)
                    {
                        t = 0f;
                        s = Maths.Clamp(-c / a, 0f, 1f);
                    }
                    else if (t > 1f)
                    {
                        t = 1f;
                        s = Maths.Clamp((b - c) / a, 0f, 1f);
                    }
                }
            }

            c0 = p0 + d1 * s;
            c1 = q0 + d2 * t;
        }

        private static int FaceManifold(in Body a, in Body b, in Obb boxA, in Obb boxB, bool referenceIsA, int referenceAxis, float3 normal,
            ref FixedList4096Bytes<Contact> contacts)
        {
            var reference = Reference(referenceIsA ? boxA : boxB, referenceAxis, referenceIsA ? normal : -normal);
            var incidentBox = referenceIsA ? boxB : boxA;
            var incidentAxis = IncidentAxis(incidentBox, reference.Normal);

            var clip0 = new FixedList512Bytes<float3>();
            var clip1 = new FixedList512Bytes<float3>();
            Incident(incidentBox, incidentAxis, reference.Normal, ref clip0);
            Clip(clip0, reference.U, math.dot(reference.U, reference.Center) + reference.ExtentU, ref clip1);
            if (clip1.Length == 0) return 0;
            Clip(clip1, -reference.U, math.dot(-reference.U, reference.Center) + reference.ExtentU, ref clip0);
            if (clip0.Length == 0) return 0;
            Clip(clip0, reference.V, math.dot(reference.V, reference.Center) + reference.ExtentV, ref clip1);
            if (clip1.Length == 0) return 0;
            Clip(clip1, -reference.V, math.dot(-reference.V, reference.Center) + reference.ExtentV, ref clip0);
            if (clip0.Length == 0) return 0;

            var midpoints = new FixedList128Bytes<float3>();
            var prefix = ((referenceIsA ? FaceA : FaceB) << 24) | ((referenceAxis & 0xFF) << 16) | ((incidentAxis & 0xFF) << 8);
            for (var i = 0; i < clip0.Length && contacts.Length < Most; i++)
            {
                var incident = clip0[i];
                var distance = math.dot(incident - reference.Center, reference.Normal);
                if (distance > PlaneEpsilon) continue;
                var onReference = incident - reference.Normal * distance;
                Add(a, b, ref contacts, ref midpoints, referenceIsA ? onReference : incident, referenceIsA ? incident : onReference, prefix | (i & 0xFF));
            }

            if (contacts.Length == 0) Add(a, b, ref contacts, ref midpoints, Support(boxA, normal), Support(boxB, -normal), prefix);
            return contacts.Length;
        }

        private static int EdgeContact(in Body a, in Body b, in Obb boxA, in Obb boxB, int axisA, int axisB, float3 normal,
            ref FixedList4096Bytes<Contact> contacts)
        {
            SupportEdge(boxA, axisA, normal, out var a0, out var a1);
            SupportEdge(boxB, axisB, -normal, out var b0, out var b1);
            ClosestOnSegments(a0, a1, b0, b1, out var xA, out var xB);

            var midpoints = new FixedList128Bytes<float3>();
            var feature = (Edge << 24) | ((axisA & 0xFF) << 8) | (axisB & 0xFF);
            Add(a, b, ref contacts, ref midpoints, xA, xB, feature);
            if (contacts.Length == 0) Add(a, b, ref contacts, ref midpoints, Support(boxA, normal), Support(boxB, -normal), feature);
            return contacts.Length;
        }
    }
}
