using System;
using NUnit.Framework;
using Unity.Mathematics;

namespace Avbd.Tests
{
    // Point masses: 3 degrees of freedom, held by joints and springs, alone and together with boxes.
    public class PointTests
    {
        [Test]
        public void FallsAsImplicitEuler()
        {
            using var solver = new Solver();
            var point = solver.AddPoint(1f, new float3(0f, 10f, 0f));
            var dt = solver.Settings.Dt;
            var g = solver.Settings.Gravity.y;
            for (var n = 1; n <= 60; n++)
            {
                solver.Step();
                Assert.AreEqual(10f + g * dt * dt * n * (n + 1) / 2f, solver.Get(point).Position.y, 1e-3f, $"step {n}"); // (x − x₀)/dt rounding
            }

            Assert.AreEqual(quaternion.identity, solver.Get(point).Rotation, "no rotation");
        }

        // A box hung by one end from the world, the tip glued by a joint to its other end, horizontal at rest.
        // A point tip must move as a 1 cm box of the same mass; the joints' own gap (~5 mm at 10 iterations)
        // is the solver's, the demo's world pivot opens as much.
        [Test]
        public void HangsLikeATinyBox()
        {
            using var withPoint = Rig(true, out var box, out var point);
            using var withBox = Rig(false, out var refBox, out var refTip);
            float apart = 0f, gap = 0f, rise = float.NegativeInfinity, lowest = 5f;
            var mBox = withPoint.Get(box).Mass;
            for (var step = 0; step < 300; step++)
            {
                withPoint.Step();
                withBox.Step();
                var b = withPoint.Get(box);
                var p = withPoint.Get(point).Position;
                apart = math.max(apart, math.distance(p, withBox.Get(refTip).Position));
                apart = math.max(apart, math.distance(b.Position, withBox.Get(refBox).Position));
                gap = math.max(gap, math.distance(math.transform(new float4x4(b.Rotation, b.Position), new float3(0.5f, 0f, 0f)), p));
                rise = math.max(rise, mBox * b.Position.y + p.y - (mBox + 1f) * 5f);
                lowest = math.min(lowest, p.y);
            }

            UnityEngine.Debug.Log($"[Avbd] box + point: {apart * 1000f:F4} mm from the tiny box, joint gap {gap * 1000f:F3} mm, potential rise {rise:E2} kg·m");
            Assert.Less(apart, 1e-4f, "as a tiny box");
            Assert.Less(lowest, 4.1f, "swings down to the bottom");
            Assert.Less(rise, 1e-3f, "no energy gained");
        }

        private static Solver Rig(bool point, out int box, out int tip)
        {
            var solver = new Solver();
            box = solver.AddBox(new float3(1f, 0.1f, 0.1f), 100f, 0.5f, new float3(0.5f, 5f, 0f), quaternion.identity);
            tip = point
                ? solver.AddPoint(1f, new float3(1f, 5f, 0f))
                : solver.AddBox(new float3(0.01f), 1e6f, 0.5f, new float3(1f, 5f, 0f), quaternion.identity);
            solver.AddJoint(-1, box, new float3(0f, 5f, 0f), new float3(-0.5f, 0f, 0f));
            solver.AddJoint(box, tip, new float3(0.5f, 0f, 0f), float3.zero);
            return solver;
        }

        [Test]
        public void SpringSettlesAtItsLoad()
        {
            using var solver = new Solver();
            const float k = 200f, mass = 1f;
            var top = solver.AddPoint(0f, new float3(0f, 5f, 0f));
            var weight = solver.AddPoint(mass, new float3(0f, 4f, 0f));
            solver.AddSpring(top, weight, float3.zero, float3.zero, k);
            for (var step = 0; step < 600; step++) solver.Step();
            var stretch = 4f - solver.Get(weight).Position.y;
            UnityEngine.Debug.Log($"[Avbd] spring: stretch {stretch * 1000f:F2} mm, expected {mass * 10f / k * 1000f:F2}");
            Assert.AreEqual(mass * 10f / k, stretch, 0.05f * mass * 10f / k);
        }

        [Test]
        public void RefusesRotationAndAnchors()
        {
            using var solver = new Solver();
            var point = solver.AddPoint(1f, float3.zero);
            Assert.Throws<ArgumentException>(() => solver.AddJoint(-1, point, float3.zero, float3.zero, float.PositiveInfinity, 1f));
            var joint = solver.AddJoint(-1, point, float3.zero, float3.zero);
            Assert.Throws<ArgumentException>(() => solver.LimitJoint(joint, quaternion.identity, quaternion.identity, float3.zero, float3.zero));
            Assert.Throws<ArgumentException>(() => solver.AddSpring(solver.AddPoint(0f, math.up()), point, float3.zero, math.right(), 1f));
        }
    }
}
