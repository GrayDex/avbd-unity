using NUnit.Framework;
using Unity.Mathematics;

namespace Avbd.Tests
{
    // Joint angle limits on a pendulum: a 1 m bar pinned at one end to the world, falling about world z.
    // The joint frame turns x to world z, so the hinge angle is the twist. A twist limit alone does not hold
    // a ball joint: the bar swings round the vertical at the bound's height and falls past it.
    public class LimitTests
    {
        private static readonly quaternion Frame = quaternion.RotateY(-math.PI / 2f);

        private static Solver Pendulum(out int bar, out int joint)
        {
            var solver = new Solver();
            bar = solver.AddBox(new float3(1f, 0.1f, 0.1f), 1f, 0.5f, new float3(0.5f, 5f, 0f), quaternion.identity);
            joint = solver.AddJoint(-1, bar, new float3(0f, 5f, 0f), new float3(-0.5f, 0f, 0f));
            return solver;
        }

        // Degrees of the bar about world z, from the horizontal.
        private static float Hinge(Solver solver, int bar)
        {
            var x = math.rotate(solver.Get(bar).Rotation, math.right());
            return math.degrees(math.atan2(x.y, x.x));
        }

        private static float Tilt(Solver solver, int bar) =>
            math.degrees(math.acos(math.clamp(math.rotate(solver.Get(bar).Rotation, math.forward()).z, -1f, 1f)));

        [Test]
        public void StopsAtLimit()
        {
            using var solver = Pendulum(out var bar, out var joint);
            var limit = math.radians(30f);
            solver.LimitJoint(joint, Frame, Frame, new float3(-limit, 0f, 0f), new float3(limit, 0f, 0f));
            float lo = float.MaxValue, hi = float.MinValue;
            for (var step = 0; step < 240; step++)
            {
                solver.Step();
                if (step < 180) continue;
                lo = math.min(lo, Hinge(solver, bar));
                hi = math.max(hi, Hinge(solver, bar));
            }

            UnityEngine.Debug.Log($"[Avbd] limit −30°: last second {lo:F3}…{hi:F3}°, tilt {Tilt(solver, bar):F3}°");
            Assert.AreEqual(-30f, Hinge(solver, bar), 0.5f, "rests at the bound");
            Assert.Less(hi - lo, 0.05f, "no jitter");
        }

        [Test]
        public void InsideLimitsChangesNothing()
        {
            using var free = Pendulum(out var freeBar, out _);
            using var limited = Pendulum(out var bar, out var joint);
            var limit = math.radians(90f);
            limited.LimitJoint(joint, Frame, Frame, -limit, limit);
            for (var step = 0; step < 15; step++)
            {
                free.Step();
                limited.Step();
                Assert.Greater(Hinge(limited, bar), -80f, "still inside");
                Assert.AreEqual(free.Get(freeBar).Position, limited.Get(bar).Position, $"step {step}");
                Assert.AreEqual(free.Get(freeBar).Rotation, limited.Get(bar).Rotation, $"step {step}");
            }
        }

        [Test]
        public void LockedSwingsMakeAHinge()
        {
            using var ball = Pendulum(out var ballBar, out _);
            using var hinge = Pendulum(out var bar, out var joint);
            hinge.LimitJoint(joint, Frame, Frame, new float3(float.NegativeInfinity, 0f, 0f), new float3(float.PositiveInfinity, 0f, 0f));
            ball.Get(ballBar).AngularVelocity = hinge.Get(bar).AngularVelocity = new float3(3f, 0f, 0f);
            float tilt = 0f, ballTilt = 0f;
            for (var step = 0; step < 120; step++)
            {
                ball.Step();
                hinge.Step();
                tilt = math.max(tilt, Tilt(hinge, bar));
                ballTilt = math.max(ballTilt, Tilt(ball, ballBar));
            }

            UnityEngine.Debug.Log($"[Avbd] hinge tilt {tilt:F3}°, ball joint {ballTilt:F1}°");
            Assert.Greater(ballTilt, 5f, "the push tilts a ball joint");
            Assert.Less(tilt, 0.5f, "a hinge stays in its plane");
        }

        [Test]
        public void AngleGradientsMatchDifferences()
        {
            var random = new Random(7);
            const float h = 1e-3f;
            for (var n = 0; n < 50; n++)
            {
                var r = math.normalize(new quaternion(random.NextFloat4(-1f, 1f)));
                var angles = Step.Angles(r, out var gradient);
                for (var i = 0; i < 3; i++)
                {
                    var u = float3.zero;
                    u[i] = h;
                    var plus = Step.Angles(math.mul(quaternion.AxisAngle(math.normalize(u), h), r), out _);
                    var minus = Step.Angles(math.mul(quaternion.AxisAngle(math.normalize(u), -h), r), out _);
                    var numeric = (plus - minus) / (2f * h);
                    if (math.any(math.abs(plus - angles) > 1f)) continue; // wrapped through ±π
                    for (var a = 0; a < 3; a++)
                        Assert.AreEqual(numeric[a], gradient[i][a], 2e-2f, $"rotation {n}, ∂angle{a}/∂u{i}");
                }
            }
        }
    }
}
