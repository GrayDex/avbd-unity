// ReSharper disable RedundantCast — the casts round to float, which the editor's Mono does not do by itself
using System;
using Unity.Mathematics;

namespace Avbd.Tests
{
    // avbd-demo3d's scenes.h through the Solver API, in the demo's frame (z up), for the reference comparison.
    // Every sum is cast to float: the editor's Mono keeps intermediates in double and would round otherwise.
    internal static class DemoScenes
    {
        private static readonly float3 Zero = float3.zero;

        public static readonly (string Name, Action<Solver> Build)[] All =
        {
            ("Ground", Ground), ("Dynamic Friction", DynamicFriction), ("Static Friction", StaticFriction), ("Pyramid", Pyramid),
            ("Rope", Rope), ("Heavy Rope", HeavyRope), ("Spring", Spring), ("Spring Ratio", SpringsRatio), ("Stack", Stack),
            ("Stack Ratio", StackRatio), ("Soft Body", SoftBody), ("Bridge", Bridge), ("Breakable", Breakable)
        };

        public static Solver Create(int scene)
        {
            var solver = new Solver();
            solver.Settings.Gravity = new float3(0f, 0f, -10f);
            All[scene].Build(solver);
            return solver;
        }

        private static int Box(Solver s, float3 size, float density, float friction, float3 position, float3 velocity = default) =>
            s.AddBox(size, density, friction, position, quaternion.identity, velocity);

        private static void Floor(Solver s, float z = 0f) => Box(s, new float3(100, 100, 1), 0f, 0.5f, new float3(0, 0, z));

        private static void Ground(Solver s)
        {
            Floor(s);
            Box(s, 1, 1f, 0.5f, new float3(0, 0, 4));
        }

        private static void DynamicFriction(Solver s)
        {
            Floor(s);
            for (var x = 0; x <= 10; x++)
                Box(s, new float3(1, 1, 0.5f), 1f, (float)(5f - (float)((float)(x / 10f) * 5f)), new float3(0, -30f + x * 2f, 0.75f), new float3(10f, 0, 0));
        }

        private static void StaticFriction(Solver s)
        {
            Floor(s);
            const float angle = 30f * 0.01745329251994329577f;
            var rotation = new quaternion(0, (float)Math.Sin(angle * 0.5f), 0, (float)Math.Cos(angle * 0.5f));
            var ramp = s.AddBox(new float3(40, 24, 1), 0f, 1f, new float3(0, 0, 3), rotation);
            var tangent = Maths.Rotate(rotation, new float3(1, 0, 0));
            tangent /= math.sqrt(Derive.LengthSq(tangent));
            var normal = Maths.Rotate(rotation, new float3(0, 0, 1));
            normal /= math.sqrt(Derive.LengthSq(normal));
            for (var i = 0; i <= 10; i++)
            {
                var friction = (float)((float)((float)(i / 10f) * 0.25f) + 0.25f);
                var y = -10f + i * 2f;
                Box(s, 1, 1f, friction, s.Get(ramp).Position + tangent * -12f + new float3(0, y, 0) + normal * 1.05f);
            }
        }

        private static void Pyramid(Solver s)
        {
            const int size = 16;
            Floor(s, -0.5f);
            for (var y = 0; y < size; y++)
            for (var x = 0; x < size - y; x++)
                Box(s, new float3(1, 0.5f, 0.5f), 1f, 0.5f, new float3((float)((float)((float)(x * 1.01f) + y * 0.5f) - size / 2f), 0f, (float)((float)(y * 0.85f) + 0.5f)));
        }

        private static void Rope(Solver s)
        {
            Floor(s, -20f);
            var previous = -1;
            for (var i = 0; i < 20; i++)
            {
                var current = Box(s, new float3(1, 0.5f, 0.5f), i == 0 ? 0f : 1f, 0.5f, new float3(i, 0f, 10f));
                if (previous >= 0) s.AddJoint(previous, current, new float3(0.5f, 0, 0), new float3(-0.5f, 0, 0));
                previous = current;
            }
        }

        private static void HeavyRope(Solver s)
        {
            const int n = 20;
            const float size = 5f;
            Floor(s, -20f);
            var previous = -1;
            for (var i = 0; i < n; i++)
            {
                var last = i == n - 1;
                var current = Box(s, last ? new float3(size) : new float3(1, 0.5f, 0.5f), i == 0 ? 0f : 1f, 0.5f,
                    new float3(i + (last ? size / 2 : 0), 0f, 10f));
                if (previous >= 0) s.AddJoint(previous, current, new float3(0.5f, 0, 0), last ? new float3(-size / 2, 0, 0) : new float3(-0.5f, 0, 0));
                previous = current;
            }
        }

        private static void Spring(Solver s)
        {
            Floor(s);
            var anchor = Box(s, 1, 0f, 0.5f, new float3(0, 0, 14f));
            var block = Box(s, 2, 1f, 0.5f, new float3(0, 0, 8f));
            s.AddSpring(anchor, block, Zero, Zero, 100f, 4f);
        }

        private static void SpringsRatio(Solver s)
        {
            const int n = 8;
            Floor(s, -10f);
            var previous = -1;
            for (var i = 0; i < n; i++)
            {
                var x = (i - (n - 1) * 0.5f) * 3f;
                var current = Box(s, new float3(1, 0.75f, 0.75f), i == 0 || i == n - 1 ? 0f : 1f, 0.5f, new float3(x, 0f, 12f));
                if (previous >= 0) s.AddSpring(previous, current, new float3(0.5f, 0, 0), new float3(-0.5f, 0, 0), i % 2 == 0 ? 10f : 10000f, 3f);
                previous = current;
            }
        }

        private static void Stack(Solver s)
        {
            Floor(s);
            for (var i = 0; i < 10; i++) Box(s, 1, 1f, 0.5f, new float3(0, 0, i * 1.5f + 1f));
        }

        private static void StackRatio(Solver s)
        {
            Floor(s);
            var top = 0.5f;
            var size = 1f;
            for (var i = 0; i < 4; i++)
            {
                var half = size * 0.5f;
                var center = top + half;
                Box(s, size, 1f, 0.5f, new float3(0, 0, center));
                top = center + half;
                size *= 2f;
            }
        }

        private static void SoftBody(Solver s)
        {
            const float lin = 1000f, ang = 250f, size = 0.8f, half = size * 0.5f, baseZ = 8f, gap = 2f;
            const int w = 4, d = 4, h = 4;
            Floor(s);
            for (var i = 0; i < 3; i++)
            {
                var grid = new int[w, d, h];
                var stackZ = (float)(i * (float)((float)(h * size) + gap));
                for (var x = 0; x < w; x++)
                for (var y = 0; y < d; y++)
                for (var z = 0; z < h; z++)
                    grid[x, y, z] = Box(s, size, 1f, 0.5f, new float3((float)((x - (w - 1) * 0.5f) * size), (float)((y - (d - 1) * 0.5f) * size), (float)((float)(baseZ + stackZ) + (float)(z * size))));

                for (var x = 1; x < w; x++)
                for (var y = 0; y < d; y++)
                for (var z = 0; z < h; z++)
                    s.AddJoint(grid[x - 1, y, z], grid[x, y, z], new float3(half, 0, 0), new float3(-half, 0, 0), lin, ang);
                for (var x = 0; x < w; x++)
                for (var y = 1; y < d; y++)
                for (var z = 0; z < h; z++)
                    s.AddJoint(grid[x, y - 1, z], grid[x, y, z], new float3(0, half, 0), new float3(0, -half, 0), lin, ang);
                for (var x = 0; x < w; x++)
                for (var y = 0; y < d; y++)
                for (var z = 1; z < h; z++)
                    s.AddJoint(grid[x, y, z - 1], grid[x, y, z], new float3(0, 0, half), new float3(0, 0, -half), lin, ang);

                for (var x = 1; x < w; x++)
                for (var y = 0; y < d; y++)
                for (var z = 1; z < h; z++)
                {
                    s.IgnoreCollision(grid[x - 1, y, z - 1], grid[x, y, z]);
                    s.IgnoreCollision(grid[x, y, z - 1], grid[x - 1, y, z]);
                }

                for (var x = 0; x < w; x++)
                for (var y = 1; y < d; y++)
                for (var z = 1; z < h; z++)
                {
                    s.IgnoreCollision(grid[x, y - 1, z - 1], grid[x, y, z]);
                    s.IgnoreCollision(grid[x, y, z - 1], grid[x, y - 1, z]);
                }

                for (var x = 1; x < w; x++)
                for (var y = 1; y < d; y++)
                for (var z = 0; z < h; z++)
                {
                    s.IgnoreCollision(grid[x - 1, y - 1, z], grid[x, y, z]);
                    s.IgnoreCollision(grid[x, y - 1, z], grid[x - 1, y, z]);
                }
            }
        }

        private static void Bridge(Solver s)
        {
            const int n = 40;
            const float length = 1f, width = 4f, height = 0.5f, halfLength = length * 0.5f, halfWidth = width * 0.5f;
            Floor(s);
            var previous = -1;
            for (var i = 0; i < n; i++)
            {
                var current = Box(s, new float3(length, width, height), i == 0 || i == n - 1 ? 0f : 1f, 0.5f, new float3(i - n / 2f, 0f, 10f));
                if (previous >= 0)
                {
                    s.AddJoint(previous, current, new float3(halfLength, halfWidth, 0), new float3(-halfLength, halfWidth, 0));
                    s.AddJoint(previous, current, new float3(halfLength, -halfWidth, 0), new float3(-halfLength, -halfWidth, 0));
                }

                previous = current;
            }

            for (var x = 0; x < n / 4; x++)
            for (var y = 0; y < n / 8; y++)
                Box(s, 1, 1f, 0.5f, new float3(x - n / 8f, 0f, y + 12f));
        }

        private static void Breakable(Solver s)
        {
            const int n = 10, m = 5;
            const float breakForce = 90f;
            Floor(s);
            var previous = -1;
            for (var i = 0; i <= n; i++)
            {
                var current = Box(s, new float3(1, 1, 0.5f), 1f, 0.5f, new float3(i - n / 2f, 0f, 6f));
                if (previous >= 0)
                    s.AddJoint(previous, current, new float3(0.5f, 0, 0), new float3(-0.5f, 0, 0), float.PositiveInfinity, float.PositiveInfinity, breakForce);
                previous = current;
            }

            Box(s, new float3(1, 1, 5), 0f, 0.5f, new float3(-n / 2f, 0, 2.5f));
            Box(s, new float3(1, 1, 5), 0f, 0.5f, new float3(n / 2f, 0, 2.5f));
            for (var i = 0; i < m; i++) Box(s, new float3(2, 1, 1), 1f, 0.5f, new float3(0, 0, i * 2f + 8f));
        }
    }
}
