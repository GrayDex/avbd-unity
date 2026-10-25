using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Avbd
{
    public struct Settings
    {
        public float Dt;
        public float3 Gravity;
        public int Iterations;
        public float Alpha; // stabilisation: 1 keeps all of the error from the step start, 0 removes it at once
        public float BetaLin, BetaAng; // penalty ramp of linear and angular rows
        public float Gamma; // warm-start decay of λ and penalties, < 1

        public static Settings Default => new()
        {
            Dt = 1f / 60f, Gravity = new float3(0f, -10f, 0f), Iterations = 10,
            Alpha = 0.99f, BetaLin = 10000f, BetaAng = 100f, Gamma = 0.999f
        };
    }

    // One step of the demo's Solver::step on one thread. Bodies and forces are kept in creation order and walked
    // from the newest, as the demo's prepend-linked lists are, since Gauss–Seidel results depend on the order.
    [BurstCompile(CompileSynchronously = true)]
    internal partial struct Step : IJob
    {
        public NativeList<Body> Bodies;
        public NativeList<Force> Forces;
        public NativeList<Contact> Contacts;
        public NativeList<int> FreeBlocks;
        public NativeHashMap<long, int> Pairs; // body-id pair → forces between them
        public NativeList<int> Start, Adjacent; // forces of body i: Adjacent[Start[i] .. Start[i + 1]), newest first
        public Settings S;

        public void Execute()
        {
            Broadphase();
            for (var f = Forces.Length - 1; f >= 0; f--)
                if (!Initialize(ref Forces.ElementAt(f)))
                    Remove(f);

            var dt = S.Dt;
            var g = S.Gravity;
            var down = g / math.length(g);
            for (var i = Bodies.Length - 1; i >= 0; i--)
            {
                ref var body = ref Bodies.ElementAt(i);
                body.Inertial = body.Position + body.Velocity * dt;
                if (body.Mass > 0f) body.Inertial += g * (dt * dt);
                body.InertialRotation = Maths.Add(body.Rotation, body.AngularVelocity * dt);

                // Adaptive warm start of VBD: as much gravity as the body actually fell with last step.
                var accel = (body.Velocity - body.PreviousVelocity) / dt;
                var weight = Maths.Clamp(math.dot(accel, down) / math.length(g), 0f, 1f);
                if (!math.isfinite(weight)) weight = 0f;

                body.Initial = body.Position;
                body.InitialRotation = body.Rotation;
                if (body.Mass <= 0f) continue;
                body.Position = body.Position + body.Velocity * dt + g * (weight * dt * dt);
                body.Rotation = Maths.Add(body.Rotation, body.AngularVelocity * dt);
            }

            Adjacency();
            for (var it = 0; it < S.Iterations; it++)
            {
                for (var i = Bodies.Length - 1; i >= 0; i--)
                    if (Bodies[i].Mass > 0f)
                        Primal(i);
                for (var f = Forces.Length - 1; f >= 0; f--) Dual(ref Forces.ElementAt(f));
            }

            for (var i = Bodies.Length - 1; i >= 0; i--)
            {
                ref var body = ref Bodies.ElementAt(i);
                body.PreviousVelocity = body.Velocity;
                if (body.Mass <= 0f) continue;
                body.Velocity = (body.Position - body.Initial) / dt;
                body.AngularVelocity = Maths.Difference(body.Rotation, body.InitialRotation) / dt;
            }
        }

        // Naive O(n²) like the demo: a manifold for every pair whose bounding spheres meet and that no force links.
        private void Broadphase()
        {
            for (var a = Bodies.Length - 1; a >= 0; a--)
            for (var b = a - 1; b >= 0; b--)
            {
                if (Bodies[a].Shape != BodyShape.Box || Bodies[b].Shape != BodyShape.Box) continue;
                var dp = Bodies[a].Position - Bodies[b].Position;
                var r = Bodies[a].Radius + Bodies[b].Radius;
                if (math.dot(dp, dp) > r * r || Pairs.ContainsKey(Pair(Bodies[a].Id, Bodies[b].Id))) continue;
                int block;
                if (FreeBlocks.Length > 0)
                {
                    block = FreeBlocks[^1];
                    FreeBlocks.RemoveAt(FreeBlocks.Length - 1);
                }
                else
                {
                    block = Contacts.Length / Collide.Most;
                    Contacts.Resize(Contacts.Length + Collide.Most, NativeArrayOptions.ClearMemory);
                }

                Link(new Force { Id = -1, Kind = ForceKind.Manifold, A = a, B = b, Block = block });
            }
        }

        private void Primal(int i)
        {
            var body = Bodies[i];
            var dt2 = S.Dt * S.Dt;
            var mLin = Maths.Diagonal(body.Mass) / dt2;
            var mAng = Maths.Diagonal(body.Moment) / dt2;
            var lhs = new Lhs { Lin = mLin, Ang = mAng };
            var rhsLin = math.mul(mLin, body.Position - body.Inertial);
            var rhsAng = math.mul(mAng, Maths.Difference(body.Rotation, body.InertialRotation));

            for (var k = Start[i]; k < Start[i + 1]; k++)
            {
                ref var force = ref Forces.ElementAt(Adjacent[k]);
                switch (force.Kind)
                {
                    case ForceKind.Joint: JointPrimal(force, i, ref lhs, ref rhsLin, ref rhsAng); break;
                    case ForceKind.Spring: SpringPrimal(force, i, ref lhs, ref rhsLin, ref rhsAng); break;
                    case ForceKind.Manifold: ManifoldPrimal(force, i, ref lhs, ref rhsLin, ref rhsAng); break;
                    case ForceKind.IgnoreCollision:
                    default: break;
                }
            }

            ref var moved = ref Bodies.ElementAt(i);
            if (body.Shape == BodyShape.Point)
            {
                moved.Position += math.mul(math.inverse(lhs.Lin), -rhsLin); // 3×3 block: forces' angular rows fall away
                return;
            }

            Maths.Solve(lhs.Lin, lhs.Ang, lhs.Cross, -rhsLin, -rhsAng, out var dxLin, out var dxAng);
            moved.Position += dxLin;
            moved.Rotation = Maths.Add(moved.Rotation, dxAng);
        }

        private bool Initialize(ref Force force) => force.Kind switch
        {
            ForceKind.Joint => JointInitialize(ref force),
            ForceKind.Manifold => ManifoldInitialize(ref force),
            _ => true
        };

        private void Dual(ref Force force)
        {
            if (force.Kind == ForceKind.Joint) JointDual(ref force);
            else if (force.Kind == ForceKind.Manifold) ManifoldDual(ref force);
        }

        private void Adjacency()
        {
            var n = Bodies.Length;
            Start.Clear();
            Start.Resize(n + 1, NativeArrayOptions.ClearMemory);
            for (var f = 0; f < Forces.Length; f++)
            {
                if (Forces[f].A >= 0) Start[Forces[f].A + 1]++;
                Start[Forces[f].B + 1]++;
            }

            for (var i = 0; i < n; i++) Start[i + 1] += Start[i];
            Adjacent.Resize(Start[n], NativeArrayOptions.UninitializedMemory);
            var fill = new NativeArray<int>(n, Allocator.Temp);
            for (var f = Forces.Length - 1; f >= 0; f--)
            {
                var a = Forces[f].A;
                var b = Forces[f].B;
                if (a >= 0) Adjacent[Start[a] + fill[a]++] = f;
                Adjacent[Start[b] + fill[b]++] = f;
            }
        }

        internal void Link(Force force)
        {
            Forces.Add(force);
            if (force.A < 0) return;
            var key = Pair(Bodies[force.A].Id, Bodies[force.B].Id);
            Pairs[key] = Pairs.TryGetValue(key, out var count) ? count + 1 : 1;
        }

        internal void Remove(int f)
        {
            var force = Forces[f];
            if (force.Kind == ForceKind.Manifold) FreeBlocks.Add(force.Block);
            if (force.A >= 0)
            {
                var key = Pair(Bodies[force.A].Id, Bodies[force.B].Id);
                var count = Pairs[key] - 1;
                if (count > 0) Pairs[key] = count;
                else Pairs.Remove(key);
            }

            Forces.RemoveAt(f);
        }

        internal static long Pair(int a, int b) => ((long)math.min(a, b) << 32) | (uint)math.max(a, b);

        private struct Lhs
        {
            public float3x3 Lin, Ang, Cross; // Cross: angular rows, linear columns

            // Stamps JᵀKJ of rows with linear Jacobian jLin and angular jAng.
            public void Stamp(float3x3 jLin, float3x3 jAng, float3x3 k)
            {
                var jLinT = math.transpose(jLin);
                var jAngTk = math.mul(math.transpose(jAng), k);
                Lin += math.mul(math.mul(jLinT, k), jLin);
                Ang += math.mul(jAngTk, jAng);
                Cross += math.mul(jAngTk, jLin);
            }
        }
    }
}
