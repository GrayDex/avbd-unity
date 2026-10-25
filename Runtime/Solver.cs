using System;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Avbd
{
    // The world: bodies and forces under stable ids, stepped by one Burst job on the calling thread.
    // Bodies and forces stay in creation order — the order the demo solves them in.
    public sealed class Solver : IDisposable
    {
        public Settings Settings = Settings.Default;

        private NativeList<Body> _bodies = new(Allocator.Persistent);
        private NativeList<Force> _forces = new(Allocator.Persistent);
        private NativeList<Contact> _contacts = new(Allocator.Persistent);
        private NativeList<int> _freeBlocks = new(Allocator.Persistent);
        private NativeHashMap<long, int> _pairs = new(64, Allocator.Persistent);
        private NativeList<int> _start = new(Allocator.Persistent);
        private NativeList<int> _adjacent = new(Allocator.Persistent);
        private NativeList<int> _index = new(Allocator.Persistent); // body id → index, −1 once removed
        private int _nextBody, _nextForce;

        public NativeArray<Body> Bodies => _bodies.AsArray();
        public int BodyCount => _bodies.Length;
        public int ForceCount => _forces.Length;

        public int ContactCount
        {
            get
            {
                var count = 0;
                foreach (var force in _forces)
                    if (force.Kind == ForceKind.Manifold)
                        count += force.Count;
                return count;
            }
        }

        public void Dispose()
        {
            if (!_bodies.IsCreated) return;
            _bodies.Dispose();
            _forces.Dispose();
            _contacts.Dispose();
            _freeBlocks.Dispose();
            _pairs.Dispose();
            _start.Dispose();
            _adjacent.Dispose();
            _index.Dispose();
        }

        public void Clear()
        {
            _bodies.Clear();
            _forces.Clear();
            _contacts.Clear();
            _freeBlocks.Clear();
            _pairs.Clear();
            for (var i = 0; i < _index.Length; i++) _index[i] = -1;
        }

        public void Step() => Job.Run();

        public int AddBox(float3 size, float density, float friction, float3 position, quaternion rotation, float3 velocity = default)
        {
            var body = Body.Box(size, density, friction, position, rotation, velocity);
            body.Id = _nextBody++;
            _index.Add(_bodies.Length);
            _bodies.Add(body);
            return body.Id;
        }

        // A point mass: joints and springs hold it by its position (its anchor is zero), so a joint glues it
        // to the other anchor. It has no rotation and collides with nothing.
        public int AddPoint(float mass, float3 position, float3 velocity = default)
        {
            var body = Body.Point(mass, position, velocity);
            body.Id = _nextBody++;
            _index.Add(_bodies.Length);
            _bodies.Add(body);
            return body.Id;
        }

        public ref Body Get(int id) => ref _bodies.ElementAt(IndexOf(id));

        public bool Contains(int body) => Find(body) >= 0;

        public void RemoveBody(int id)
        {
            var index = IndexOf(id);
            var job = Job;
            for (var f = _forces.Length - 1; f >= 0; f--)
                if (_forces[f].A == index || _forces[f].B == index)
                    job.Remove(f);
            _bodies.RemoveAt(index);
            _index[id] = -1;
            for (var i = index; i < _bodies.Length; i++) _index[_bodies[i].Id] = i;
            for (var f = 0; f < _forces.Length; f++)
            {
                ref var force = ref _forces.ElementAt(f);
                if (force.A > index) force.A--;
                if (force.B > index) force.B--;
            }
        }

        // A null bodyA (−1) pins B's anchor rB to the world point rA. Infinite stiffness makes a hard row,
        // zero frees it: linear ∞ with angular 0 is a ball joint, both ∞ a weld, finite — a spring.
        public int AddJoint(int bodyA, int bodyB, float3 rA, float3 rB, float stiffnessLin = float.PositiveInfinity,
            float stiffnessAng = 0f, float fracture = float.PositiveInfinity)
        {
            var a = bodyA >= 0 ? IndexOf(bodyA) : -1;
            var b = IndexOf(bodyB);
            if (stiffnessAng != 0f && (Pointed(a) || Pointed(b))) throw new ArgumentException("a point mass has no rotation to hold");
            Unanchored(a, rA);
            Unanchored(b, rB);
            var arm = (a >= 0 ? _bodies[a].Size : float3.zero) + _bodies[b].Size;
            return Link(new Force
            {
                Kind = ForceKind.Joint, A = a, B = b, AnchorA = rA, AnchorB = rB,
                StiffnessLin = stiffnessLin, StiffnessAng = stiffnessAng, Fracture = fracture, TorqueArm = Derive.LengthSq(arm)
            });
        }

        // Angle limits, radians, of B's joint frame from A's: twist about x, then swing about y and z. Frames are
        // the joint axes in each body's space (in the world for a world pin). Lower = upper locks an axis, infinite
        // bounds free it. The joint's angular stiffness then pulls toward the frames' rest instead of equal rotations.
        public void LimitJoint(int joint, quaternion frameA, quaternion frameB, float3 lower, float3 upper)
        {
            ref var force = ref GetForce(joint);
            if (force.Kind != ForceKind.Joint) throw new ArgumentException($"force {joint} is not a joint");
            if (Pointed(force.A) || Pointed(force.B)) throw new ArgumentException("a point mass has no rotation to limit");
            force.FrameA = frameA;
            force.FrameB = frameB;
            force.Lower = lower;
            force.Upper = upper;
            force.Framed = true;
        }

        // A negative rest length takes the current distance between the anchors.
        public int AddSpring(int bodyA, int bodyB, float3 rA, float3 rB, float stiffness, float rest = -1f)
        {
            var a = IndexOf(bodyA);
            var b = IndexOf(bodyB);
            Unanchored(a, rA);
            Unanchored(b, rB);
            if (rest < 0f)
                rest = Derive.Distance(_bodies[a].Position, _bodies[a].Rotation, rA, _bodies[b].Position, _bodies[b].Rotation, rB);
            return Link(new Force { Kind = ForceKind.Spring, A = a, B = b, AnchorA = rA, AnchorB = rB, StiffnessLin = stiffness, Rest = rest });
        }

        public int IgnoreCollision(int bodyA, int bodyB) =>
            Link(new Force { Kind = ForceKind.IgnoreCollision, A = IndexOf(bodyA), B = IndexOf(bodyB) });

        public bool ContainsForce(int id) => FindForce(id) >= 0;

        // A broken joint leaves the solver by itself; removing it again is a no-op.
        public void RemoveForce(int id)
        {
            var f = FindForce(id);
            if (f >= 0) Job.Remove(f);
        }

        public ref Force GetForce(int id)
        {
            var f = FindForce(id);
            if (f < 0) throw new ArgumentException($"no force {id}");
            return ref _forces.ElementAt(f);
        }

        // The demo's pick: the nearest box a ray enters, with the hit point in its body space.
        public int Pick(float3 origin, float3 direction, out float3 local)
        {
            const float epsilon = 1e-6f;
            var bestT = float.PositiveInfinity;
            var best = -1;
            local = float3.zero;
            foreach (var body in _bodies)
            {
                if (body.Mass <= 0f || body.Shape != BodyShape.Box) continue;
                var inverse = Maths.Conjugate(body.Rotation);
                var o = Maths.Rotate(inverse, origin - body.Position);
                var d = Maths.Rotate(inverse, direction);
                var half = body.Size * 0.5f;
                float enter = 0f, exit = float.PositiveInfinity;
                var hit = true;
                for (var i = 0; i < 3 && hit; i++)
                {
                    if (math.abs(d[i]) < epsilon)
                    {
                        hit = o[i] >= -half[i] && o[i] <= half[i];
                        continue;
                    }

                    var t0 = (-half[i] - o[i]) / d[i];
                    var t1 = (half[i] - o[i]) / d[i];
                    enter = math.max(enter, math.min(t0, t1));
                    exit = math.min(exit, math.max(t0, t1));
                    hit = enter <= exit;
                }

                var t = enter >= 0f ? enter : exit;
                if (!hit || t < 0f || t >= bestT) continue;
                bestT = t;
                best = body.Id;
                local = o + d * t;
            }

            return best;
        }

        private Step Job => new()
        {
            Bodies = _bodies, Forces = _forces, Contacts = _contacts, FreeBlocks = _freeBlocks, Pairs = _pairs,
            Start = _start, Adjacent = _adjacent, S = Settings
        };

        private int Link(Force force)
        {
            force.Id = _nextForce++;
            Job.Link(force);
            return force.Id;
        }

        private bool Pointed(int index) => index >= 0 && _bodies[index].Shape == BodyShape.Point;

        private void Unanchored(int index, float3 anchor)
        {
            if (Pointed(index) && math.any(anchor != float3.zero)) throw new ArgumentException("a point mass is held at its position: anchor it at zero");
        }

        private int Find(int id) => id >= 0 && id < _index.Length ? _index[id] : -1;

        private int IndexOf(int id)
        {
            var index = Find(id);
            return index >= 0 ? index : throw new ArgumentException($"no body {id}");
        }

        private int FindForce(int id)
        {
            for (var f = 0; f < _forces.Length; f++)
                if (_forces[f].Id == id)
                    return f;
            return -1;
        }
    }
}
