using Unity.Collections;
using Unity.Mathematics;

namespace Avbd
{
    // The demo's joint.cpp, spring.cpp and manifold.cpp: what each force stamps into its body's 6×6 block,
    // and the dual step after each iteration (λ ← F, penalty ramp of Eq. 16).
    internal partial struct Step
    {
        private static readonly float3x3 Identity = float3x3.identity;

        private float3 JointError(in Force joint) =>
            (joint.A >= 0 ? Maths.Transform(Bodies[joint.A].Position, Bodies[joint.A].Rotation, joint.AnchorA) : joint.AnchorA)
            - Maths.Transform(Bodies[joint.B].Position, Bodies[joint.B].Rotation, joint.AnchorB);

        private float3 JointAngle(in Force joint)
        {
            var a = joint.A >= 0 ? Bodies[joint.A].Rotation : quaternion.identity;
            var b = Bodies[joint.B].Rotation;
            if (joint.Framed) (a, b) = (Maths.Mul(a, joint.FrameA), Maths.Mul(b, joint.FrameB));
            return Maths.Difference(a, b) * joint.TorqueArm;
        }

        private bool JointInitialize(ref Force joint)
        {
            joint.C0Lin = JointError(joint);
            joint.C0Ang = JointAngle(joint);
            joint.LambdaLin = joint.LambdaLin * S.Alpha * S.Gamma;
            joint.LambdaAng = joint.LambdaAng * S.Alpha * S.Gamma;
            joint.PenaltyLin = Maths.Clamp(joint.PenaltyLin * S.Gamma, Maths.PenaltyMin, Maths.PenaltyMax);
            joint.PenaltyAng = Maths.Clamp(joint.PenaltyAng * S.Gamma, Maths.PenaltyMin, Maths.PenaltyMax);
            joint.PenaltyLin = Maths.Min(joint.PenaltyLin, joint.StiffnessLin);
            joint.PenaltyAng = Maths.Min(joint.PenaltyAng, joint.StiffnessAng);
            if (joint.Framed) LimitInitialize(ref joint);
            return !joint.Broken;
        }

        private void JointPrimal(in Force joint, int body, ref Lhs lhs, ref float3 rhsLin, ref float3 rhsAng)
        {
            var isA = body == joint.A;
            if (math.lengthsq(joint.PenaltyLin) > 0f)
            {
                var k = Maths.Diagonal(joint.PenaltyLin);
                var c = JointError(joint);
                if (math.isinf(joint.StiffnessLin)) c -= joint.C0Lin * S.Alpha;
                var f = math.mul(k, c) + joint.LambdaLin;

                var rotated = isA ? Maths.Rotate(Bodies[joint.A].Rotation, joint.AnchorA) : Maths.Rotate(Bodies[joint.B].Rotation, joint.AnchorB);
                var jLin = isA ? Identity : -Identity;
                var jAng = isA ? Maths.Skew(-rotated) : Maths.Skew(rotated);
                lhs.Stamp(jLin, jAng, k);

                // Diagonal approximation of the second-order term (geometric stiffness of the anchor arm).
                var r = isA ? rotated : -rotated;
                var h = Maths.BallSocketStiffness(0, r) * f.x + Maths.BallSocketStiffness(1, r) * f.y + Maths.BallSocketStiffness(2, r) * f.z;
                lhs.Ang += Maths.Diagonalize(h);
                rhsLin += math.mul(math.transpose(jLin), f);
                rhsAng += math.mul(math.transpose(jAng), f);
            }

            if (math.lengthsq(joint.PenaltyAng) > 0f)
            {
                var k = Maths.Diagonal(joint.PenaltyAng);
                var c = JointAngle(joint);
                if (math.isinf(joint.StiffnessAng)) c -= joint.C0Ang * S.Alpha;
                var f = math.mul(k, c) + joint.LambdaAng;
                var jAng = (isA ? Identity : -Identity) * joint.TorqueArm;
                var jAngT = math.transpose(jAng);
                lhs.Ang += math.mul(math.mul(jAngT, k), jAng);
                rhsAng += math.mul(jAngT, f);
            }

            if (joint.Framed) LimitPrimal(joint, isA, ref lhs, ref rhsAng);
        }

        private void JointDual(ref Force joint)
        {
            if (math.lengthsq(joint.PenaltyLin) > 0f)
            {
                var c = JointError(joint);
                if (math.isinf(joint.StiffnessLin))
                {
                    c -= joint.C0Lin * S.Alpha;
                    joint.LambdaLin = math.mul(Maths.Diagonal(joint.PenaltyLin), c) + joint.LambdaLin;
                }

                joint.PenaltyLin = Maths.Min(joint.PenaltyLin + math.abs(c) * S.BetaLin, math.min(joint.StiffnessLin, Maths.PenaltyMax));
            }

            if (math.lengthsq(joint.PenaltyAng) > 0f)
            {
                var c = JointAngle(joint);
                if (math.isinf(joint.StiffnessAng))
                {
                    c -= joint.C0Ang * S.Alpha;
                    joint.LambdaAng = math.mul(Maths.Diagonal(joint.PenaltyAng), c) + joint.LambdaAng;
                }

                joint.PenaltyAng = Maths.Min(joint.PenaltyAng + math.abs(c) * S.BetaAng, math.min(joint.StiffnessAng, Maths.PenaltyMax));
            }

            if (joint.Framed) LimitDual(ref joint);
            if (math.lengthsq(joint.LambdaAng) <= joint.Fracture * joint.Fracture) return;
            joint.PenaltyLin = joint.PenaltyAng = joint.LambdaLin = joint.LambdaAng = float3.zero;
            joint.Broken = true;
        }

        private void SpringPrimal(in Force spring, int body, ref Lhs lhs, ref float3 rhsLin, ref float3 rhsAng)
        {
            var a = Bodies[spring.A];
            var b = Bodies[spring.B];
            var d = Maths.Transform(a.Position, a.Rotation, spring.AnchorA) - Maths.Transform(b.Position, b.Rotation, spring.AnchorB);
            var length = math.length(d);
            if (length <= 1e-6f) return;

            var n = d / length;
            var stiffness = spring.StiffnessLin;
            var f = stiffness * (length - spring.Rest);
            float3 jLin, jAng;
            if (body == spring.A)
            {
                jLin = n;
                jAng = math.cross(Maths.Rotate(a.Rotation, spring.AnchorA), n);
            }
            else
            {
                jLin = -n;
                jAng = -math.cross(Maths.Rotate(b.Rotation, spring.AnchorB), n);
            }

            lhs.Lin += Maths.Outer(jLin, jLin) * stiffness;
            lhs.Ang += Maths.Outer(jAng, jAng) * stiffness;
            lhs.Cross += Maths.Outer(jAng, jLin) * stiffness;
            rhsLin += jLin * f;
            rhsAng += jAng * f;
        }

        private bool ManifoldInitialize(ref Force manifold)
        {
            var a = Bodies[manifold.A];
            var b = Bodies[manifold.B];
            manifold.Friction = math.sqrt(a.Friction * b.Friction);

            var fresh = new FixedList4096Bytes<Contact>();
            Collide.Boxes(a, b, ref fresh, out var basis);
            if (fresh.Length > 0) manifold.Basis = basis;
            var block = manifold.Block * Collide.Most;
            for (var i = 0; i < fresh.Length; i++)
            for (var j = 0; j < manifold.Count; j++)
            {
                var old = Contacts[block + j];
                if (fresh[i].Feature != old.Feature) continue;
                // Static friction last step: keep the anchors where they stuck, so the contact does not creep.
                if (!old.Stick) (old.AnchorA, old.AnchorB) = (fresh[i].AnchorA, fresh[i].AnchorB);
                fresh[i] = old;
                break;
            }

            manifold.Count = fresh.Length;
            for (var i = 0; i < fresh.Length; i++)
            {
                var contact = fresh[i];
                var xA = Maths.Transform(a.Position, a.Rotation, contact.AnchorA);
                var xB = Maths.Transform(b.Position, b.Rotation, contact.AnchorB);
                contact.C0 = math.mul(manifold.Basis, xA - xB) + new float3(Maths.CollisionMargin, 0f, 0f);
                contact.Lambda = contact.Lambda * S.Alpha * S.Gamma;
                contact.Penalty = Maths.Clamp(contact.Penalty * S.Gamma, Maths.PenaltyMin, Maths.PenaltyMax);
                Contacts[block + i] = contact;
            }

            return manifold.Count > 0;
        }

        // Taylor expansion of every contact row about the step start (paper §4), force clamped to the friction cone.
        private float3 ContactForce(in Force manifold, in Contact contact, out float3 c, out float3x3 jALin, out float3x3 jBLin,
            out float3x3 jAAng, out float3x3 jBAng, out bool sliding)
        {
            var a = Bodies[manifold.A];
            var b = Bodies[manifold.B];
            var rAWorld = Maths.Rotate(a.Rotation, contact.AnchorA);
            var rBWorld = Maths.Rotate(b.Rotation, contact.AnchorB);
            jALin = manifold.Basis;
            jBLin = -manifold.Basis;
            var rowsA = math.transpose(jALin);
            var rowsB = math.transpose(jBLin);
            jAAng = Maths.Rows(math.cross(rAWorld, rowsA.c0), math.cross(rAWorld, rowsA.c1), math.cross(rAWorld, rowsA.c2));
            jBAng = Maths.Rows(math.cross(rBWorld, rowsB.c0), math.cross(rBWorld, rowsB.c1), math.cross(rBWorld, rowsB.c2));

            c = contact.C0 * (1f - S.Alpha)
                + math.mul(jALin, a.Position - a.Initial) + math.mul(jBLin, b.Position - b.Initial)
                + math.mul(jAAng, Maths.Difference(a.Rotation, a.InitialRotation)) + math.mul(jBAng, Maths.Difference(b.Rotation, b.InitialRotation));
            var f = math.mul(Maths.Diagonal(contact.Penalty), c) + contact.Lambda;
            f.x = math.min(f.x, 0f);

            var bounds = math.abs(f.x) * manifold.Friction;
            var scale = math.length(new float2(f.y, f.z));
            sliding = scale > bounds;
            if (!sliding || !(scale > 0f)) return f;
            f.y *= bounds / scale;
            f.z *= bounds / scale;
            return f;
        }

        private void ManifoldPrimal(in Force manifold, int body, ref Lhs lhs, ref float3 rhsLin, ref float3 rhsAng)
        {
            var isA = body == manifold.A;
            for (var i = 0; i < manifold.Count; i++)
            {
                var contact = Contacts[manifold.Block * Collide.Most + i];
                var f = ContactForce(manifold, contact, out _, out var jALin, out var jBLin, out var jAAng, out var jBAng, out _);
                var jLin = isA ? jALin : jBLin;
                var jAng = isA ? jAAng : jBAng;
                lhs.Stamp(jLin, jAng, Maths.Diagonal(contact.Penalty));
                rhsLin += math.mul(math.transpose(jLin), f);
                rhsAng += math.mul(math.transpose(jAng), f);
            }
        }

        private void ManifoldDual(ref Force manifold)
        {
            for (var i = 0; i < manifold.Count; i++)
            {
                ref var contact = ref Contacts.ElementAt(manifold.Block * Collide.Most + i);
                var f = ContactForce(manifold, contact, out var c, out _, out _, out _, out _, out var sliding);
                contact.Lambda = f;
                if (f.x < 0f) contact.Penalty.x = math.min(contact.Penalty.x + S.BetaLin * math.abs(c.x), Maths.PenaltyMax);
                if (sliding) continue;
                contact.Penalty.y = math.min(contact.Penalty.y + S.BetaLin * math.abs(c.y), Maths.PenaltyMax);
                contact.Penalty.z = math.min(contact.Penalty.z + S.BetaLin * math.abs(c.z), Maths.PenaltyMax);
                contact.Stick = math.length(new float2(c.y, c.z)) < Maths.StickThreshold;
            }
        }
    }
}
