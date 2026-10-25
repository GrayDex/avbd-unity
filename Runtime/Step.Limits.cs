using Unity.Mathematics;

namespace Avbd
{
    // Joint angle limits — rows with a clamped force (§3.2), as fmin/fmax in avbd-demo2d. B's joint frame relative
    // to A's splits into twist about x, then swing about y and z, as PhysX D6 and Unity's ConfigurableJoint do.
    // Per axis: lower = upper is one hard two-sided row, otherwise a one-sided row per finite bound, C ≥ 0 inside.
    internal partial struct Step
    {
        private float3 LimitAngles(in Force joint, out quaternion frameA, out float3x3 gradient)
        {
            frameA = Maths.Mul(joint.A >= 0 ? Bodies[joint.A].Rotation : quaternion.identity, joint.FrameA);
            var relative = Maths.Mul(Maths.Inverse(frameA), Maths.Mul(Bodies[joint.B].Rotation, joint.FrameB));
            return Angles(relative, out gradient);
        }

        // Twist and swing angles of r, and their gradients (rows) by u for r ← exp(u)·r — a world turn ω of B
        // is u = ω in A's joint frame, of A it is −u.
        internal static float3 Angles(quaternion relative, out float3x3 gradient)
        {
            var r = relative.value;
            if (r.w < 0f) r = -r;
            var d = r.w * r.w + r.x * r.x;
            var dd = new float4(2f * r.x, 0f, 0f, 2f * r.w);
            var twist = Atan(r, r.x, new float4(1f, 0f, 0f, 0f), r.w, new float4(0f, 0f, 0f, 1f), out var gx);
            var swingY = Atan(r, r.y * r.w - r.z * r.x, new float4(-r.z, r.w, -r.x, r.y), d, dd, out var gy);
            var swingZ = Atan(r, r.z * r.w + r.y * r.x, new float4(r.y, r.x, r.w, r.z), d, dd, out var gz);
            gradient = Maths.Rows(gx, gy, gz);
            return new float3(twist, swingY, swingZ);
        }

        // θ = 2·atan2(n, d) with partials of n and d by r's (x, y, z, w); δr = ½(0, u)·r turns them into ∂θ/∂u.
        private static float Atan(float4 r, float n, float4 dn, float d, float4 dd, out float3 gradient)
        {
            var dTheta = 2f * (d * dn - n * dd) / math.max(n * n + d * d, 1e-12f);
            gradient = 0.5f * (r.w * dTheta.xyz - dTheta.w * r.xyz + math.cross(r.xyz, dTheta.xyz));
            return 2f * math.atan2(n, d);
        }

        private static bool Locked(in Force joint, int axis) => !(joint.Lower[axis] < joint.Upper[axis]);

        // The row's constraint with the step-start error ignored (Eq. 18); a bound only ignores its violation,
        // or the row would push before the bound is reached.
        private float LimitC(in Force joint, float3 angles, int axis, bool high)
        {
            var c = (high ? joint.Upper[axis] - angles[axis] : angles[axis] - joint.Lower[axis]) * joint.TorqueArm;
            var c0 = high ? joint.C0High[axis] : joint.C0Low[axis];
            return c - S.Alpha * (Locked(joint, axis) ? c0 : math.min(c0, 0f));
        }

        private static bool Row(in Force joint, int axis, bool high) =>
            high ? !Locked(joint, axis) && joint.Upper[axis] < float.PositiveInfinity : joint.Lower[axis] > float.NegativeInfinity;

        private static float RowMax(in Force joint, int axis) => Locked(joint, axis) ? float.PositiveInfinity : 0f;

        private void LimitInitialize(ref Force joint)
        {
            var angles = LimitAngles(joint, out _, out _);
            joint.C0Low = (angles - joint.Lower) * joint.TorqueArm;
            joint.C0High = (joint.Upper - angles) * joint.TorqueArm;
            joint.LambdaLow = joint.LambdaLow * S.Alpha * S.Gamma;
            joint.LambdaHigh = joint.LambdaHigh * S.Alpha * S.Gamma;
            joint.PenaltyLow = Maths.Clamp(joint.PenaltyLow * S.Gamma, Maths.PenaltyMin, Maths.PenaltyMax);
            joint.PenaltyHigh = Maths.Clamp(joint.PenaltyHigh * S.Gamma, Maths.PenaltyMin, Maths.PenaltyMax);
        }

        // A row clamped at its bound stamps the rescaled stiffness of Eq. 14, (λmax − λ) / C: none once λ is spent,
        // so a penalty a limit ramped up does not brake the joint after it leaves the limit.
        private void LimitPrimal(in Force joint, bool isA, ref Lhs lhs, ref float3 rhsAng)
        {
            var angles = LimitAngles(joint, out var frameA, out var gradient);
            var rows = math.transpose(gradient);
            for (var axis = 0; axis < 3; axis++)
            for (var side = 0; side < 2; side++)
            {
                var high = side == 1;
                if (!Row(joint, axis, high)) continue;
                var k = high ? joint.PenaltyHigh[axis] : joint.PenaltyLow[axis];
                var lambda = high ? joint.LambdaHigh[axis] : joint.LambdaLow[axis];
                var c = LimitC(joint, angles, axis, high);
                var f = Maths.Clamp(k * c + lambda, float.NegativeInfinity, RowMax(joint, axis));
                if (!(f < RowMax(joint, axis))) k = c > 0f ? -lambda / c : 0f; // only a one-sided row, λmax = 0, clamps
                var j = Maths.Rotate(frameA, rows[axis]) * (joint.TorqueArm * (high == isA ? 1f : -1f));
                lhs.Ang += Maths.Outer(j, j) * k;
                rhsAng += j * f;
            }
        }

        private void LimitDual(ref Force joint)
        {
            var angles = LimitAngles(joint, out _, out _);
            for (var axis = 0; axis < 3; axis++)
            for (var side = 0; side < 2; side++)
            {
                var high = side == 1;
                if (!Row(joint, axis, high)) continue;
                var c = LimitC(joint, angles, axis, high);
                ref var k = ref high ? ref joint.PenaltyHigh : ref joint.PenaltyLow;
                ref var lambda = ref high ? ref joint.LambdaHigh : ref joint.LambdaLow;
                lambda[axis] = Maths.Clamp(k[axis] * c + lambda[axis], float.NegativeInfinity, RowMax(joint, axis));
                if (lambda[axis] < RowMax(joint, axis)) k[axis] = math.min(k[axis] + S.BetaAng * math.abs(c), Maths.PenaltyMax);
            }
        }
    }
}
