using Unity.Mathematics;

namespace Avbd
{
    public enum ForceKind : byte
    {
        Joint,
        Spring,
        IgnoreCollision,
        Manifold
    }

    // Every force of the demo in one record: the kind picks which fields matter.
    // A and B are body indices; A = −1 pins a joint to the world point AnchorA.
    public struct Force
    {
        public int Id;
        public ForceKind Kind;
        public int A, B;

        // Joint and spring: anchors in body space. A spring keeps its stiffness in StiffnessLin.
        public float3 AnchorA, AnchorB;
        public float StiffnessLin, StiffnessAng, Fracture, Rest;
        internal float3 C0Lin, C0Ang, PenaltyLin, PenaltyAng, LambdaLin, LambdaAng;
        internal float TorqueArm;
        internal bool Broken;

        // Joint angle limits (Solver.LimitJoint): joint axes in body space, bounds on twist x, swing y and z.
        public quaternion FrameA, FrameB;
        public float3 Lower, Upper;
        internal bool Framed;
        internal float3 C0Low, C0High, PenaltyLow, PenaltyHigh, LambdaLow, LambdaHigh;

        // Manifold: up to Manifold.Most contacts at Contacts[Block * Most ..].
        internal float3x3 Basis; // rows: normal from B to A, two tangents
        internal int Block, Count;
        internal float Friction;
    }

    public struct Contact
    {
        public int Feature;
        public float3 AnchorA, AnchorB; // body-space offsets from the centres
        internal float3 C0, Penalty, Lambda;
        internal bool Stick;
    }
}
