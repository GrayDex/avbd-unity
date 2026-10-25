using Unity.Mathematics;

namespace Avbd
{
    // A rigid box. Mass 0 is static or kinematic: the solver moves it only if you do.
    public struct Body
    {
        public int Id;
        public float3 Position;
        public quaternion Rotation;
        public float3 Velocity;
        public float3 AngularVelocity;
        public float3 Size; // full widths
        public float Mass;
        public float3 Moment; // principal moments of inertia, body axes
        public float Friction;
        public float Radius; // bounding sphere for the broadphase

        internal float3 Initial, Inertial, PreviousVelocity;
        internal quaternion InitialRotation, InertialRotation;

        public bool Static => Mass <= 0f;

        public static Body Box(float3 size, float density, float friction, float3 position, quaternion rotation, float3 velocity)
        {
            Derive.Box(size, density, out var mass, out var moment, out var radius);
            return new Body
            {
                Position = position, Rotation = rotation, Velocity = velocity, PreviousVelocity = velocity,
                Size = size, Mass = mass, Moment = moment, Friction = friction, Radius = radius
            };
        }
    }
}
