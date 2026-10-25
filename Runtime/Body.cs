using Unity.Mathematics;

namespace Avbd
{
    public enum BodyShape : byte
    {
        Box,
        Point // a point mass: position only, no rotation, no collisions
    }

    // A rigid box or a point mass. Mass 0 is static or kinematic: the solver moves it only if you do.
    public struct Body
    {
        public int Id;
        public BodyShape Shape;
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

        public static Body Point(float mass, float3 position, float3 velocity) => new()
        {
            Shape = BodyShape.Point, Position = position, Rotation = quaternion.identity, Velocity = velocity,
            PreviousVelocity = velocity, Mass = mass
        };
    }
}
