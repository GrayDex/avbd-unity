using Unity.Burst;
using Unity.Mathematics;

namespace Avbd
{
    // Setup arithmetic as Burst direct calls: the editor's Mono keeps float sums in double precision,
    // so the same expression in plain C# rounds differently there than in a player or in the step job.
    [BurstCompile]
    internal static class Derive
    {
        [BurstCompile(CompileSynchronously = true)]
        public static void Box(in float3 size, float density, out float mass, out float3 moment, out float radius)
        {
            mass = size.x * size.y * size.z * density;
            moment = new float3(
                (size.y * size.y + size.z * size.z) / 12f * mass,
                (size.x * size.x + size.z * size.z) / 12f * mass,
                (size.x * size.x + size.y * size.y) / 12f * mass);
            radius = math.length(size * 0.5f);
        }

        [BurstCompile(CompileSynchronously = true)]
        public static float LengthSq(in float3 v) => math.lengthsq(v);

        [BurstCompile(CompileSynchronously = true)]
        public static float Distance(in float3 positionA, in quaternion rotationA, in float3 rA, in float3 positionB, in quaternion rotationB, in float3 rB) =>
            math.length(Maths.Transform(positionA, rotationA, rA) - Maths.Transform(positionB, rotationB, rB));
    }
}
