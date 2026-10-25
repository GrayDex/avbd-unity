using Unity.Mathematics;
using UnityEngine;

namespace Avbd
{
    // Holds an anchor of this body to an anchor of the connected one, or to a world point without one.
    // Infinite stiffness is a hard row, zero a free one: linear ∞ and angular 0 is a ball joint, both ∞ a weld,
    // finite values make springs. Past the fracture torque the joint breaks and leaves the solver.
    // Angle limits follow ConfigurableJoint: twist about Axis, swing about Secondary Axis and their cross,
    // measured from the pose at enable — the angular stiffness then pulls back to that pose.
    [RequireComponent(typeof(AvbdBody))]
    public class AvbdJoint : MonoBehaviour
    {
        [SerializeField, Tooltip("None pins this body to the world point Connected Anchor.")] private AvbdBody connected;
        [SerializeField, Tooltip("On this body, in its axes.")] private Vector3 anchor;
        [SerializeField, Tooltip("On the connected body, in its axes; a world point without one.")] private Vector3 connectedAnchor;
        [SerializeField] private float linearStiffness = float.PositiveInfinity;
        [SerializeField] private float angularStiffness;
        [SerializeField] private float fracture = float.PositiveInfinity;

        [Header("Angle limits")]
        [SerializeField, Tooltip("Twist axis, in this body's axes.")] private Vector3 axis = Vector3.right;
        [SerializeField, Tooltip("First swing axis, in this body's axes.")] private Vector3 secondaryAxis = Vector3.up;
        [SerializeField] private AngularMotion angularXMotion, angularYMotion, angularZMotion;
        [SerializeField, Range(-180f, 180f)] private float lowAngularXLimit = -45f;
        [SerializeField, Range(-180f, 180f)] private float highAngularXLimit = 45f;
        [SerializeField, Range(0f, 180f)] private float angularYLimit = 45f;
        [SerializeField, Range(0f, 180f)] private float angularZLimit = 45f;

        private AvbdWorld _world;

        public int Id { get; private set; } = -1;
        public bool Broken => Id >= 0 && _world && _world.Alive && !_world.Solver.ContainsForce(Id);

        public void Connect(AvbdBody body, Vector3 at, Vector3 connectedAt, float linear = float.PositiveInfinity, float angular = 0f,
            float breaksAt = float.PositiveInfinity)
        {
            connected = body;
            anchor = at;
            connectedAnchor = connectedAt;
            linearStiffness = linear;
            angularStiffness = angular;
            fracture = breaksAt;
            if (!isActiveAndEnabled) return;
            OnDisable();
            OnEnable();
        }

        private void OnEnable()
        {
            var body = GetComponent<AvbdBody>();
            if (!body.Register() || connected && !connected.Register()) return;
            _world = body.World;
            Id = _world.Solver.AddJoint(connected ? connected.Id : -1, body.Id, connectedAnchor, anchor, linearStiffness, angularStiffness, fracture);
            if (angularXMotion != AngularMotion.Free || angularYMotion != AngularMotion.Free || angularZMotion != AngularMotion.Free) Limit(body);
        }

        private void Limit(AvbdBody body)
        {
            var x = math.normalize(axis);
            var z = math.normalize(math.cross(x, secondaryAxis));
            var frame = new quaternion(new float3x3(x, math.cross(z, x), z));
            var rotationA = connected ? _world.Solver.Get(connected.Id).Rotation : quaternion.identity;
            var connectedFrame = math.mul(math.inverse(rotationA), math.mul(_world.Solver.Get(body.Id).Rotation, frame));
            var lower = new float3(Bound(angularXMotion, lowAngularXLimit, -1f), Bound(angularYMotion, -angularYLimit, -1f),
                Bound(angularZMotion, -angularZLimit, -1f));
            var upper = new float3(Bound(angularXMotion, highAngularXLimit, 1f), Bound(angularYMotion, angularYLimit, 1f),
                Bound(angularZMotion, angularZLimit, 1f));
            _world.Solver.LimitJoint(Id, connectedFrame, frame, lower, upper);
        }

        private static float Bound(AngularMotion motion, float degrees, float side) => motion switch
        {
            AngularMotion.Locked => 0f,
            AngularMotion.Limited => math.radians(degrees),
            _ => side * float.PositiveInfinity
        };

        private void OnDisable()
        {
            if (Id >= 0 && _world && _world.Alive) _world.Solver.RemoveForce(Id);
            Id = -1;
        }
    }

    public enum AngularMotion
    {
        Free,
        Limited,
        Locked
    }
}
