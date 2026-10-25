using UnityEngine;

namespace Avbd
{
    // Holds an anchor of this body to an anchor of the connected one, or to a world point without one.
    // Infinite stiffness is a hard row, zero a free one: linear ∞ and angular 0 is a ball joint, both ∞ a weld,
    // finite values make springs. Past the fracture torque the joint breaks and leaves the solver.
    [RequireComponent(typeof(AvbdBody))]
    public class AvbdJoint : MonoBehaviour
    {
        [SerializeField, Tooltip("None pins this body to the world point Connected Anchor.")] private AvbdBody connected;
        [SerializeField, Tooltip("On this body, in its axes.")] private Vector3 anchor;
        [SerializeField, Tooltip("On the connected body, in its axes; a world point without one.")] private Vector3 connectedAnchor;
        [SerializeField] private float linearStiffness = float.PositiveInfinity;
        [SerializeField] private float angularStiffness;
        [SerializeField] private float fracture = float.PositiveInfinity;

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
        }

        private void OnDisable()
        {
            if (Id >= 0 && _world && _world.Alive) _world.Solver.RemoveForce(Id);
            Id = -1;
        }
    }
}
