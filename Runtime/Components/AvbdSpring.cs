using UnityEngine;

namespace Avbd
{
    // A linear spring between an anchor of this body and one of the connected body.
    [RequireComponent(typeof(AvbdBody))]
    public class AvbdSpring : MonoBehaviour
    {
        [SerializeField] private AvbdBody connected;
        [SerializeField, Tooltip("On this body, in its axes.")] private Vector3 anchor;
        [SerializeField, Tooltip("On the connected body, in its axes.")] private Vector3 connectedAnchor;
        [SerializeField, Min(0f), Tooltip("N/m.")] private float stiffness = 100f;
        [SerializeField, Tooltip("Metres; negative — the distance when enabled.")] private float rest = -1f;

        private AvbdWorld _world;

        public int Id { get; private set; } = -1;

        public void Connect(AvbdBody body, Vector3 at, Vector3 connectedAt, float k, float length = -1f)
        {
            connected = body;
            anchor = at;
            connectedAnchor = connectedAt;
            stiffness = k;
            rest = length;
            if (!isActiveAndEnabled) return;
            OnDisable();
            OnEnable();
        }

        private void OnEnable()
        {
            var body = GetComponent<AvbdBody>();
            if (!connected || !body.Register() || !connected.Register()) return;
            _world = body.World;
            Id = _world.Solver.AddSpring(connected.Id, body.Id, connectedAnchor, anchor, stiffness, rest);
        }

        private void OnDisable()
        {
            if (Id >= 0 && _world && _world.Alive) _world.Solver.RemoveForce(Id);
            Id = -1;
        }
    }
}
