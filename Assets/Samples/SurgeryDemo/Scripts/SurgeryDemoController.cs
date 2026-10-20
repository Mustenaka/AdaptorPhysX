using UnityEngine;
using UnityEngine.SceneManagement;

namespace AdaptorPhysX.Samples
{
    /// <summary>
    /// Mouse-driven scalpel and forceps for the surgery demo.
    /// 1 / 2: scalpel / forceps. LMB: cut or grab. RMB drag: orbit. Wheel: zoom.
    /// Q / E: blade depth. R: reload.
    /// </summary>
    [DefaultExecutionOrder(-100)]
    public sealed class SurgeryDemoController : MonoBehaviour
    {
        public enum Tool
        {
            Scalpel,
            Forceps,
        }

        public Tool tool = Tool.Scalpel;
        public ApxScalpel scalpel;
        public Vector3 orbitTarget = new Vector3(0.0f, 0.03f, 0.0f);
        public float orbitDistance = 0.55f;
        [Range(0.002f, 0.1f)] public float bladeDepth = 0.015f;
        public float grabRadius = 0.012f;

        private Camera _camera;
        private float _yaw = 0.0f;
        private float _pitch = 38.0f;
        private float _bladeHeight;
        private bool _bladeActive;
        private ApxSoftBody _grabbed;
        private Plane _grabPlane;
        private int _severedThisStroke;

        private void Start()
        {
            _camera = Camera.main;
            if (scalpel != null)
            {
                scalpel.cutting = false;
            }
        }

        private void Update()
        {
            if (Input.GetKeyDown(KeyCode.Alpha1)) tool = Tool.Scalpel;
            if (Input.GetKeyDown(KeyCode.Alpha2)) tool = Tool.Forceps;
            if (Input.GetKeyDown(KeyCode.R)) SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex);
            if (Input.GetKey(KeyCode.Q)) bladeDepth = Mathf.Max(0.002f, bladeDepth - 0.02f * Time.deltaTime);
            if (Input.GetKey(KeyCode.E)) bladeDepth = Mathf.Min(0.1f, bladeDepth + 0.02f * Time.deltaTime);

            UpdateCamera();
            Ray ray = _camera.ScreenPointToRay(Input.mousePosition);
            if (tool == Tool.Scalpel)
            {
                UpdateScalpel(ray);
            }
            else
            {
                UpdateForceps(ray);
            }
        }

        private void UpdateCamera()
        {
            if (Input.GetMouseButton(1))
            {
                _yaw += Input.GetAxis("Mouse X") * 4.0f;
                _pitch = Mathf.Clamp(_pitch - Input.GetAxis("Mouse Y") * 4.0f, 5.0f, 85.0f);
            }
            orbitDistance = Mathf.Clamp(orbitDistance * (1.0f - Input.mouseScrollDelta.y * 0.1f), 0.15f, 2.0f);
            Quaternion rotation = Quaternion.Euler(_pitch, _yaw, 0.0f);
            _camera.transform.SetPositionAndRotation(orbitTarget + rotation * new Vector3(0.0f, 0.0f, -orbitDistance), rotation);
        }

        private void UpdateScalpel(Ray ray)
        {
            if (scalpel == null)
            {
                return;
            }
            if (Input.GetMouseButtonDown(0) && PickSurface(ray, out Vector3 hit, out _))
            {
                _bladeHeight = hit.y;
                _bladeActive = true;
                _severedThisStroke = 0;
                PlaceBlade(hit);
                scalpel.ResetSweep();
            }
            if (Input.GetMouseButtonUp(0))
            {
                _bladeActive = false;
            }
            scalpel.cutting = _bladeActive;
            if (_bladeActive && new Plane(Vector3.up, new Vector3(0.0f, _bladeHeight, 0.0f)).Raycast(ray, out float enter))
            {
                PlaceBlade(ray.GetPoint(enter));
            }
            _severedThisStroke += scalpel.LastSeveredEdges;
        }

        private void PlaceBlade(Vector3 surfacePoint)
        {
            scalpel.transform.SetPositionAndRotation(surfacePoint + Vector3.up * 0.01f, Quaternion.identity);
            scalpel.edgeStart = Vector3.zero;
            scalpel.edgeEnd = Vector3.down * (0.01f + bladeDepth);
        }

        private void UpdateForceps(Ray ray)
        {
            if (Input.GetMouseButtonDown(0))
            {
                float best = float.MaxValue;
                foreach (ApxSoftBody body in ApxSoftBody.Active)
                {
                    if (body.BeginGrab(ray, grabRadius, out Vector3 point))
                    {
                        float d = Vector3.Distance(ray.origin, point);
                        if (d < best)
                        {
                            if (_grabbed != null) _grabbed.EndGrab();
                            _grabbed = body;
                            best = d;
                            _grabPlane = new Plane(-_camera.transform.forward, point);
                        }
                        else
                        {
                            body.EndGrab();
                        }
                    }
                }
            }
            if (_grabbed != null && Input.GetMouseButton(0) && _grabPlane.Raycast(ray, out float enter))
            {
                _grabbed.MoveGrab(ray.GetPoint(enter));
            }
            if (Input.GetMouseButtonUp(0) && _grabbed != null)
            {
                _grabbed.EndGrab();
                _grabbed = null;
            }
        }

        private static bool PickSurface(Ray ray, out Vector3 point, out ApxSoftBody hitBody)
        {
            point = default;
            hitBody = null;
            float best = float.MaxValue;
            foreach (ApxSoftBody body in ApxSoftBody.Active)
            {
                SoftBodySimulation sim = body.Simulation;
                if (sim == null)
                {
                    continue;
                }
                int node = sim.PickNode(ray.origin, ray.direction, sim.CellSize);
                if (node < 0)
                {
                    continue;
                }
                Vector3 p = sim.Positions[node];
                float d = Vector3.Distance(ray.origin, p);
                if (d < best)
                {
                    best = d;
                    point = p;
                    hitBody = body;
                }
            }
            return hitBody != null;
        }

        private void OnGUI()
        {
            GUILayout.BeginArea(new Rect(10, 10, 420, 400), GUI.skin.box);
            GUILayout.Label($"Tool: {tool}   [1] scalpel  [2] forceps   [R] reload");
            GUILayout.Label("LMB: cut / grab    RMB drag: orbit    wheel: zoom");
            GUILayout.Label($"Blade depth: {bladeDepth * 1000.0f:0} mm   [Q]/[E]   severed this stroke: {_severedThisStroke}");
            foreach (ApxSoftBody body in ApxSoftBody.Active)
            {
                SoftBodySimulation sim = body.Simulation;
                if (sim == null)
                {
                    continue;
                }
                GUILayout.Label($"{body.name}: cells {sim.CellCount}  copies {sim.CopyCount}  nodes {sim.NodeCount}  " +
                                $"step {sim.LastStepMilliseconds:0.0} ms  topo {sim.LastTopologyMilliseconds:0.0} ms");
            }
            GUILayout.EndArea();
        }
    }
}
