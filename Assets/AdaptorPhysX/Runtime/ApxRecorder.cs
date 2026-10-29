using System;
using System.Collections.Generic;
using UnityEngine;

namespace AdaptorPhysX
{
    /// <summary>
    /// Kind of a recorded operation (<see cref="ApxOperation"/>), with the fields it uses. Recordings store the kind as its
    /// number: new kinds go at the end.
    /// </summary>
    public enum ApxOperationKind
    {
        /// <summary><see cref="ApxSoftBody.Cut"/>: edge (A, B) to (C, D), stroke I.</summary>
        Cut,
        /// <summary><see cref="ApxSoftBody.EndStroke"/>: stroke I.</summary>
        EndStroke,
        /// <summary><see cref="ApxSoftBody.WakeUp"/>.</summary>
        WakeUp,
        /// <summary>BeginGrab(Ray, float, out Vector3): ray origin A and direction B, radius X.</summary>
        BeginGrab,
        /// <summary>BeginGrab(Ray, float, Quaternion, out Vector3): ray origin A and direction B, radius X, Rotation.</summary>
        BeginFrameGrab,
        /// <summary>MoveGrab(Vector3): target A.</summary>
        MoveGrab,
        /// <summary>MoveGrab(int, Vector3, Quaternion): grab I, position A, Rotation.</summary>
        MoveFrameGrab,
        /// <summary>EndGrab: grab I.</summary>
        EndGrab,
        /// <summary>AddMaterialPoint: position A.</summary>
        AddMaterialPoint,
        /// <summary>RemoveMaterialPoint: point I.</summary>
        RemoveMaterialPoint,
        /// <summary>AddAttachment: target instrument J, Points, compliance X, break force Y (negative: none).</summary>
        AddAttachment,
        /// <summary>RemoveAttachment: attachment I.</summary>
        RemoveAttachment,
        /// <summary>SetAttachmentBreakForce: attachment I, break force X.</summary>
        SetAttachmentBreakForce,
        /// <summary>AddStitch: points I and J, speed X, compliance Y.</summary>
        AddStitch,
        /// <summary>SetStitchTarget: stitch I, length X, speed Y.</summary>
        SetStitchTarget,
        /// <summary>RemoveStitch: stitch I.</summary>
        RemoveStitch,
        /// <summary><see cref="ApxSoftBody.Rebuild"/>.</summary>
        Rebuild,
        /// <summary><see cref="ApxScalpel.ResetSweep"/> of the instrument.</summary>
        ResetSweep,
        /// <summary><see cref="ApxGrasper.Release"/> of the instrument.</summary>
        ReleaseGrasp,
    }

    /// <summary>
    /// An operation made before fixed step <see cref="Tick"/> on a body (<see cref="Target"/>: index in
    /// <see cref="ApxRecorder.bodies"/>) or, for <see cref="ApxOperationKind.ResetSweep"/> and
    /// <see cref="ApxOperationKind.ReleaseGrasp"/>, on an instrument (index in <see cref="ApxRecorder.instruments"/>).
    /// </summary>
    [Serializable]
    public struct ApxOperation
    {
        public int Tick;
        public ApxOperationKind Kind;
        public int Target;
        public Vector3 A, B, C, D;
        public Quaternion Rotation;
        public float X, Y;
        public int I, J;
        public int[] Points;
    }

    /// <summary>Local position and rotation of an instrument from fixed step <see cref="Tick"/> on.</summary>
    [Serializable]
    public struct ApxPoseSample
    {
        public int Tick;
        public int Instrument;
        public Vector3 Position;
        public Quaternion Rotation;
    }

    /// <summary>
    /// Settings of an instrument from fixed step <see cref="Tick"/> on: whether its object is active, and those of its
    /// <see cref="ApxScalpel"/> and <see cref="ApxGrasper"/>.
    /// </summary>
    [Serializable]
    public struct ApxInstrumentSample
    {
        public int Tick;
        public int Instrument;
        public bool Active;
        public bool Cutting;
        public Vector3 EdgeStart, EdgeEnd;
        public float Gap, GraspGap;
        public Vector3 JawSize;
        public float Friction, Compliance;

        /// <summary>Whether the settings are the same, bit for bit.</summary>
        internal bool SameSettings(in ApxInstrumentSample other) =>
            Active == other.Active && Cutting == other.Cutting && EdgeStart.Equals(other.EdgeStart) &&
            EdgeEnd.Equals(other.EdgeEnd) && Gap.Equals(other.Gap) && GraspGap.Equals(other.GraspGap) &&
            JawSize.Equals(other.JawSize) && Friction.Equals(other.Friction) && Compliance.Equals(other.Compliance);
    }

    /// <summary>A fixed step in which a body skipped its step (<see cref="ApxSoftBody.maxStepsPerFrame"/>).</summary>
    [Serializable]
    public struct ApxStepSkip
    {
        public int Tick;
        public int Body;
    }

    /// <summary>Node positions hash (<see cref="SoftBodySimulation.PositionsHash"/>) of a body before fixed step <see cref="Tick"/>.</summary>
    [Serializable]
    public struct ApxStateCheck
    {
        public int Tick;
        public int Body;
        public string Hash;
    }

    /// <summary>The inputs of a procedure, recorded by <see cref="ApxRecorder"/>.</summary>
    [Serializable]
    public sealed class ApxRecording
    {
        /// <summary>Time.fixedDeltaTime while recording; the replay in Play mode sets it.</summary>
        public float FixedDeltaTime;
        /// <summary>Fixed steps recorded.</summary>
        public int Steps;
        /// <summary>Scalpel stroke counter at the start: the replay's strokes get the same numbers.</summary>
        public int FirstStroke;
        /// <summary>Names of the bodies and instruments, to check the replay's scene.</summary>
        public List<string> Bodies = new List<string>();
        public List<string> Instruments = new List<string>();
        /// <summary>Instrument poses and settings, each when it changed.</summary>
        public List<ApxPoseSample> Poses = new List<ApxPoseSample>();
        public List<ApxInstrumentSample> Settings = new List<ApxInstrumentSample>();
        public List<ApxOperation> Operations = new List<ApxOperation>();
        public List<ApxStepSkip> Skips = new List<ApxStepSkip>();
        /// <summary>Hashes every <see cref="ApxRecorder.checkInterval"/> steps and after the last step (Tick = Steps).</summary>
        public List<ApxStateCheck> Checks = new List<ApxStateCheck>();

        public string ToJson() => JsonUtility.ToJson(this);

        public static ApxRecording FromJson(string json) => JsonUtility.FromJson<ApxRecording>(json);
    }

    /// <summary>
    /// Records the inputs of a procedure and replays them (TODO MED-14). <see cref="Step"/> runs at the start of every fixed
    /// step (FixedUpdate, before the instruments and the bodies). While recording it stores the local pose of every
    /// instrument and the settings of its <see cref="ApxScalpel"/> and <see cref="ApxGrasper"/> when they changed, and the
    /// bodies record the operations made on them (cuts, grabs, material points, attachments, stitches) and the fixed steps
    /// in which they skipped their step. Operations that instruments make (the scalpel's cuts, the grasper's attachments),
    /// that other operations make, and that event handlers make are not recorded: in the replay the instruments and the
    /// handlers make them again. The replay starts from the scene's initial state: the same scene, before any body stepped.
    /// Each fixed step it sets the instruments' poses and settings, makes the operations recorded before that step, and has
    /// the bodies skip the steps they skipped; the input scripts (e.g. <c>ApxToolbox</c>) must be off. Every
    /// <see cref="checkInterval"/> steps and at the end it compares the node positions hashes of the bodies with the
    /// recorded ones (<see cref="DivergedTick"/>).
    /// </summary>
    [DefaultExecutionOrder(-100)]
    public sealed class ApxRecorder : MonoBehaviour
    {
        [Tooltip("Bodies whose operations are recorded, in the order the recording refers to them; empty: the bodies of " +
                 "the scene by hierarchy path.")]
        public ApxSoftBody[] bodies = new ApxSoftBody[0];
        [Tooltip("Instruments whose pose is recorded: scalpels, graspers, collider instruments, attachment targets, moving pins.")]
        public Transform[] instruments = new Transform[0];
        [Tooltip("Fixed steps between checks of the bodies' node positions; 0 checks at the end only.")]
        [Min(0)] public int checkInterval = 60;
        [Tooltip("Start recording on Start.")]
        public bool recordOnStart;

        public bool IsRecording => _mode == Mode.Recording;
        public bool IsReplaying => _mode == Mode.Replaying;
        /// <summary>The recording being made or replayed, or the last one.</summary>
        public ApxRecording Recording { get; private set; }
        /// <summary>Index of the current fixed step since recording or replay began.</summary>
        public int Tick => _tick;
        /// <summary>Fixed step before which the replay's node positions first differed from the recording's; -1 while equal.</summary>
        public int DivergedTick { get; private set; } = -1;
        public bool Diverged => DivergedTick >= 0;
        /// <summary>Hashes compared in the replay.</summary>
        public int ChecksCompared { get; private set; }

        /// <summary>Raised after the last recorded step was replayed and checked.</summary>
        public event Action<ApxRecorder> ReplayEnded;

        private enum Mode
        {
            Idle,
            Recording,
            Replaying,
        }

        private Mode _mode;
        private int _tick = -1;
        private ApxSoftBody[] _bodies = new ApxSoftBody[0];
        private ApxPoseSample[] _lastPoses;
        private ApxInstrumentSample[] _lastSettings;
        private bool[] _sampled;
        private int _poseCursor;
        private int _settingCursor;
        private int _operationCursor;
        private int _checkCursor;
        private readonly HashSet<(int, int)> _skips = new HashSet<(int, int)>();
        private int _strokesBefore;

        /// <summary>Recorders recording, for the instruments' operations.</summary>
        private static readonly List<ApxRecorder> Recorders = new List<ApxRecorder>();
        /// <summary>Operations in progress: those they make are not recorded.</summary>
        private static int s_unrecorded;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        private static void ResetStatics()
        {
            Recorders.Clear();
            s_unrecorded = 0;
        }

        private void Start()
        {
            if (recordOnStart)
            {
                StartRecording();
            }
        }

        private void FixedUpdate() => Step();

        private void OnDestroy() => End();

        /// <summary>
        /// Starts a recording. The bodies must not have stepped yet: the replay starts from the scene's initial state.
        /// </summary>
        public void StartRecording()
        {
            End();
            ResolveBodies();
            Recording = new ApxRecording { FixedDeltaTime = Time.fixedDeltaTime, FirstStroke = ApxScalpel.Strokes };
            foreach (ApxSoftBody body in _bodies)
            {
                Recording.Bodies.Add(body.name);
            }
            foreach (Transform instrument in instruments)
            {
                Recording.Instruments.Add(instrument != null ? instrument.name : "");
            }
            WarnIfStepped("recording");
            Begin(Mode.Recording);
            _lastPoses = new ApxPoseSample[instruments.Length];
            _lastSettings = new ApxInstrumentSample[instruments.Length];
            _sampled = new bool[instruments.Length];
            Recorders.Add(this);
        }

        /// <summary>Ends the recording after the last fixed step, with the bodies' hashes; returns it.</summary>
        public ApxRecording StopRecording()
        {
            if (!IsRecording)
            {
                return null;
            }
            Recording.Steps = _tick + 1;
            AddChecks(Recording.Steps);
            End();
            return Recording;
        }

        /// <summary>
        /// Replays a recording from the next fixed step on. The scene must be the recording's, before any body stepped, with
        /// the same <see cref="bodies"/> and <see cref="instruments"/>.
        /// </summary>
        public void Replay(ApxRecording recording)
        {
            End();
            ResolveBodies();
            if (recording.Bodies.Count != _bodies.Length || recording.Instruments.Count != instruments.Length)
            {
                Debug.LogError($"{name}: the recording has {recording.Bodies.Count} bodies and {recording.Instruments.Count} " +
                               $"instruments, the scene {_bodies.Length} and {instruments.Length}; not replayed.", this);
                return;
            }
            for (int b = 0; b < _bodies.Length; b++)
            {
                if (recording.Bodies[b] != _bodies[b].name)
                {
                    Debug.LogWarning($"{name}: body {b} is {_bodies[b].name}, recorded as {recording.Bodies[b]}.", this);
                }
            }
            WarnIfStepped("replay");
            Recording = recording;
            _poseCursor = _settingCursor = _operationCursor = _checkCursor = 0;
            _skips.Clear();
            foreach (ApxStepSkip skip in recording.Skips)
            {
                _skips.Add((skip.Tick, skip.Body));
            }
            DivergedTick = -1;
            ChecksCompared = 0;
            _strokesBefore = ApxScalpel.Strokes;
            ApxScalpel.Strokes = recording.FirstStroke;
            if (Application.isPlaying && recording.FixedDeltaTime > 0.0f)
            {
                Time.fixedDeltaTime = recording.FixedDeltaTime;
            }
            Begin(Mode.Replaying);
        }

        /// <summary>
        /// Records or replays the start of a fixed step: before the instruments (<see cref="ApxScalpel.Sweep"/>,
        /// <see cref="ApxGrasper.Step"/>) and the bodies. Called by FixedUpdate; edit-mode code calls it itself.
        /// </summary>
        public void Step()
        {
            if (_mode == Mode.Idle)
            {
                return;
            }
            _tick++;
            if (IsRecording)
            {
                if (checkInterval > 0 && _tick > 0 && _tick % checkInterval == 0)
                {
                    AddChecks(_tick);
                }
                SampleInstruments();
                // The colliders' broad phase follows the instruments in recording and replay alike.
                Physics.SyncTransforms();
                return;
            }
            if (_tick == Recording.Steps)
            {
                MakeOperations();
                CompareChecks();
                ApxScalpel.Strokes = Math.Max(_strokesBefore, ApxScalpel.Strokes);
                End();
                ReplayEnded?.Invoke(this);
                return;
            }
            ApplyInstruments();
            Physics.SyncTransforms();
            MakeOperations();
            CompareChecks();
        }

        /// <summary>Records an operation on a body: called by the body's operations, before they act.</summary>
        internal void Record(ApxSoftBody body, in ApxOperation operation, IReadOnlyList<int> points, Transform target)
        {
            int index = Array.IndexOf(_bodies, body);
            if (!IsRecording || s_unrecorded > 0 || index < 0)
            {
                return;
            }
            ApxOperation recorded = operation;
            recorded.Tick = _tick + 1;
            recorded.Target = index;
            if (points != null)
            {
                recorded.Points = new int[points.Count];
                for (int p = 0; p < points.Count; p++)
                {
                    recorded.Points[p] = points[p];
                }
            }
            if (target != null)
            {
                recorded.J = Array.IndexOf(instruments, target);
                if (recorded.J < 0)
                {
                    Debug.LogWarning($"{name}: the target {target.name} of an attachment of {body.name} is not an instrument; " +
                                     "the replay cannot make the attachment.", this);
                }
            }
            List<ApxOperation> operations = Recording.Operations;
            int last = operations.Count - 1;
            // Moving a grab only sets its target: of several moves before a step, the last one counts.
            if (last >= 0 && (recorded.Kind == ApxOperationKind.MoveGrab || recorded.Kind == ApxOperationKind.MoveFrameGrab) &&
                operations[last].Kind == recorded.Kind && operations[last].Tick == recorded.Tick &&
                operations[last].Target == recorded.Target && operations[last].I == recorded.I)
            {
                operations[last] = recorded;
                return;
            }
            operations.Add(recorded);
        }

        /// <summary>Records an operation on an instrument with every recorder that lists it.</summary>
        internal static void Record(Component instrument, ApxOperationKind kind)
        {
            if (s_unrecorded > 0)
            {
                return;
            }
            foreach (ApxRecorder recorder in Recorders)
            {
                int index = Array.IndexOf(recorder.instruments, instrument.transform);
                if (index >= 0)
                {
                    recorder.Recording.Operations.Add(new ApxOperation { Tick = recorder._tick + 1, Kind = kind, Target = index });
                }
            }
        }

        /// <summary>
        /// Called by a body's FixedUpdate with whether it steps: records a skipped step, and in the replay skips the steps the
        /// body skipped in the recording and only those.
        /// </summary>
        internal void DecideStep(ApxSoftBody body, ref bool step)
        {
            int index = Array.IndexOf(_bodies, body);
            if (index < 0)
            {
                return;
            }
            if (IsRecording && !step)
            {
                Recording.Skips.Add(new ApxStepSkip { Tick = _tick, Body = index });
            }
            else if (IsReplaying)
            {
                step = !_skips.Contains((_tick, index));
            }
        }

        /// <summary>Warns that a body cannot repeat a recording it was not part of from its start.</summary>
        internal void Unrecorded(ApxSoftBody body, string operation)
        {
            if (IsRecording && s_unrecorded == 0)
            {
                Debug.LogWarning($"{name}: {operation} of {body.name} is not recorded; the replay will differ.", this);
            }
        }

        /// <summary>
        /// Keeps the operations made while it is open from being recorded: those of an operation in progress, of an
        /// instrument and of event handlers.
        /// </summary>
        internal readonly struct Scope : IDisposable
        {
            internal static Scope Begin()
            {
                s_unrecorded++;
                return default;
            }

            public void Dispose() => s_unrecorded--;
        }

        private void Begin(Mode mode)
        {
            _mode = mode;
            _tick = -1;
            foreach (ApxSoftBody body in _bodies)
            {
                body.Recorder = this;
            }
        }

        private void End()
        {
            if (_mode == Mode.Idle)
            {
                return;
            }
            _mode = Mode.Idle;
            Recorders.Remove(this);
            foreach (ApxSoftBody body in _bodies)
            {
                if (body != null && body.Recorder == this)
                {
                    body.Recorder = null;
                }
            }
        }

        private void ResolveBodies()
        {
            if (bodies.Length > 0)
            {
                _bodies = (ApxSoftBody[])bodies.Clone();
                return;
            }
            var found = new List<ApxSoftBody>(FindObjectsOfType<ApxSoftBody>());
            found.Sort((x, y) => string.CompareOrdinal(PathOf(x.transform), PathOf(y.transform)));
            _bodies = found.ToArray();
        }

        private static string PathOf(Transform t) =>
            t.parent == null ? $"{t.GetSiblingIndex():D4}{t.name}" : $"{PathOf(t.parent)}/{t.GetSiblingIndex():D4}{t.name}";

        private void WarnIfStepped(string what)
        {
            foreach (ApxSoftBody body in _bodies)
            {
                if (body.Simulation != null && body.Simulation.StepCount > 0)
                {
                    Debug.LogWarning($"{name}: {body.name} has stepped before the {what}; the replay starts from the scene's " +
                                     "initial state and will differ.", this);
                }
            }
        }

        private void AddChecks(int tick)
        {
            for (int b = 0; b < _bodies.Length; b++)
            {
                Recording.Checks.Add(new ApxStateCheck { Tick = tick, Body = b, Hash = HashOf(_bodies[b]) });
            }
        }

        private static string HashOf(ApxSoftBody body) =>
            body != null && body.Simulation != null ? body.Simulation.PositionsHash().ToString("x16") : "";

        private void SampleInstruments()
        {
            for (int i = 0; i < instruments.Length; i++)
            {
                Transform t = instruments[i];
                if (t == null)
                {
                    continue;
                }
                var pose = new ApxPoseSample { Tick = _tick, Instrument = i, Position = t.localPosition, Rotation = t.localRotation };
                if (!_sampled[i] || !pose.Position.Equals(_lastPoses[i].Position) || !pose.Rotation.Equals(_lastPoses[i].Rotation))
                {
                    Recording.Poses.Add(pose);
                    // Unity normalizes a rotation when it is set, and a rotation read back and set again can change in its last
                    // bits: the recording sets the pose it stores, as the replay will, so both step with the same rotation.
                    // The next steps compare with what the Transform holds then.
                    t.localPosition = pose.Position;
                    t.localRotation = pose.Rotation;
                    pose.Position = t.localPosition;
                    pose.Rotation = t.localRotation;
                    _lastPoses[i] = pose;
                }
                ApxInstrumentSample settings = SettingsOf(t);
                settings.Tick = _tick;
                settings.Instrument = i;
                if (!_sampled[i] || !settings.SameSettings(_lastSettings[i]))
                {
                    Recording.Settings.Add(settings);
                    _lastSettings[i] = settings;
                }
                _sampled[i] = true;
            }
        }

        private static ApxInstrumentSample SettingsOf(Transform t)
        {
            var settings = new ApxInstrumentSample { Active = t.gameObject.activeSelf };
            if (t.TryGetComponent(out ApxScalpel scalpel))
            {
                settings.Cutting = scalpel.cutting;
                settings.EdgeStart = scalpel.edgeStart;
                settings.EdgeEnd = scalpel.edgeEnd;
            }
            if (t.TryGetComponent(out ApxGrasper grasper))
            {
                settings.Gap = grasper.gap;
                settings.GraspGap = grasper.graspGap;
                settings.JawSize = grasper.jawSize;
                settings.Friction = grasper.friction;
                settings.Compliance = grasper.compliance;
            }
            return settings;
        }

        private void ApplyInstruments()
        {
            List<ApxPoseSample> poses = Recording.Poses;
            for (; _poseCursor < poses.Count && poses[_poseCursor].Tick <= _tick; _poseCursor++)
            {
                ApxPoseSample pose = poses[_poseCursor];
                Transform t = instruments[pose.Instrument];
                if (t != null)
                {
                    t.localPosition = pose.Position;
                    t.localRotation = pose.Rotation;
                }
            }
            List<ApxInstrumentSample> settings = Recording.Settings;
            for (; _settingCursor < settings.Count && settings[_settingCursor].Tick <= _tick; _settingCursor++)
            {
                ApxInstrumentSample s = settings[_settingCursor];
                Transform t = instruments[s.Instrument];
                if (t == null)
                {
                    continue;
                }
                if (t.gameObject.activeSelf != s.Active)
                {
                    t.gameObject.SetActive(s.Active);
                }
                if (t.TryGetComponent(out ApxScalpel scalpel))
                {
                    scalpel.cutting = s.Cutting;
                    scalpel.edgeStart = s.EdgeStart;
                    scalpel.edgeEnd = s.EdgeEnd;
                }
                if (t.TryGetComponent(out ApxGrasper grasper))
                {
                    grasper.gap = s.Gap;
                    grasper.graspGap = s.GraspGap;
                    grasper.jawSize = s.JawSize;
                    grasper.friction = s.Friction;
                    grasper.compliance = s.Compliance;
                }
            }
        }

        private void MakeOperations()
        {
            List<ApxOperation> operations = Recording.Operations;
            for (; _operationCursor < operations.Count && operations[_operationCursor].Tick <= _tick; _operationCursor++)
            {
                Make(operations[_operationCursor]);
            }
        }

        private void Make(in ApxOperation op)
        {
            if (op.Kind == ApxOperationKind.ResetSweep || op.Kind == ApxOperationKind.ReleaseGrasp)
            {
                Transform instrument = instruments[op.Target];
                if (op.Kind == ApxOperationKind.ResetSweep && instrument != null && instrument.TryGetComponent(out ApxScalpel scalpel))
                {
                    scalpel.ResetSweep();
                }
                else if (instrument != null && instrument.TryGetComponent(out ApxGrasper grasper))
                {
                    grasper.Release();
                }
                return;
            }
            ApxSoftBody body = _bodies[op.Target];
            switch (op.Kind)
            {
                case ApxOperationKind.Cut:
                    body.Cut(op.A, op.B, op.C, op.D, op.I);
                    break;
                case ApxOperationKind.EndStroke:
                    body.EndStroke(op.I);
                    break;
                case ApxOperationKind.WakeUp:
                    body.WakeUp();
                    break;
                case ApxOperationKind.BeginGrab:
                    body.BeginGrab(op.A, op.B, op.X, out _);
                    break;
                case ApxOperationKind.BeginFrameGrab:
                    body.BeginGrab(op.A, op.B, op.X, op.Rotation, out _);
                    break;
                case ApxOperationKind.MoveGrab:
                    body.MoveGrab(op.A);
                    break;
                case ApxOperationKind.MoveFrameGrab:
                    body.MoveGrab(op.I, op.A, op.Rotation);
                    break;
                case ApxOperationKind.EndGrab:
                    body.EndGrab(op.I);
                    break;
                case ApxOperationKind.AddMaterialPoint:
                    body.AddMaterialPoint(op.A);
                    break;
                case ApxOperationKind.RemoveMaterialPoint:
                    body.RemoveMaterialPoint(op.I);
                    break;
                case ApxOperationKind.AddAttachment:
                    if (op.J < 0 || instruments[op.J] == null)
                    {
                        Debug.LogError($"{name}: an attachment of {body.name} at step {op.Tick} has no target instrument.", this);
                        break;
                    }
                    body.AddAttachment(instruments[op.J], op.Points, op.X, op.Y < 0.0f ? float.PositiveInfinity : op.Y);
                    break;
                case ApxOperationKind.RemoveAttachment:
                    body.RemoveAttachment(op.I);
                    break;
                case ApxOperationKind.SetAttachmentBreakForce:
                    body.SetAttachmentBreakForce(op.I, op.X);
                    break;
                case ApxOperationKind.AddStitch:
                    body.AddStitch(op.I, op.J, op.X, op.Y);
                    break;
                case ApxOperationKind.SetStitchTarget:
                    body.SetStitchTarget(op.I, op.X, op.Y);
                    break;
                case ApxOperationKind.RemoveStitch:
                    body.RemoveStitch(op.I);
                    break;
                case ApxOperationKind.Rebuild:
                    body.Rebuild();
                    break;
            }
        }

        private void CompareChecks()
        {
            List<ApxStateCheck> checks = Recording.Checks;
            for (; _checkCursor < checks.Count && checks[_checkCursor].Tick <= _tick; _checkCursor++)
            {
                ApxStateCheck check = checks[_checkCursor];
                if (check.Tick != _tick || check.Body >= _bodies.Length)
                {
                    continue;
                }
                ChecksCompared++;
                string hash = HashOf(_bodies[check.Body]);
                if (hash != check.Hash && DivergedTick < 0)
                {
                    DivergedTick = _tick;
                    Debug.LogWarning($"{name}: the replay differs from the recording before step {_tick}: {_bodies[check.Body].name} " +
                                     $"has positions hash {hash}, recorded {check.Hash}.", this);
                }
            }
        }
    }
}
