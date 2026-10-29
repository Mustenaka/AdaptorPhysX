using System;
using System.Collections.Generic;
using UnityEngine;

namespace AdaptorPhysX
{
    /// <summary>Edges a stroke severed in a body (<see cref="ApxSoftBody.Severed"/>).</summary>
    public struct ApxCutEvent
    {
        /// <summary>The stroke of <see cref="ApxSoftBody.Cut"/>; 0 for a cut that is a stroke of its own.</summary>
        public int Stroke;
        public int SeveredEdges;
    }

    /// <summary>A material piece: its id (<see cref="SoftBodySimulation.PieceId"/>, valid until the next topology change) and real nodes.</summary>
    public struct ApxPiece
    {
        public int Id;
        public int RealNodes;
    }

    /// <summary>The number of pieces of a body changed (<see cref="ApxSoftBody.PiecesChanged"/>).</summary>
    public struct ApxPiecesEvent
    {
        public int PreviousCount;
        /// <summary>All pieces now, largest first.</summary>
        public ApxPiece[] Pieces;
    }

    /// <summary>A collider began or ended touching a body (<see cref="ApxSoftBody.ContactBegan"/>).</summary>
    public struct ApxContactEvent
    {
        public Collider Collider;
        /// <summary>Force on the collider (HAP-1) in the first step of the contact, or in the last one for an end.</summary>
        public ApxWrench Wrench;
    }

    /// <summary>Points of a dynamic attachment let go above its break force (<see cref="ApxSoftBody.PointsReleased"/>).</summary>
    public struct ApxReleaseEvent
    {
        public int Attachment;
        /// <summary>Points that let go in the step, and points still held.</summary>
        public int Released;
        public int Held;
    }

    /// <summary>
    /// Events (TODO ECO-10), raised on the main thread once a step has completed: in LateUpdate
    /// (<see cref="UpdateRender"/>) or, for a step completed otherwise, at the start of the next
    /// <see cref="Simulate"/>. Sleeping and waking are raised in <see cref="Simulate"/>, as a sleeping body does not step.
    /// </summary>
    public sealed partial class ApxSoftBody
    {
        /// <summary>A stroke severed edges; raised after the next step, once per stroke and step.</summary>
        public event Action<ApxSoftBody, ApxCutEvent> Severed;
        /// <summary>The number of material pieces changed (a cut separated a part, or a state was restored).</summary>
        public event Action<ApxSoftBody, ApxPiecesEvent> PiecesChanged
        {
            add
            {
                if (_piecesChanged == null && Simulation != null && _pieceCount < 0)
                {
                    _pieceCount = Simulation.CountPieces();
                    _pieceTopology = Simulation.TopologyVersion;
                }
                _piecesChanged += value;
            }
            remove => _piecesChanged -= value;
        }
        /// <summary>A collider touched the body in a step after one it did not.</summary>
        public event Action<ApxSoftBody, ApxContactEvent> ContactBegan;
        /// <summary>A collider that touched the body in a step did not in the next.</summary>
        public event Action<ApxSoftBody, ApxContactEvent> ContactEnded;
        /// <summary>Points of a dynamic attachment (<see cref="AddAttachment"/>) let go in a step.</summary>
        public event Action<ApxSoftBody, ApxReleaseEvent> PointsReleased;
        /// <summary>The body fell asleep (true) or woke up (false).</summary>
        public event Action<ApxSoftBody, bool> SleepChanged;

        private Action<ApxSoftBody, ApxPiecesEvent> _piecesChanged;
        /// <summary>Step whose events were raised.</summary>
        private int _eventStep = -1;
        /// <summary>Strokes that severed edges since the last events, in order.</summary>
        private readonly List<ApxCutEvent> _pendingCuts = new List<ApxCutEvent>();
        /// <summary>Piece count at topology version <see cref="_pieceTopology"/>; -1 when not counted.</summary>
        private int _pieceCount = -1;
        private int _pieceTopology = -1;
        /// <summary>Colliders in contact in the step of the last events, with their wrenches.</summary>
        private readonly Dictionary<Collider, ApxWrench> _contacts = new Dictionary<Collider, ApxWrench>();
        private readonly List<Collider> _endedContacts = new List<Collider>();
        /// <summary>Points each dynamic attachment held at the last events.</summary>
        private readonly SortedDictionary<int, int> _heldPoints = new SortedDictionary<int, int>();
        private readonly List<int> _attachmentHandles = new List<int>();
        private bool _sleeping;

        /// <summary>Raises the events of the last completed step, once.</summary>
        private void RaiseStepEvents()
        {
            if (Simulation == null)
            {
                return;
            }
            Simulation.CompleteStep();
            if (_eventStep == Simulation.StepCount)
            {
                return;
            }
            _eventStep = Simulation.StepCount;
            RaiseCuts();
            RaisePieces();
            RaiseContacts();
            _attachmentHandles.Clear();
            _attachmentHandles.AddRange(_heldPoints.Keys);
            foreach (int attachment in _attachmentHandles)
            {
                RaiseReleases(attachment);
            }
        }

        private void RaiseCuts()
        {
            foreach (ApxCutEvent cut in _pendingCuts)
            {
                Severed?.Invoke(this, cut);
            }
            _pendingCuts.Clear();
        }

        /// <summary>Counts the pieces after a topology change; only while someone listens.</summary>
        private void RaisePieces()
        {
            if (_pieceTopology == Simulation.TopologyVersion)
            {
                return;
            }
            if (_piecesChanged == null)
            {
                _pieceCount = -1;
                return;
            }
            _pieceTopology = Simulation.TopologyVersion;
            List<ApxPiece> pieces = Simulation.Pieces();
            int previous = _pieceCount;
            _pieceCount = pieces.Count;
            if (previous >= 0 && previous != pieces.Count)
            {
                _piecesChanged.Invoke(this, new ApxPiecesEvent { PreviousCount = previous, Pieces = pieces.ToArray() });
            }
        }

        private void RaiseContacts()
        {
            IReadOnlyDictionary<Collider, ApxWrench> wrenches = ColliderWrenches;
            _endedContacts.Clear();
            foreach (Collider collider in _contacts.Keys)
            {
                if (!wrenches.ContainsKey(collider))
                {
                    _endedContacts.Add(collider);
                }
            }
            foreach (Collider collider in _endedContacts)
            {
                ApxWrench last = _contacts[collider];
                _contacts.Remove(collider);
                ContactEnded?.Invoke(this, new ApxContactEvent { Collider = collider, Wrench = last });
            }
            foreach (KeyValuePair<Collider, ApxWrench> pair in wrenches)
            {
                bool began = !_contacts.ContainsKey(pair.Key);
                _contacts[pair.Key] = pair.Value;
                if (began)
                {
                    ContactBegan?.Invoke(this, new ApxContactEvent { Collider = pair.Key, Wrench = pair.Value });
                }
            }
        }

        /// <summary>Raises the points an attachment let go since the last events.</summary>
        private void RaiseReleases(int attachment)
        {
            if (!_heldPoints.TryGetValue(attachment, out int previous))
            {
                return;
            }
            int held = Simulation.HeldPoints(attachment);
            _heldPoints[attachment] = held;
            if (held < previous)
            {
                PointsReleased?.Invoke(this, new ApxReleaseEvent { Attachment = attachment, Released = previous - held, Held = held });
            }
        }

        /// <summary>Raises a change of <see cref="IsSleeping"/>; called by <see cref="Simulate"/>.</summary>
        private void RaiseSleep()
        {
            bool sleeping = IsSleeping;
            if (sleeping != _sleeping)
            {
                _sleeping = sleeping;
                SleepChanged?.Invoke(this, sleeping);
            }
        }

        /// <summary>Adds the edges a stroke severed to the events of the next step.</summary>
        private void RecordCut(int stroke, int severed)
        {
            for (int i = 0; i < _pendingCuts.Count; i++)
            {
                if (stroke != 0 && _pendingCuts[i].Stroke == stroke)
                {
                    ApxCutEvent cut = _pendingCuts[i];
                    cut.SeveredEdges += severed;
                    _pendingCuts[i] = cut;
                    return;
                }
            }
            _pendingCuts.Add(new ApxCutEvent { Stroke = stroke, SeveredEdges = severed });
        }

        /// <summary>Starts the events afresh for a new simulation.</summary>
        private void ResetEvents()
        {
            _eventStep = -1;
            _pendingCuts.Clear();
            _pieceCount = Simulation != null ? Simulation.CountPieces() : -1;
            _pieceTopology = Simulation != null ? Simulation.TopologyVersion : -1;
            _contacts.Clear();
            _heldPoints.Clear();
            _sleeping = false;
        }
    }
}
