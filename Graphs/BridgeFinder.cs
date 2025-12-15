using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace g3
{
    public record struct Bridge
    {
        public int edgeIdx;
        public int subtreeSize;
        public int rootSide;
        public int branchSide;
        // euler tour interval in _dfsOrder
        internal int tin;
        internal int tout => tin + subtreeSize - 1;
    }

    public record struct ArticulationPoint
    {
        /// <summary>
        /// index of the node that is the articulation point
        /// </summary>
        public int nodeIdx;
        /// <summary>
        /// size of the component containing DFS root after deletion of this articulation point
        /// </summary>
        public int rootComponentSize;
    }

    public interface IBridgeFinder : IIncidence
    {
        int SeenCount { get; }
        List<Bridge> Bridges { get; }
        ArticulationPoint[] ArticulationPoints { get; }
        int Root { get; set; }
        int LastSearchComponentSize { get; }
        int LastSearchEdgeCount { get; }

        IEnumerable<int> BranchSideComponent(Bridge b);
        IEnumerable<int> RootSideComponent(Bridge b);
        void FindBridges();
        bool HasSeen(int nodeIdx);
        IEnumerable<int> IterateComponent();
        IEnumerable<int> IterateComponentBFS(int seed);
        IEnumerable<int> IterateComponentBFS();
        IEnumerable<int> IterateEdges();
        void MarkEdgeAsDeleted(int edgeIdx);
        void MarkNodeAsDisconnected(int nodeIdx);
        bool RevertMarkAsDeleted(int edgeIdx);
        int RootComponentSizeAfterDeleting(int nodeIdx);
        IReadOnlyList<int> EulerTour();
    }

    /// <summary>
    /// Wraps an IMutableIncidenceSource, provides methods for finding bridges and articulation points in the graph
    /// using Tarjan's bridge finding algorithm. Supports deletion of edges and nodes by marking edges (reversible)
    /// and repeated queries after deletions.
    /// </summary>
    public class BridgeFinder<idxT> : IBridgeFinder where idxT : unmanaged, INumber<idxT>, IMinMaxValue<idxT>, IBinaryInteger<idxT>
    {
        IMutableIncidenceSource<idxT> _incdtSrc;

        ThreadSafeBitArray _seen;
        idxT[] _disc;
        idxT[] _low;
        idxT[] _subtree;
        // Euler tour order
        idxT[] _dfsOrder;
        readonly Stack<StackFrame> _tarjanStack = new();
        readonly Stack<int> _iterStack = new();
        readonly Queue<int> _iterQueue = new();

        idxT _time;
        idxT _root = idxT.Zero;
        public int Root { get => ToInt(_root); set => _root = idxT.CreateChecked(value); }
        public List<Bridge> Bridges { get; } = new();
        public ArticulationPoint[] ArticulationPoints =>
            cutSums.SelectToArray(kvp => new ArticulationPoint() { 
                nodeIdx = kvp.Key, 
                rootComponentSize = _lastSearchComponentSize - 1 - kvp.Value,
            });
        public int RootComponentSizeAfterDeleting(int nodeIdx)
            => cutSums.TryGetValue(nodeIdx, out int rootCompSize) ? _lastSearchComponentSize - 1 - rootCompSize : _lastSearchComponentSize - 1;
        private Dictionary<int, int> cutSums = new();
        int rootChildCount;
        int _lastSearchComponentSize = -1;
        int _lastSearchEdgeCount = -1;
        public int LastSearchComponentSize => _lastSearchComponentSize;
        public int LastSearchEdgeCount => _lastSearchEdgeCount;

        private int ToInt(idxT idx) => IncidenceList<idxT>.ToInt(idx);
        private idxT EdgeIdxT(idxT idx) => IncidenceList<idxT>.EdgeIdxT(idx);
        private bool IsInvalid(idxT idx) => IncidenceList<idxT>.IsInvalid(idx);
        private idxT invalidIdx => IncidenceList<idxT>.invalidIdx;

        public BridgeFinder(IMutableIncidenceSource<idxT> incidenceSource)
        {
            _incdtSrc = incidenceSource;

            int n = incidenceSource.MaxNodeIdxCount;
            int e = incidenceSource.MaxEdgeIdxCount;
            int max = int.Max(n, e);
            AllocArrays(max);
        }

        public void Reset(IMutableIncidenceSource<idxT> newIncidenceSource)
        {
            _incdtSrc = newIncidenceSource;
            int n = newIncidenceSource.MaxNodeIdxCount;
            int e = newIncidenceSource.MaxEdgeIdxCount;
            int max = int.Max(n, e);
            if (max > _seen.Length)
            {
                //grow arrays
                AllocArrays(max);
            }
            Clear();
            _lastSearchComponentSize = -1;
            _lastSearchEdgeCount = -1;
        }

        private void AllocArrays(int capacity)
        {
            capacity = (int)BitOperations.RoundUpToPowerOf2((uint)capacity);
            _seen = new ThreadSafeBitArray(capacity);
            _disc = new idxT[capacity];
            _low = new idxT[capacity];
            _subtree = new idxT[capacity];
            _dfsOrder = new idxT[capacity];
        }

        public bool HasSeen(int nodeIdx) => _seen[nodeIdx];

        public IEnumerable<int> IterateComponent() => IterateComponent(ToInt(_root));

        public IEnumerable<int> IterateComponent(int seed)
        {
            _iterStack.Clear();
            _seen.SetAll(false);
            _iterStack.Push(seed);
            _seen[seed] = true;

            while (_iterStack.Count > 0)
            {
                int u = _iterStack.Pop();
                yield return u;
                foreach (var (v, _) in Neighbors(u))
                {
                    if (_seen[v]) continue;
                    _seen[v] = true;
                    _iterStack.Push(v);
                }
            }
        }

        public IEnumerable<int> IterateComponentBFS() => IterateComponentBFS(ToInt(_root));
        public IEnumerable<int> IterateComponentBFS(int seed)
        {
            _iterQueue.Clear();
            _iterStack.Clear();
            _seen.SetAll(false);
            _iterQueue.Enqueue(seed);
            _seen[seed] = true;

            while (_iterQueue.Count > 0)
            {
                int u = _iterQueue.Dequeue();
                yield return u;

                // because IncidenceList always adds to the front of the singly linked list (overwriting head)
                // it is in reverse insertion order naturally.
                // When we use a stack (DFS), the stack reverses neighbor order again on its own.
                // To get consistent ordering convention (insertion order) between DFS and BFS, we must reverse neighbor order in BFS
                foreach (var (v, _) in Neighbors(u))
                {
                    if (_seen[v]) continue;
                    _seen[v] = true;
                    _iterStack.Push(v);
                }

                // enqueue in reverse from stack
                while (_iterStack.Count > 0)
                    _iterQueue.Enqueue(_iterStack.Pop());
            }
        }

        public IEnumerable<int> IterateEdges() => IterateEdges(ToInt(_root));
        public IEnumerable<int> IterateEdges(int seed)
        {
            _iterStack.Clear();
            _seen.SetAll(false);
            _iterStack.Push(seed);

            while (_iterStack.Count > 0)
            {
                int u = _iterStack.Pop();
                foreach (var (v, e) in Neighbors(u))
                {
                    if (_seen[e]) continue;
                    _seen[e] = true;
                    yield return e;
                    _iterStack.Push(v);
                }
            }
        }
        public IEnumerable<(int nodeIdx, int edgeIdx)> Neighbors(int u) => _incdtSrc.Neighbors(u);

        private void Clear()
        {
            _seen.SetAll(false);
            _tarjanStack.Clear();
            _time = idxT.Zero;
            Bridges.Clear();
            cutSums.Clear();
            rootChildCount = 0;
        }

        public void FindBridges()
        {
            //clear state for reuse
            //The rest of the Tarjan arrays will get overwritten in order.
            Clear();

            // seed the DFS with root
            _tarjanStack.Push(StackFrame.Entry(_root, parentU: invalidIdx,
                parentEdge: invalidIdx, nextInc: _incdtSrc.GetHead(ToInt(_root))));

            while (_tarjanStack.Count > 0)
            {
                var f = _tarjanStack.Pop();
                if (!f.Returning)
                    ProcessEntry(f);
                else
                    ProcessExit(f);
            }

            _lastSearchComponentSize = SeenCount;
            _lastSearchEdgeCount = EdgeCount;
            // root is an articulation point if it has more than 1 child
            if (rootChildCount > 1) cutSums[ToInt(_root)] = _lastSearchComponentSize - 2;
        }

        public IEnumerable<int> RootSideComponent(Bridge b)
        {
            for (int i = 0; i < b.tin; i++)
                yield return ToInt(_dfsOrder[i]);
            for (int i = b.tout + 1; i < int.CreateChecked(_time); i++)
                yield return ToInt(_dfsOrder[i]);
        }

        public IEnumerable<int> BranchSideComponent(Bridge b)
        {
            var slice = new ArraySegment<idxT>(_dfsOrder, b.tin, b.tout + 1 - b.tin);
            return slice.Select(ToInt);
        }

        public IReadOnlyList<int> EulerTour()
        {
            var slice = new ArraySegment<idxT>(_dfsOrder, 0, int.CreateChecked(_time));
            return slice.Select(ToInt);
        }

        public void MarkEdgeAsDeleted(int edgeIdx) => _incdtSrc.MarkEdgeAsDeleted(edgeIdx);

        public void MarkNodeAsDisconnected(int nodeIdx) => _incdtSrc.MarkNodeAsDisconnected(nodeIdx);
        public bool RevertMarkAsDeleted(int edgeIdx) => _incdtSrc.RevertMarkAsDeleted(edgeIdx);

        public int SeenCount => _seen.PopulationCount;
        public int NodeCount => _incdtSrc.NodeCount;
        public int EdgeCount => _incdtSrc.EdgeCount;
        public int MaxEdgeIdxCount => _incdtSrc.MaxEdgeIdxCount;
        public int MaxNodeIdxCount => _incdtSrc.MaxNodeIdxCount;

        private void ProcessEntry(StackFrame f)
        {
            int u = ToInt(f.U);
            //resume?
            if (!_seen[u])
            {
                _seen[u] = true;
                _disc[u] = _low[u] = _time;
                _subtree[u] = idxT.One;
                _dfsOrder[int.CreateChecked(_time)] = f.U;
                _time++;
                if (f.ParentU == _root) rootChildCount++;
                // post-order marker
                _tarjanStack.Push(StackFrame.Exit(f));
            }

            Incidence<idxT> entry;
            // scan neighbors
            for (idxT inc = f.NextInc; !IsInvalid(inc); inc = entry.Next)
            {
                entry = _incdtSrc.GetIncidence(inc);
                idxT eIdx = EdgeIdxT(inc);
                if (eIdx == f.ParentEdge)
                    continue;
                //skip deleted edges when processing neighbors
                if (_incdtSrc.IsEdgeDeleted(eIdx))
                    continue;

                int v = ToInt(entry.To);

                if (!_seen[v])
                {
                    // resume u after this incidence
                    f.NextInc = entry.Next;
                    if (!IsInvalid(f.NextInc)) _tarjanStack.Push(f);
                    // dive into v
                    _tarjanStack.Push(StackFrame.Entry(entry.To, f.U, eIdx, _incdtSrc.GetHead(v)));
                    return;
                }
                else
                {
                    // back-edge
                    _low[u] = idxT.Min(_low[u], _disc[v]);
                }
            }
            // if no unseen neighbor, we'll handle exit next
        }

        private void ProcessExit(StackFrame f)
        {
            int u = ToInt(f.U);    
            if (IsInvalid(f.ParentU)) return;   // root

            int p = ToInt(f.ParentU);
            _subtree[p] += _subtree[u];
            _low[p] = idxT.Min(_low[p], _low[u]);

            // Bridge condition
            if (_low[u] > _disc[p])
            {
                Bridges.Add(new Bridge()
                {
                    edgeIdx = IncidenceList<idxT>.ToInt(f.ParentEdge),
                    subtreeSize = ToInt(_subtree[u]),
                    rootSide = p,
                    branchSide = u,
                    tin = ToInt(_disc[u]),
                });
            }

            // Non-root articulation condition
            if (_low[u] >= _disc[p] && f.ParentU != _root)
            {
                bool found = cutSums.TryGetValue(p, out int cutSum);
                if (!found) cutSum = 0;
                cutSums[p] = cutSum + ToInt(_subtree[u]);
            }
        }

        // small struct to hold Tarjan manual-stack frames
        private struct StackFrame
        {
            public idxT U, ParentU, ParentEdge;
            public idxT NextInc;

            //because incidences are always added in pairs in the undirected graph, both maxIndex and maxIndex-1 are never used
            //while maxIndex signals invalid/end of linked list, we high-jack maxIndex-1 for a boolean
            public bool Returning => NextInc == returningIdxT;
            private static readonly idxT returningIdxT = idxT.MaxValue - idxT.One;

            public static StackFrame Entry(idxT u, idxT parentU, idxT parentEdge, idxT nextInc)
                => new() { U = u, ParentU = parentU, ParentEdge = parentEdge, NextInc = nextInc };

            public static StackFrame Exit(StackFrame f)
            {
                f.NextInc = returningIdxT;
                return f;
            }
        }
    }
}