using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace g3
{
    public interface IIncidenceAdd : IIncidence
    {
        void AddEdge(int u, int v);
        int AddUninitializedNode();
        void InitNodeWithNoEdges(int nodeIdx);
        bool IsNodeInitialized(int nodeIdx);
    }
    public interface IIncidence
    {
        int NodeCount { get; }
        int EdgeCount { get; }
        int MaxEdgeIdxCount { get; }
        int MaxNodeIdxCount { get; }
        IEnumerable<int> IterateComponent(int seed);
        IEnumerable<int> IterateEdges(int seed);
        IEnumerable<(int nodeIdx, int edgeIdx)> Neighbors(int u);
    }

    public interface IIncidenceSource<idxT> : IIncidence where idxT : unmanaged, INumber<idxT>, IMinMaxValue<idxT>, IBinaryInteger<idxT>
    {
        Incidence<idxT> GetIncidence(idxT idx);
        idxT GetHead(int idx);
        bool IsEdgeDeleted(idxT edgeIdx);
        bool IsEdgeDeleted(int edgeIdx) => IsEdgeDeleted(idxT.CreateChecked(edgeIdx));
    }

    public interface IMutableIncidenceSource<idxT> : IIncidenceSource<idxT> where idxT : unmanaged, INumber<idxT>, IMinMaxValue<idxT>, IBinaryInteger<idxT>
    {
        void MarkEdgeAsDeleted(int edgeIdx);
        void MarkNodeAsDisconnected(int nodeIdx);
        public bool RevertMarkAsDeleted(int edgeIdx);
    }
    public struct Incidence<idxT> where idxT : unmanaged, INumber<idxT>, IMinMaxValue<idxT>, IBinaryInteger<idxT>
    {
        public idxT To;        // neighbor node
        public idxT Next;      // next incidence index for the same source
    }

    /// <summary>
    /// Incidence list to store edges of graphs. For customization (adding custom data) nodes and/or edges arrays have to be stored externally to associate them with the indices. Edges are undirected.
    /// Uses a flat array structure to store linked lists of incidences. Supports using byte, ushort or int type indices based on capacity for smaller storage for small graphs. Highly cache friendly.
    /// </summary>
    /// <typeparam name="idxT"></typeparam>
    public partial class IncidenceList<idxT> : IIncidenceAdd, IIncidenceSource<idxT> where idxT : unmanaged, INumber<idxT>, IMinMaxValue<idxT>, IBinaryInteger<idxT>
    {
        /// <summary>
        /// A lightweight wrapper to add a virtual node to an existing IncidenceList.
        /// Because the base incidence list is used read only, it is only possible to add undirected edges
        /// from the new virtual node to existing nodes.
        /// </summary>
        public class IncidenceListDeltaNode : IIncidence, IMutableIncidenceSource<idxT>
        {
            IncidenceList<idxT> baseIncidences;
            //incidence lists for the delta
            private readonly List<Incidence<idxT>> deltaIncidences;
            //dict for overwritten heads
            private readonly Dictionary<int, idxT> deltaHeads;
            readonly HashSet<idxT> deletedEdges = new();

            public int EdgeCount => MaxEdgeIdxCount - deletedEdges.Count;
            public int MaxEdgeIdxCount => baseIncidences.EdgeCount + deltaIncidences.Count / 2;
            public int MaxNodeIdxCount { get; private set; }
            public int NodeCount => baseIncidences.NodeCount + deltaNodeCount;
            private int deltaNodeCount = 0;
            public int DeltaNodeIdx => baseIncidences.NodeCount;

            public IncidenceListDeltaNode(IncidenceList<idxT> baseIncidences, int neighborCapacity = 0)
            {
                this.baseIncidences = baseIncidences ?? throw new ArgumentNullException(nameof(baseIncidences));
                deltaIncidences = new(neighborCapacity * 2);
                deltaHeads = new(neighborCapacity);
                MaxNodeIdxCount = baseIncidences.NodeCount;
            }

            #region IIncidenceSource implementation
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public Incidence<idxT> GetIncidence(idxT idxT)
            {
                int idx = ToInt(idxT);
                int baseCnt = baseIncidences.IncidenceCount;
                return idx < baseCnt ? baseIncidences.GetIncidence(idxT) : deltaIncidences[idx - baseCnt];
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public idxT GetHead(int idx)
            {
                bool found = deltaHeads.TryGetValue(idx, out idxT head);
                return found ? head : baseIncidences.GetHead(idx);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool IsEdgeDeleted(idxT edgeIdx)
            {
                return deletedEdges.Contains(edgeIdx);
            }
            #endregion

            public bool AddNode(int nodeIdx)
            {
                bool success = deltaHeads.TryAdd(nodeIdx, uninitializedIdx);
                if (success)
                {
                    deltaNodeCount++;
                    MaxNodeIdxCount = int.Max(MaxNodeIdxCount, nodeIdx + 1);
                }
                return success;
            }

            public void AddEdge(int u, int v)
            {
                int baseCnt = baseIncidences.incidences.Count;
                idxT incdsCnt = idxT.CreateChecked(baseCnt + deltaIncidences.Count);
                // 1) add incidence u→v
                idxT headU = GetHead(u);
                headU = headU != uninitializedIdx ? headU : invalidIdx;
                deltaIncidences.Add(new Incidence<idxT>
                {
                    To = idxT.CreateChecked(v),
                    Next = headU,
                });
                deltaHeads[u] = incdsCnt;

                // 2) add incidence v→u
                idxT headV = GetHead(v);
                headV = headV != uninitializedIdx ? headV : invalidIdx;
                deltaIncidences.Add(new Incidence<idxT>
                {
                    To = idxT.CreateChecked(u),
                    Next = headV,
                });
                deltaHeads[v] = incdsCnt + idxT.One;
            }

            public void MarkEdgeAsDeleted(int edgeIdx)
            {
                deletedEdges.Add(idxT.CreateChecked(edgeIdx));
            }

            public void MarkNodeAsDisconnected(int nodeIdx)
            {
                foreach (var neighbor in Neighbors(nodeIdx))
                {
                    MarkEdgeAsDeleted(neighbor.edgeIdx);
                }
            }

            public bool RevertMarkAsDeleted(int edgeIdx)
            {
                return deletedEdges.Remove(idxT.CreateChecked(edgeIdx));
            }

            public IEnumerable<(int nodeIdx, int edgeIdx)> Neighbors(int u) => IncidenceList<idxT>.Neighbors(this, u);
            public IEnumerable<int> IterateComponent(int seed) => IncidenceList<idxT>.IterateComponent(this, seed);
            public IEnumerable<int> IterateEdges(int seed) => IncidenceList<idxT>.IterateEdges(this, seed);
        }

        #region fields
        //the edge index is implicitly derived by the layout, for each edge 2 incidences are added
        private readonly ReadAndAddOnlyList<Incidence<idxT>> incidences;
        private readonly ReadAndAddOnlyList<idxT> head;
        #endregion

        #region properties
        public int NodeCount => head.Count;
        public int InitializedNodeCount => head.Count(i => i != uninitializedIdx);
        public int EdgeCount => incidences.Count / 2;
        public int MaxEdgeIdxCount => EdgeCount;
        public int MaxNodeIdxCount => NodeCount;
        public int IncidenceCount => incidences.Count;
        public static idxT MaxCapacity => (idxT.MaxValue / TWO) - idxT.One;
        #endregion

        #region helpers
        internal static int ToInt(idxT idx) => int.CreateChecked(idx);
        public static int EdgeIdxFromIncidenceIdx(idxT incidenceIdx) => ToInt(incidenceIdx >> 1);
        public static idxT EdgeIdxT(idxT incidenceIdx) => incidenceIdx >> 1;
        internal static bool IsInvalid(idxT idx) => idx == invalidIdx;
        internal static readonly idxT invalidIdx = idxT.MaxValue;
        internal static readonly idxT uninitializedIdx = idxT.MaxValue - idxT.One;
        private static readonly idxT TWO = idxT.CreateChecked(2);
        #endregion

        public IncidenceList(int nodeCapacity = 0)
        {
            incidences = new(nodeCapacity * 2);
            head = new(nodeCapacity);
        }

        /// <summary>
        /// Create a new IncidenceList for the desired maximum capacity.
        /// This is either of type byte, ushort or int.
        /// </summary>
        /// <param name="startCapacity">desired start capacity of the internal lists</param>
        /// <param name="maxCapacity">desired maximum node capacity. Resulting MaxCapacity can differ (uses byte, ushort or int)</param>
        /// <returns></returns>
        public static IIncidenceAdd IncidenceListWithCapacity(int startCapacity, int maxCapacity)
        {
            if (maxCapacity <= IncidenceList<byte>.MaxCapacity)
                return new IncidenceList<byte>(startCapacity);
            if (maxCapacity <= IncidenceList<ushort>.MaxCapacity)
                return new IncidenceList<ushort>(startCapacity);
            //.NET arrays can't handle uint or ulong sizes
            return new IncidenceList<int>(startCapacity);
        }

        #region IIncidenceSource implementation
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Incidence<idxT> GetIncidence(idxT idx)
        {
            return incidences[ToInt(idx)];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public idxT GetHead(int idx)
        {
            return head[idx];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IsEdgeDeleted(idxT edgeIdx) => false;
        #endregion

        #region methods
        public int AddUninitializedNode()
        {
            int newIndex = head.Count;
            head.Add(uninitializedIdx);
            return newIndex;
        }

        public bool IsNodeInitialized(int nodeIdx)
        {
            if (head.Count <= (uint)nodeIdx) return false;
            return head[nodeIdx] != uninitializedIdx;
        }

        public void InitNodeWithNoEdges(int nodeIdx)
        {
            head.AsSpan()[nodeIdx] = invalidIdx;
        }

        public void AddEdge(int u, int v)
        {
            idxT incdsCnt = idxT.CreateChecked(incidences.Count);

            Span<idxT> headSpan = head.AsSpan();
            // 1) add incidence u→v
            idxT headU = headSpan[u];
            if (headU == uninitializedIdx) headU = invalidIdx;
            incidences.Add(new Incidence<idxT>
            {
                To = idxT.CreateChecked(v),
                Next = headU,
            });

            // 2) add incidence v→u
            idxT headV = headSpan[v];
            if (headV == uninitializedIdx) headV = invalidIdx;
            incidences.Add(new Incidence<idxT>
            {
                To = idxT.CreateChecked(u),
                Next = headV,
            });

            // 3) update heads
            headSpan[u] = incdsCnt;
            headSpan[v] = incdsCnt + idxT.One;
        }

        public IEnumerable<(int nodeIdx, int edgeIdx)> Neighbors(int u) => Neighbors(this, u);
        private static IEnumerable<(int nodeIdx, int edgeIdx)> Neighbors(IIncidenceSource<idxT> incidenceSource, int u)
        {
            Incidence<idxT> entry;
            for (idxT idx = incidenceSource.GetHead(u); !IsInvalid(idx); idx = entry.Next)
            {
                entry = incidenceSource.GetIncidence(idx);
                int eIdx = EdgeIdxFromIncidenceIdx(idx);
                if (incidenceSource.IsEdgeDeleted(eIdx))
                    continue;
                yield return (ToInt(entry.To), eIdx);
            }
        }

        public IEnumerable<int> IterateComponent(int seed) => IterateComponent(this, seed);
        public IEnumerable<int> IterateEdges(int seed) => IterateEdges(this, seed);
        private static IEnumerable<int> IterateComponent(IIncidenceSource<idxT> incidenceSource, int seed)
        {
            var seen = new BitArray(incidenceSource.MaxNodeIdxCount);
            var stack = new Stack<int>();
            stack.Push(seed);
            seen[seed] = true;

            while (stack.Count > 0)
            {
                int u = stack.Pop();
                yield return u;
                foreach (var (v, _) in Neighbors(incidenceSource, u))
                {
                    if (seen[v]) continue;
                    seen[v] = true;
                    stack.Push(v);
                }
            }
        }

        private static IEnumerable<int> IterateEdges(IIncidenceSource<idxT> incidenceSource, int seed)
        {
            var seen = new BitArray(incidenceSource.MaxEdgeIdxCount);
            var stack = new Stack<int>();
            stack.Push(seed);
            seen[seed] = true;

            while (stack.Count > 0)
            {
                int u = stack.Pop();
                foreach (var (_, e) in Neighbors(incidenceSource, u))
                {
                    if (seen[e]) continue;
                    seen[e] = true;
                    yield return e;
                    stack.Push(e);
                }
            }
        }
        #endregion
    }
}