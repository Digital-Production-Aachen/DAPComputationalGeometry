using System;
using System.Collections.Generic;
using System.Data;
using System.Linq;
using static g3.Clipper2Wrapper;

namespace g3
{
    [Flags]
    public enum ConnectionState : byte
    {
        UNKNOWN           = 0b0000_0000,  // still uninitialized
        NOT_CONNECTED     = 0b0000_0001,  // no connections

        CONNECTED_UP      = 0b0000_0010,  // there's a path to an exit upward
        CONNECTED_DOWN    = 0b0000_0100,  // there's a path to an exit downward
        CONNECTED_UP_DOWN = 0b0000_0110,  // named flag combo: exits upward and downward

        EXIT              = 0b0001_1000,  // edge leads out of the graph up and down
        EXIT_UP           = 0b0001_0000,  // edge leads up out of the graph
        EXIT_DOWN         = 0b0000_1000,  // edge leads down out of the graph

        //LOCAL_MINIMUM     = 0b0010_0000,  // area of the nfp hole is minimal locally at this node
    }

    public static class ConnectionStateExtensions
    {
        public static bool IsConnected(this ConnectionState connection)
        {
            const ConnectionState connectedMask = ConnectionState.CONNECTED_UP | ConnectionState.CONNECTED_DOWN;
            return (connection & connectedMask) != 0;
        }
        public static bool IsConnectedDown(this ConnectionState connection)
            => (connection & ConnectionState.CONNECTED_DOWN) != 0;
        public static bool IsConnectedUp(this ConnectionState connection)
            => (connection & ConnectionState.CONNECTED_UP) != 0;
        public static bool IsExit(this ConnectionState connection)
            => (connection & ConnectionState.EXIT) != 0;
    }

    public sealed class PolyHoleNode
    {
        //indices of the polygons forming a group in the poly tree
        //(hole and outers nested in hole in the next tree level)
        public readonly int[] layerPolyGroupIdcs;

        public PolyHoleNode(int[] layerPolyGroupIdcs)
        {
            this.layerPolyGroupIdcs = layerPolyGroupIdcs;
        }

        public int HolePolyIdx => layerPolyGroupIdcs[0];
        public ConnectionState connectState = ConnectionState.UNKNOWN;
        public readonly List<PolyHoleConnection> connectionsUp = new();
        public readonly List<PolyHoleConnection> connectionsDown = new();
    }

    public struct PolyHoleConnection
    {
        public readonly Polygon2d connectionPoly;
        public readonly int targetNodeIdx;
        public bool IsExitOnly => targetNodeIdx == exitMarker;
        const int exitMarker = -2;

        public PolyHoleConnection(Polygon2d connectionPoly) : this()
        {
            this.connectionPoly = connectionPoly;
            this.targetNodeIdx = exitMarker;
        }

        public PolyHoleConnection(int targetNodeIdx, Polygon2d connectionPoly) : this()
        {
            this.connectionPoly = connectionPoly;
            this.targetNodeIdx = targetNodeIdx;
        }
    }

    public sealed class Polygon2dLayer<T, T2>
    {
        public PolyHoleNode[] Nodes { get; }
        public Polygon2d[] Polygons { get; }
        public T2 MutableAttribute { get; set; }
        public T Value { get; }

        public Polygon2dLayer(Polygon2d[] polygons, T value)
        {
            this.Polygons = polygons;
            Value = value;
            int maxHoleIdx = -1;
            for (int i = 0; i < polygons.Length; i++)
            {
                if (polygons[i].IsHole) maxHoleIdx = i;
            }
            if (maxHoleIdx >= 0)
            {
                Nodes = new PolyHoleNode[maxHoleIdx + 1];
            }
            else
            {
                Nodes = Array.Empty<PolyHoleNode>();
            }
        }

        public bool AllConnected => Nodes.Length > 0 && Nodes.All(c => c == null ? true : c.connectState.IsConnected());
        public bool HasConnections => Nodes.Any(n => n != null);
        public int ConnectedCount => Nodes.Count(c => c != null && c.connectState.IsConnected());

        public void AddNodes(Dictionary<int, int[]> holeIdcsLookup)
        {
            foreach (var kvp in holeIdcsLookup)
            {
                int holeIdx = kvp.Key;
                ref var node = ref Nodes[holeIdx];
                if (node == null)
                    node = new PolyHoleNode(kvp.Value);
            }
        }

        public void AddConnection(IEnumerable<Polygon2dLayerConnection> connections, bool upConnection)
        {
            foreach (var connection in connections) AddConnection(connection, upConnection);
        }
        public void AddConnection(Polygon2dLayerConnection connection, bool upConnection)
        {
            ref PolyHoleNode node = ref Nodes[connection.Layer1PolyIdx];
            PolyHoleConnection edge;
            if (connection.IsOutsideLayer2)
            {
                edge = new(connection.connectionPoly);
                if (upConnection)
                    node.connectState = ConnectionState.EXIT_UP | ConnectionState.CONNECTED_UP;
                else
                    node.connectState = ConnectionState.EXIT_DOWN | ConnectionState.CONNECTED_DOWN;
            }
            else
            {
                edge = new(connection.firstLayer2PolyIdx, connection.connectionPoly);
            }

            if (upConnection)
                node.connectionsUp.Add(edge);
            else
                node.connectionsDown.Add(edge);
        }

        public void MarkExitNodes(Polygon2d[] targetPolys, bool upConnection)
        {
            foreach (var node in Nodes)
            {
                if (node is null) continue;
                List<PolyHoleConnection> connections = upConnection ? node.connectionsUp : node.connectionsDown;
                foreach (var connection in connections)
                {
                    if (connection.IsExitOnly) continue;
                    int target = connection.targetNodeIdx;
                    Polygon2d targetPoly = targetPolys[target];
                    if (targetPoly.IsHole) continue;

                    // check if target outer is also an outermost polygon, or lower level polygon nested inside another hole
                    Vector2d testVtx = targetPoly[0];
                    if (targetPolys.Any(poly => poly.IsHole && poly.ContainsInclusive(testVtx))) continue;

                    // success, this is an exit connection
                    node.connectState = upConnection ? ConnectionState.EXIT_UP : ConnectionState.EXIT_DOWN;
                }
            }
        }

        public void AddConnectionReversed(IEnumerable<Polygon2dLayerConnection> connections, bool upConnection)
        {
            foreach (var connection in connections) AddConnectionReversed(connection, upConnection);
        }
        public void AddConnectionReversed(Polygon2dLayerConnection connection, bool upConnection)
        {
            if (connection.IsOutsideLayer2) return;
            ref PolyHoleNode node = ref Nodes[connection.firstLayer2PolyIdx];
            PolyHoleConnection edge = new(connection.Layer1PolyIdx, connection.connectionPoly);
            if (upConnection)
                node.connectionsUp.Add(edge);
            else
                node.connectionsDown.Add(edge);
        }

        public static void PropagateConnectedState(IReadOnlyList<Polygon2dLayer<T, T2>> stackToPropagate)
        {
            Stack<(int layerIdx, int nodeIdx)> toDo = new();
            for (int exitLayerIdx = 0; exitLayerIdx < stackToPropagate.Count; exitLayerIdx++)
            {
                var layer = stackToPropagate[exitLayerIdx];
                for (int exitNodeIdx = 0; exitNodeIdx < layer.Nodes.Length; exitNodeIdx++)
                {
                    PolyHoleNode node = layer.Nodes[exitNodeIdx];
                    if (node is null) continue;
                    if (!node.connectState.IsExit()) continue;
                    PropagateExit(stackToPropagate, toDo, (exitLayerIdx, exitNodeIdx));
                }
            }
        }

        private static void PropagateExit(IReadOnlyList<Polygon2dLayer<T, T2>> stackToPropagate, Stack<(int layerIdx, int nodeIdx)> toDo, (int exitIdx, int exitNodeIdx) exit)
        {
            //propagate up
            toDo.Clear();
            toDo.Push(exit);
            while (toDo.Count > 0)
            {
                (int layerIdx, int nodeIdx) = toDo.Pop();
                for (int i = layerIdx; i < stackToPropagate.Count; ++i)
                {
                    PolyHoleNode node = stackToPropagate[i].Nodes[nodeIdx];
                    node.connectState |= ConnectionState.CONNECTED_DOWN;
                    List<PolyHoleConnection> conn = node.connectionsUp;
                    if (conn.Count == 0) break;
                    //push all but first connection to toDo
                    for (int j = 1; j < conn.Count; j++)
                    {
                        if (conn[j].IsExitOnly) continue;
                        toDo.Push((i + 1, conn[j].targetNodeIdx));
                    }
                    if (conn[0].IsExitOnly) break;
                    nodeIdx = conn[0].targetNodeIdx;
                }
            }

            //propagate down
            toDo.Clear();
            toDo.Push(exit);
            while (toDo.Count > 0)
            {
                (int layerIdx, int nodeIdx) = toDo.Pop();
                for (int i = layerIdx; i >= 0; --i)
                {
                    PolyHoleNode node = stackToPropagate[i].Nodes[nodeIdx];
                    node.connectState |= ConnectionState.CONNECTED_UP;
                    List<PolyHoleConnection> conn = node.connectionsDown;
                    if (conn.Count == 0) break;
                    //push all but first connection to toDo
                    for (int j = 1; j < conn.Count; j++)
                    {
                        if (conn[j].IsExitOnly) continue;
                        toDo.Push((i - 1, conn[j].targetNodeIdx));
                    }
                    if (conn[0].IsExitOnly) break;
                    nodeIdx = conn[0].targetNodeIdx;
                }
            }
        }
    }
}
