using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using static g3.ConvexHullBridgeFinder;

namespace g3
{
    public class ConvexHullBridgeFinder
    {
        public interface IConvexHullBridge
        {
            public int StartIdx { get; }
            public int EndIdx { get; }
            public int BridgedVtxCount(int vtxCnt) => EndIdx >= StartIdx ? EndIdx - StartIdx + 1 : vtxCnt - StartIdx + EndIdx + 1;
            public Segment2d BridgeSegment(IList<Vector2d> vertices) => new Segment2d(vertices[StartIdx], vertices[EndIdx]);
        }

        public class BridgeResult<T> where T : IConvexHullBridge
        {
            public Polygon2d Polygon { get => _polygon ?? throw new InvalidOperationException("BridgeResult is from a point cloud, not a polygon"); init => _polygon = value; }
            public IList<Vector2d> Vertices
            {
                get => IsPolygonHull ? Polygon.Vertices as IList<Vector2d> : _vertices;
                init { _vertices = value; }
            }
            public bool IsPolygonHull => _polygon != null;

            private readonly IList<Vector2d> _vertices;
            private Polygon2d _polygon;

            public List<T> Bridges { get; init; }
            public ConvexHull2 ConvexHull { get; init; }
            public Polygon2d HullPolygon => IsConvex ? Polygon : ConvexHull.GetHullPolygon();
            public bool IsConvex => ConvexHull == null;

            public PolyLine2d[] AsPolylines => Bridges.SelectToArray(bridge => new PolyLine2d([Vertices[bridge.StartIdx], Vertices[bridge.EndIdx]]));

            public int GetBridgeIndex(int vtxIndex) //works with single-poly-brdiges, gives back if the poly-vertex given is covered by a bridge
            {
                for (int i = 0; i < Bridges.Count; i++)
                {
                    T bridge = Bridges[i];
                    int x = bridge.StartIdx;
                    int y = bridge.EndIdx;
                    bool wraps = y < x;
                    if (wraps)
                    {
                        if (vtxIndex >= y || vtxIndex <= x)
                            return i;
                    }
                    else
                    {
                        if (vtxIndex >= x && vtxIndex <= y)
                            return i;
                    }
                }
                return -1;
            }

            public IEnumerable<Polygon2d> BridgedConcavitiesAsPolygon()
            {
                foreach (var bridge in Bridges)
                {
                    ReadOnlySpan<Vector2d> polyVerts = Polygon.VerticesAsReadOnlySpan;
                    int vtxCnt = polyVerts.Length;
                    int concavityVtxCnt = bridge.BridgedVtxCount(vtxCnt);
                    Polygon2d concavity = new Polygon2d();
                    Span<Vector2d> verts = concavity.VerticesAsSpanWithCount(concavityVtxCnt);
                    for (int i = 0; i < verts.Length; i++)
                    {
                        int idx = bridge.StartIdx + i;
                        idx = idx < vtxCnt ? idx : idx - vtxCnt;
                        verts[i] = polyVerts[idx];
                    }

                    yield return concavity;
                }
            }

            public Polygon2d ConvexHullWithPocketForBridge(int bridgeIdx)
            {
                Polygon2d hullWithPocket = new();
                int[] hullIndices = ConvexHull.HullIndices;
                ReadOnlySpan<Vector2d> polyVerts = Polygon.VerticesAsReadOnlySpan;
                var bridge = Bridges[bridgeIdx];
                int bridgedVtxCount = bridge.BridgedVtxCount(Polygon.VertexCount) - 2;
                Span<Vector2d> targetVerts = hullWithPocket.VerticesAsSpanWithCount(hullIndices.Length + bridgedVtxCount);
                int targetCnt = 0;
                for (int i = 0; i < hullIndices.Length; i++)
                {
                    int hullIdx = hullIndices[i];
                    targetVerts[targetCnt++] = polyVerts[hullIdx];
                    if (hullIdx == bridge.StartIdx)
                    {
                        for (int j = 1; j <= bridgedVtxCount; j++)
                        {
                            int wrappedIdx = hullIdx + j;
                            wrappedIdx = wrappedIdx < polyVerts.Length ? wrappedIdx : wrappedIdx - polyVerts.Length; //wrap around
                            targetVerts[targetCnt++] = polyVerts[wrappedIdx];
                        }
                    }
                }
                return hullWithPocket;
            }
        }

        public static BridgeResult<Bridge> ConvexHullBridges(Polygon2d polygon)
            => GetBridges(polygon, (p, vec, verts) => new Bridge() { StartIdx = vec.x, EndIdx = vec.y });

        public struct Bridge : IConvexHullBridge
        {
            public int StartIdx { get; set; }

            public int EndIdx { get; set; }

            public int BridgedVtxCount(int vtxCnt) => EndIdx >= StartIdx ? EndIdx - StartIdx + 1 : vtxCnt - StartIdx + EndIdx + 1;

            public Segment2d BridgeSegment(IList<Vector2d> vertices) { return new Segment2d(vertices[StartIdx], vertices[EndIdx]); }
        }

        public struct MultiPolyBridge : IConvexHullBridge
        {
            Bridge bridge;

            public int StartIdx { get => bridge.StartIdx; set => bridge.StartIdx = value; }
            public int EndIdx { get => bridge.EndIdx; set => bridge.EndIdx = value; }
            public int BridgedVtxCount(int vtxCnt) => ((IConvexHullBridge)bridge).BridgedVtxCount(vtxCnt);
            public Segment2d BridgeSegment(IList<Vector2d> vertices) => ((IConvexHullBridge)bridge).BridgeSegment(vertices);

            public int SourcePoly { get; set; }
            public int TargetPoly { get; set; }
            public bool IsSinglePolyBridge => SourcePoly == TargetPoly;
        }

        public class BridgedPocket
        {
            /// <summary>
            /// The polygon formed by the pocket.
            /// </summary>
            public Polygon2d Polygon { get; init; }
            /// <summary>
            /// All bridges that are part of this pocket. Each bridge contributes exactly one edge to the pockets polygon.
            /// </summary>
            public IReadOnlyList<MultiPolyBridge> Bridges { get; init; }

            /// <summary>
            /// Orders a bridge result for a convex hullIndices of multiple polygons into pockets.
            /// A pocket is the connected interior of the convex hullIndices, consisting of polygon edges inside the convex hullIndices
            /// and one or multiple bridges that skip vertices or connect different polygons to form the convex hullIndices.
            /// </summary>
            /// <param name="polygons"></param>
            /// <param name="bridges"></param>
            /// <returns></returns>
            public static List<BridgedPocket> GetPockets(IReadOnlyList<Polygon2d> polygons, IReadOnlyList<MultiPolyBridge> bridges, double colinearTolerance = 1e-3)
            {
                //the MultiPolyBridge results [bridges] are generated by GetBridges(IEnumerable<Polygon2d> polygons)
                //this works by flattening all polygon vertices into a single vertex list and then
                //using the default QuickHull implementation in ConvexHull2 that works on a point cloud.
                if (bridges.Count == 0)
                {
                    return [];
                }

                List<BridgedPocket> pockets = new();
                Polygon2d pocketPolygon = new Polygon2d();

                //offset of the polygons needed to convert global indices to local ones
                List<int> polyOffset = new List<int>();
                int offset = 0;
                foreach (Polygon2d polygon in polygons)
                {
                    polyOffset.Add(offset);
                    offset += polygon.VertexCount;
                }

                Stack<MultiPolyBridge> bridgesStack = new Stack<MultiPolyBridge>();

                foreach (MultiPolyBridge bridge in bridges)
                {
                    bridgesStack.Push(bridge);
                    if (bridgesStack.Any(b => b.SourcePoly == bridge.TargetPoly))
                    {
                        MultiPolyBridge multiPolyBridge = bridgesStack.First(b => b.SourcePoly == bridge.TargetPoly);
                        pockets.Add(ExtractPocket(bridgesStack, polygons, polyOffset, multiPolyBridge));
                    }
                }
                return pockets;
            }
        }

        private static BridgedPocket ExtractPocket(Stack<MultiPolyBridge> bridgesStack, IReadOnlyList<Polygon2d> polygons, List<int> polyOffset, MultiPolyBridge startBridge, double colinearTolerance = 1e-3)
        {
            List<MultiPolyBridge> bridgesForPocket = new List<MultiPolyBridge> { };
            var pocketPolygon = new Polygon2d();
            Stack<Vector2d> pocketVertices = new Stack<Vector2d>();
            List<BridgedPocket> pockets = new();
            int startPolyIdx = startBridge.SourcePoly;

            MultiPolyBridge curBridge;
            int localEnd = startBridge.StartIdx - polyOffset[startBridge.SourcePoly];
            do
            {
                // Take the next bridge
                curBridge = bridgesStack.Pop();
                bridgesForPocket.Add(curBridge);
                // Extract local indices for source and target polygons
                int curLocalEnd = curBridge.EndIdx - polyOffset[curBridge.TargetPoly];
                int sourcePolyIdx = curBridge.SourcePoly;
                int targetPolyIdx = curBridge.TargetPoly;
                int localStart = curBridge.StartIdx - polyOffset[sourcePolyIdx] >= 0 ?
                    curBridge.StartIdx - polyOffset[sourcePolyIdx] : polygons[sourcePolyIdx].VertexCount - polyOffset[sourcePolyIdx];

                int vertexCount = polygons[targetPolyIdx].VertexCount;
                int i = localEnd;
                
                              
                pocketPolygon.AppendVertex(polygons[targetPolyIdx].Vertices[i]);

                do
                {
                    i = (i + 1 < vertexCount) ? (i + 1) : 0;
                    pocketPolygon.AppendVertex(polygons[targetPolyIdx].Vertices[i]);
                        
                } while (i != curLocalEnd);

                localEnd = localStart;

            } while (curBridge.SourcePoly != startPolyIdx);


            var pocket = new BridgedPocket()
            {
                Bridges = bridgesForPocket,
                Polygon = pocketPolygon
            };
            
            return pocket;
        }

        
        /// <summary>
        /// Search all bridges where the convex hullIndices closes a concavity
        /// </summary>
        /// <param name="poly"></param>
        public static BridgeResult<T> GetBridges<T>(Polygon2d polygon, Func<Polygon2d, Vector2i, IReadOnlyList<Vector2d>, T> bridgeFactory) where T : IConvexHullBridge
        {
            if (polygon.IsConvex())
                return new BridgeResult<T>() { Polygon = polygon };

            var hull = Polygon2dExtensions.CreateConvexHull2(polygon, out var maybeMovedVertices);
            var bridges = new List<T>();
            var result = new BridgeResult<T>()
            {
                Polygon = polygon,
                Bridges = bridges,
                ConvexHull = hull,
            };

            if (hull.Dimension < 2)
                return result;
            var indices = hull.HullIndices;

            int lastIdx = indices[^1];
            for (int i = 0; i < indices.Length; i++)
            {
                int idx = indices[i];
                //account for wrap around
                int nextIdx = lastIdx + 1;
                nextIdx = nextIdx < polygon.VertexCount ? nextIdx : 0;
                if (idx != nextIdx)
                {
                    var bridge = bridgeFactory(polygon, new Vector2i(lastIdx, idx), maybeMovedVertices);
                    if (bridge != null)
                        bridges.Add(bridge);
                    lastIdx = idx;
                }
                else
                {
                    lastIdx = nextIdx;
                }
            }
            bridges.SortBy(x => x.StartIdx);

            return result;
        }

        public static BridgeResult<MultiPolyBridge> GetBridges(IEnumerable<Polygon2d> polygons, double colinearTolerance = ConvexHull2.DefaultConvexHullTolerance)
        {
            //init buffers
            int vtxCnt = polygons.Sum(x => x.VertexCount);
            Vector2d[] vertices = new Vector2d[vtxCnt];
            int pCount = polygons.Count() + 1;//add 1 to query start and end of poly in the same way
            if (pCount > ushort.MaxValue)
                throw new OverflowException("polygons count exceeds max (65535)");
            Span<ushort> polygonMapping = vtxCnt < 512 ? stackalloc ushort[vtxCnt] : new ushort[vtxCnt];
            Span<int> polyOffsets = pCount < 256 ? stackalloc int[pCount] : new int[pCount];

            int polyIdx = 0;
            int polyOffset = 0;
            foreach (var polygon in polygons)
            {
                polygon.VerticesAsReadOnlySpan.CopyTo(vertices.AsSpan().Slice(polyOffset));
                int pVtxCnt = polygon.VertexCount;
                polygonMapping.Slice(polyOffset, pVtxCnt).Fill((ushort)polyIdx);
                polyOffsets[polyIdx++] = polyOffset;
                polyOffset += pVtxCnt;
            }
            polyOffsets[^1] = polyOffset;

            var hull = new ConvexHull2(vertices, colinearTolerance);
            var bridges = new List<MultiPolyBridge>();
            var result = new BridgeResult<MultiPolyBridge>()
            {
                Vertices = vertices,
                Bridges = bridges,
                ConvexHull = hull,
            };

            if (hull.Dimension < 2)
                return result;

            ReadOnlySpan<int> indices = hull.HullIndices.AsSpan();

            //find index gaps
            int lastIdx = indices[^1];
            int lastPoly = polygonMapping[lastIdx];
            for (int i = 0; i < indices.Length; i++)
            {
                int idx = indices[i];
                int curPoly = polygonMapping[idx];
                //account for wrap around
                int nextIdxExpected = lastIdx + 1;
                nextIdxExpected = nextIdxExpected < vertices.Length ? nextIdxExpected : 0;
                if (polygonMapping[nextIdxExpected] != lastPoly)
                {
                    //poly wrap around
                    nextIdxExpected = polyOffsets[lastPoly];
                }

                if (idx != nextIdxExpected || lastPoly != curPoly)
                {
                    var bridge = new MultiPolyBridge() { StartIdx = lastIdx, EndIdx = idx, SourcePoly = lastPoly, TargetPoly = curPoly };
                    Vector2d bridgeDir = new Vector2d(vertices[idx] - vertices[lastIdx]).Normalized;
                    Vector2d sourceDir = new Vector2d(vertices[nextIdxExpected] - vertices[lastIdx]).Normalized;

                    int targetPrevIdx = idx - 1;// >= 0 ? idx - 1 : vertices.Count - 1;
                    if(targetPrevIdx < 0 || polygonMapping[targetPrevIdx] != bridge.TargetPoly)
                    {
                        //poly wrap around
                        targetPrevIdx = polyOffsets[bridge.TargetPoly + 1] - 1;
                    }
                    Vector2d targetDir = new Vector2d(vertices[idx] - vertices[targetPrevIdx]).Normalized;
                    //Vector2d middleEdge = new Vector2d(vertices[idx] - vertices[targetPrevIdx]).Normalized;

                    if (Math.Abs(bridgeDir.AngleR(sourceDir)) < colinearTolerance)
                        bridge.StartIdx = nextIdxExpected;

                    if ((nextIdxExpected != targetPrevIdx) && Math.Abs(bridgeDir.AngleR(targetDir)) < colinearTolerance)
                        bridge.EndIdx = targetPrevIdx;

                    //if (Math.Abs(sourceDir.AngleR(middleEdge) - Math.PI) < colinearTolerance)
                    //{
                    //    bridge.EndIdx = targetPrevIdx;
                    //    bridge.StartIdx = lastIdx + 1 < vertices.Count ? lastIdx + 1 : 0;
                    //}
                    bridges.Add(bridge);
                    lastIdx = idx;
                    lastPoly = curPoly;
                }
                else
                {
                    lastIdx = nextIdxExpected;
                }
            }

            return result;
        }
    }
}