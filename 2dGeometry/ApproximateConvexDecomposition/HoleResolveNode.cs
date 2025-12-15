using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace g3
{
    public static partial class ApproximateConvexDecomposition
    {
        private class HoleResolveNode
        {
            public HoleResolveNode parent;
            public readonly GeneralPolygon2d genPoly;

            public readonly Polygon2d polygon;
            public readonly List<HoleResolveNode> children = new List<HoleResolveNode>();

            public int holeIdx;

            public int resolvingHoleIdx;
            public int parentBridgeAnchorIdx;
            public int thisBridgeAnchorIdx;
            public double bridgeLength;
            public Vector2d VertexToInsert = Vector2d.MaxValue;

            public HoleResolveNode(GeneralPolygon2d genPoly, Polygon2d polygon)
            {
                this.genPoly = genPoly;
                this.polygon = polygon;
            }

            public List<int> OccupiedIndices
            {
                get
                {
                    var result = children.Select(x => x.parentBridgeAnchorIdx).ToList();
                    if (holeIdx != -1) result.Add(thisBridgeAnchorIdx);
                    return result;
                }
            }

            public IEnumerable<Segment2d> AllBridges
            {
                get
                {
                    var root = Root;
                    return root.Iterator.Skip(1).Select(x => x.BridgeSegment);
                }
            }

            public Segment2d BridgeSegment => new Segment2d(polygon[thisBridgeAnchorIdx], parent.polygon[parentBridgeAnchorIdx]);
            public PolyLine2d BridgePolyline => BridgeSegment.ToPolyLine();

            public HoleResolveNode ClosestConvexCutLine(int iHole, bool checkOtherHoleCollisions = true)
            {
                Debug.Assert(resolvingHoleIdx != iHole);

                var hole = genPoly.Holes[iHole];

                int iBestTarget = -1;
                int iBestSupport = -1;
                double minDistance = double.MaxValue;
                var occupiedIndices = OccupiedIndices;
                var bridges = AllBridges.ToArray();

                for (int supportVtxIdx = 0; supportVtxIdx < hole.VertexCount; supportVtxIdx++)
                {
                    var support = hole[supportVtxIdx];
                    for (int targetVtxIdx = 0; targetVtxIdx < polygon.Vertices.Count; targetVtxIdx++)
                    {
                        if (occupiedIndices?.Contains(targetVtxIdx) == true)
                            continue;
                        Vector2d target = polygon.Vertices[targetVtxIdx];
                        var distance = target.DistanceSquared(support);
                        if (distance >= minDistance)
                            continue;
                        //check coincidence, which is the perfect cut line :-)
                        if (distance < MathUtil.ZeroTolerance)
                        {
                            minDistance = distance;
                            iBestTarget = targetVtxIdx;
                            iBestSupport = supportVtxIdx;
                            goto returnResult;
                        }
                        //check the cut line validity
                        var sideAtOuter = polygon.WhichSideAt(targetVtxIdx, support);
                        if (sideAtOuter < 1)
                            continue;
                        var sideAtHole = hole.WhichSideAt(supportVtxIdx, target);
                        if (sideAtHole < 1)
                            continue;
                        if (IsIntersecting(polygon, targetVtxIdx, support))
                            continue;
                        if (IsIntersecting(hole, supportVtxIdx, target))
                            continue;

                        var cutLineSeg = new Segment2d(polygon[targetVtxIdx], hole[supportVtxIdx]);
                        if (checkOtherHoleCollisions && IntersectsAnyHoles(genPoly, cutLineSeg, iHole, holeIdx))
                            continue;
                        foreach (var bridgeSegment in bridges)
                        {
                            if (cutLineSeg.Intersects(bridgeSegment, intervalThresh: Constants.DEFAULT_LINE_IDENTITY_TOLERANCE))
                                goto continueOuter;
                        }

                        minDistance = distance;
                        iBestTarget = targetVtxIdx;
                        iBestSupport = supportVtxIdx;

                    continueOuter:;
                    }
                }

                //found something?
                if (iBestTarget == -1)
                    return null;

                returnResult:
                return new HoleResolveNode(genPoly, hole)
                {
                    parent = this,
                    resolvingHoleIdx = resolvingHoleIdx,
                    parentBridgeAnchorIdx = iBestTarget,
                    bridgeLength = Math.Sqrt(minDistance),
                    thisBridgeAnchorIdx = iBestSupport,
                    holeIdx = iHole,
                };
            }

            public HoleResolveNode NearestNeighborVertex(int iHole)
            {
                var hole = genPoly.Holes[iHole];

                int resolvingHoleIdx = -2;
                double minDistance = double.MaxValue;
                int iNearSeg = -1;
                int thisBridgeAnchorIdx = -1;
                var bridges = AllBridges.ToArray();
                HoleResolveNode parent = null;
                var aabbCenter = hole.Bounds.Center;
                Vector2d intersectionPoint = Vector2d.MaxValue;
                double bridgeLength = 0;

                foreach (var node in Iterator.OrderBy(x => (x.polygon.Bounds.Center - aabbCenter).LengthSquared))
                {
                    int holeToSearch = node.holeIdx;
                    if (holeToSearch == iHole) continue;
                    var occupiedIndices = node.OccupiedIndices;
                    var curHole = node.polygon;
                    var vertices = curHole.Vertices;
                    if (occupiedIndices.Count == vertices.Count) continue;

                    for (int j = 0; j < hole.Vertices.Count; j++)
                    {
                        var vtx = hole[j];
                        for (int vi = 0; vi < vertices.Count; ++vi)
                        {
                            //if (occupiedIndices.Contains(vi))
                            //    continue;
                            Segment2d seg = curHole.Segment(vi);;
                            double t = (vtx - seg.Center).Dot(seg.Direction);
                            double d = double.MaxValue;
                            if (t >= seg.Extent)
                                d = seg.P1.DistanceSquared(vtx);
                            else if (t <= -seg.Extent)
                                d = seg.P0.DistanceSquared(vtx);
                            else
                                d = (seg.PointAt(t) - vtx).LengthSquared;

                            if (d < minDistance)
                            {
                                //check if this is a valid cut line (not intersecting anything)
                                int segmentToIgnore = (vi + 1) % vertices.Count;
                                var bisectorLine2 = new Line2d(hole[j], hole.GetNormal_FaceAvg(j));
                                bool isOnVertex = false;
                                var intersectionPoint2 = bisectorLine2.IntersectionPoint(seg);
                                if (intersectionPoint2 == Vector2d.MaxValue)
                                {
                                    isOnVertex = true;
                                    intersectionPoint2 = seg.NearestPoint(hole[j]);
                                    segmentToIgnore = intersectionPoint2 == seg.P0 ? vi : segmentToIgnore;
                                }
                                if (isOnVertex && occupiedIndices.Contains(segmentToIgnore))
                                    continue;
                                var cutLineSeg = new Segment2d(intersectionPoint2, hole[j]);
                                if(IsIntersecting(curHole, cutLineSeg, segmentToIgnore))
                                    goto ContinueOuter;
                                foreach (var bridgeSegment in bridges)
                                    if (cutLineSeg.Intersects(bridgeSegment))
                                        goto ContinueOuter;
                                if (IntersectsAnyHoles(genPoly, cutLineSeg, iHole, holeToSearch))
                                    goto ContinueOuter;

                                minDistance = d;
                                iNearSeg = segmentToIgnore;
                                intersectionPoint = isOnVertex ? Vector2d.MaxValue : intersectionPoint2;
                                bridgeLength = cutLineSeg.Length;
                                resolvingHoleIdx = holeToSearch;
                                thisBridgeAnchorIdx = j;
                                parent = node;
                            }
                        ContinueOuter:;
                        }
                    }
                }

                if (resolvingHoleIdx < -1)
                    return null;

                var resolvingHole = genPoly.OuterOrHoleByIdx(resolvingHoleIdx);
                //var bisectorLine = new Line2d(hole[thisBridgeAnchorIdx], hole.GetNormal_FaceAvg(thisBridgeAnchorIdx));
                //var segment = resolvingHole.Segment(iNearSeg);
                //var intersectionPoint = bisectorLine.IntersectionPoint(segment);
                //if (intersectionPoint == Vector2d.MaxValue)
                //{
                //    //check if the nearest point is one of the vertices
                //    var p = hole[thisBridgeAnchorIdx];
                //    double t = (p - segment.Center).Dot(segment.Direction);
                //    if (t <= -segment.Extent + 0.1)
                //        vertexIdx = iNearSeg;
                //    //else if (t >= segment.Extent)
                //    //    ;//vertexIdx is already correct
                //    else if (t < segment.Extent - 0.1)
                //        intersectionPoint = segment.Center + t * segment.Direction;
                //}

                //var cutLineSeg = new Segment2d(intersectionPoint, hole[thisBridgeAnchorIdx]);

                return new HoleResolveNode(genPoly, hole)
                {
                    parent = parent,
                    resolvingHoleIdx = resolvingHoleIdx,
                    parentBridgeAnchorIdx = iNearSeg,
                    bridgeLength = bridgeLength,
                    thisBridgeAnchorIdx = thisBridgeAnchorIdx,
                    holeIdx = iHole,
                    VertexToInsert = intersectionPoint,
                };
            }

            public void InsertChildNode(HoleResolveNode childToInsert)
            {
                if (childToInsert.VertexToInsert != Vector2d.MaxValue)
                {
                    int insertIdx = childToInsert.parentBridgeAnchorIdx;
                    polygon.InsertVertex(insertIdx, childToInsert.VertexToInsert);
                    IncrementIndices(insertIdx);
                }
                children.Add(childToInsert);
            }

            private void IncrementIndices(int insertIdx)
            {
                if (holeIdx >= 0 && thisBridgeAnchorIdx >= insertIdx) thisBridgeAnchorIdx++;
                for (int j = 0; j < children.Count; j++)
                {
                    HoleResolveNode child = children[j];
                    if (child.parentBridgeAnchorIdx >= insertIdx)
                        child.parentBridgeAnchorIdx++;
                }
            }

            public IEnumerable<Vector2d> BridgedVertices
            {
                get
                {
                    if (holeIdx < 0) thisBridgeAnchorIdx = 0;
                    children.SortBy(x => x.parentBridgeAnchorIdx);
                    int childIdx = children.IndexOfFirst(x => x.parentBridgeAnchorIdx >= thisBridgeAnchorIdx);
                    if (childIdx == -1) childIdx = 0;
                    int visitedChildren = 0;
                    for (int i = 0; i < polygon.VertexCount; i++)
                    {
                        int shiftedI = (i + thisBridgeAnchorIdx) % polygon.VertexCount;
                        if (visitedChildren >= children.Count || children[childIdx % children.Count].parentBridgeAnchorIdx != shiftedI)
                        {
                            yield return polygon[shiftedI];
                        }
                        else
                        {
                            do
                            {
                                //create bridge to child
                                var child = children[childIdx % children.Count];
                                if (child.bridgeLength > MathUtil.ZeroTolerance) yield return polygon[shiftedI];
                                foreach (var vtx in child.BridgedVertices)
                                    yield return vtx;

                                //duplicate the two bridge vertices to emit the bridge return
                                if (child.bridgeLength > MathUtil.ZeroTolerance) yield return child.BridgedVertices.First();
                                yield return polygon[shiftedI];
                                childIdx++;
                                visitedChildren++;
                                //there might be more than one child attached to the same vertex
                            } while (visitedChildren < children.Count && children[childIdx % children.Count].parentBridgeAnchorIdx == shiftedI);
                        }
                    }
                }
            }

            public Polygon2d GetBridgedPolygon()
            {
                Polygon2d result = SharedPolyPool.Rent(genPoly.VertexCount + genPoly.Holes.Count * 2);
                result.AppendVertices(BridgedVertices);
                return result;
            }

            public int ChildCount
            {
                get
                {
                    return children.Count + children.Sum(x => x.ChildCount);
                }
            }

            public IEnumerable<HoleResolveNode> Iterator
            {
                get
                {
                    yield return this;
                    for (int i = 0; i < children.Count; i++)
                    {
                        HoleResolveNode child = children[i];
                        foreach (var grandChild in child.Iterator)
                            yield return grandChild;
                    }
                }
            }

            public HoleResolveNode Root => parent == null ? this : parent.Root;
        }
    }
}
