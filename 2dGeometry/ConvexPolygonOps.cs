using g3;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace g3
{
    public static class ConvexPolygonOps
    {
        /// <summary>
        /// Check if two convex polygons intersect using the Separating Axis Theorem (SAT) with tolerance.
        /// The polygons must be convex, which is asserted in Debug mode.
        /// </summary>
        /// <param name="convexPolyA"></param>
        /// <param name="convexPolyB"></param>
        /// <param name="tolerance"></param>
        /// <returns></returns>
        public static bool ConvexPolygonsIntersect(Polygon2d convexPolyA, Polygon2d convexPolyB, double tolerance = MathUtil.ZeroTolerance)
        {
            Debug.Assert(convexPolyA.IsConvex(tolerance: 0.0));
            Debug.Assert(convexPolyB.IsConvex(tolerance: 0.0));

            // Initialize support indices for both polygons for both max and min queries.
            int supportA_max = 0, supportA_min = 0;
            int supportB_max = 0, supportB_min = 0;

            ReadOnlySpan<Vector2d> vertA = convexPolyA.VerticesAsReadOnlySpan;
            ReadOnlySpan<Vector2d> vertB = convexPolyB.VerticesAsReadOnlySpan;
            // Test all axes defined by the edges of polyA and polyB.
            // Two passes: one for each polygon's edges.
            for (int pass = 0; pass < 2; pass++)
            {
                ReadOnlySpan<Vector2d> vertices = (pass == 0) ? vertA : vertB;
                Vector2d prev = vertices[^1];
                for (int i = 0; i < vertices.Length; i++)
                {
                    Vector2d vert = vertices[i];
                    Vector2d edge = vert - prev;
                    Vector2d axis = edge.UnitPerp;

                    // Find the extreme vertices along the axis for each polygon.
                    supportA_max = SupportMapping(vertA, axis, supportA_max);
                    supportA_min = SupportMapping(vertA, -axis, supportA_min);
                    supportB_max = SupportMapping(vertB, axis, supportB_max);
                    supportB_min = SupportMapping(vertB, -axis, supportB_min);

                    double a_max = vertA[supportA_max].Dot(axis);
                    double a_min = vertA[supportA_min].Dot(axis);
                    double b_max = vertB[supportB_max].Dot(axis);
                    double b_min = vertB[supportB_min].Dot(axis);

                    // Use the tolerance when comparing the intervals.
                    if (a_max < b_min + tolerance || b_max < a_min + tolerance)
                        return false;  // Separating axis found, so no intersection.

                    prev = vert;
                }
            }

            // No separating axis was found; the polygons intersect.
            return true;
        }

        /// <summary>
        /// SupportMapping: Finds the index of the vertex that has the maximum dot product with the given direction.
        /// 'startIndex' is used as a hint so the search can continue from where it left off.
        /// </summary>
        /// <param name="vertices"></param>
        /// <param name="dir"></param>
        /// <param name="startIndex"></param>
        /// <returns></returns>
        private static int SupportMapping(ReadOnlySpan<Vector2d> vertices, Vector2d dir, int startIndex)
        {
            int bestIndex = startIndex;
            double bestDot = vertices[bestIndex].Dot(dir);

            // Search forward
            while (true)
            {
                int nextIndex = bestIndex + 1;
                nextIndex = nextIndex < vertices.Length ? nextIndex : 0;
                double nextDot = vertices[nextIndex].Dot(dir);
                if (nextDot >= bestDot)
                {
                    bestIndex = nextIndex;
                    bestDot = nextDot;
                }
                else
                {
                    break;
                }
            }

            // if we successfully searched forward, no need to search backwards
            if (bestIndex != startIndex) { return bestIndex; }

            // Search backward
            while (true)
            {
                int nextIndex = bestIndex - 1;
                nextIndex = nextIndex >= 0 ? nextIndex : vertices.Length - 1;
                double nextDot = vertices[nextIndex].Dot(dir);
                if (nextDot >= bestDot)
                {
                    bestIndex = nextIndex;
                    bestDot = nextDot;
                }
                else
                {
                    break;
                }
            }
            return bestIndex;
        }

        public enum ContainmentType
        {
            /// <summary>
            /// Neither polygon is contained in the other.
            /// </summary>
            IntersectingOrSeparated,
            /// <summary>
            /// Polygon A is fully contained in polygon B (thus, B is outer).
            /// </summary>
            AInsideB,
            /// <summary>
            /// Polygon B is fully contained in polygon A (thus, A is outer).
            /// </summary>
            BInsideA,
            /// <summary>
            /// polygons are identical (within tolerance)
            /// </summary>
            Identical,
            /// <summary>
            /// both polygons are separated, thus do not intersect or contain each other
            /// </summary>
            Separated,
            /// <summary>
            /// an intersection between 2 line segments of the polygons has been found
            /// </summary>
            Intersecting,
            /// <summary>
            /// Indifferent result. Polygon might be contained inside a hole, doing a full winding number calculation using Clipper2 is recommended.
            /// (Or extending this method :-))
            /// </summary>
            IntersectingOrPolyTree,
            Failure,
        }

        public static ContainmentType IsContained(Polygon2d convexPolyA, Polygon2d convexPolyB, double tolerance = MathUtil.ZeroTolerancef)
            => IsContained(convexPolyA, convexPolyB, out _, out _, tolerance);

        /// <summary>
        /// Test if one of two convex polygons is fully contained inside the other.
        /// Returns indices of A and B where the check first failed.
        /// </summary>
        /// <param name="convexPolyA"></param>
        /// <param name="convexPolyB"></param>
        /// <param name="tolerance">tolerance for removing colinear edges from the NFP</param>
        public static ContainmentType IsContained(Polygon2d convexPolyA, Polygon2d convexPolyB, out Vector2i failIdxA, out Vector2i failIdxB, double tolerance = MathUtil.ZeroTolerancef)
        {
            Debug.Assert(convexPolyA.IsConvex());
            Debug.Assert(convexPolyB.IsConvex());
            ReadOnlySpan<Vector2d> vertsA = convexPolyA.VerticesAsReadOnlySpan;
            ReadOnlySpan<Vector2d> vertsB = convexPolyB.VerticesAsReadOnlySpan;
            if (vertsA.Length < 3 || vertsB.Length < 3)
                throw new ArgumentException("Both polygons must have at least 3 vertices");

            // find start points
            int idxA = IndexOfLowestLeft(vertsA);
            int idxB = IndexOfLowestLeft(vertsB);
            failIdxA = new Vector2i(-1);
            failIdxB = new Vector2i(-1);

            // Booleans for containment tests:
            // aInsideB means all vertices of A have passed half-plane tests of B's edges.
            // bInsideA means all vertices of B have passed half-plane tests of A's edges.
            // use negative tolerance to consider "on edge" vertices inside
            bool aInsideB = true, bInsideA = true;
            ConvexTwoEdgeState twoEdges = new(vertsA, vertsB, idxA, idxB, -tolerance);
            int a = 0, b = 0;
            while (a < vertsA.Length || b < vertsB.Length)
            {
                // does the current edge of A contain the current vertex of B?
                // For polygon B to be inside A, vertex B[iB] must lie on or inside the half‐plane of edge A[iA]→A[nextA].
                if (bInsideA && twoEdges.CurBOutsideEdgeA)
                {
                    bInsideA = false;
                    failIdxB = new Vector2i(twoEdges.IdxA, twoEdges.IdxB);
                    if (!aInsideB) break;
                }

                // does the current edge of B contain the current vertex of A?
                // For polygon A to be inside B, vertex A[iA] must lie on or inside the half‐plane of edge B[iB]→B[nextB].
                if (aInsideB && twoEdges.CurAOutsideEdgeB)
                {
                    aInsideB = false;
                    failIdxA = new Vector2i(twoEdges.IdxA, twoEdges.IdxB);
                    if (!bInsideA) break;
                }

                double crossEdges = twoEdges.CrossEdges;
                //if abs of cross is smaller than tolerance, we increment both indices
                //this skips 2 colinear edges
                if (crossEdges >= -tolerance)
                {
                    twoEdges.IncrA();
                    a++;
                }
                if (crossEdges <= tolerance)
                {
                    twoEdges.IncrB();
                    b++;
                }
            }

            return (aInsideB, bInsideA) switch
            {
                (true, true) => ContainmentType.Identical,
                (true, false) => ContainmentType.AInsideB,
                (false, true) => ContainmentType.BInsideA,
                (false, false) => ContainmentType.IntersectingOrSeparated,
            };
        }

        #region ConvexNFP
        /// <summary>
        /// Compute the no-fit polygon for moving movingPolygon around fixedPolygon.
        /// Reference is the coordinate origin of moving polygon, that means if you translate movingPolygon onto
        /// the NFP it will touch fixedPolygon but not intersect.
        /// Both input polygons must be convex.
        /// </summary>
        /// <param name="fixedPolygon"></param>
        /// <param name="movingPolygon"></param>
        /// <param name="tolerance">tolerance for removing colinear edges from the NFP</param>
        /// <returns>the no-fit polygon for moving movingPolygon around fixedPolygon</returns>
        public static Polygon2d ConvexOuterNfp(this Polygon2d fixedPolygon, Polygon2d movingPolygon, double tolerance = MathUtil.ZeroTolerancef)
        {
            Debug.Assert(fixedPolygon.IsConvex());
            Debug.Assert(movingPolygon.IsConvex());
            //compute Minkoswki Difference, which is the NFP
            //ported and adapted from https://cp-algorithms.com/geometry/minkowski.html
            int capacity = fixedPolygon.VertexCount + movingPolygon.VertexCount;
            Polygon2d nfp = SharedPolyPool.Rent(capacity);
            int mDifIdx = 0;
            Span<Vector2d> minkowskiDif = nfp.VerticesAsSpanWithCount(capacity);

            ReadOnlySpan<Vector2d> fixV = fixedPolygon.VerticesAsReadOnlySpan;
            ReadOnlySpan<Vector2d> movV = movingPolygon.VerticesAsReadOnlySpan;

            int offFix = IndexOfLowestLeft(fixV);
            //we avoid copy moving polygon and multiply it by -1, do the difference in place in the main loop
            //so here find offset of most top right vertex because its Minkoswki Difference -> -movingPolygon
            int offMov = IndexOfTopMostRight(movV);

            //main loop: since the shapes are convex, we can merge edges in order of polar angle
            int a = 0, b = 0;
            while (a < fixV.Length || b < movV.Length)
            {
                //this line is effectively the Minkowski difference, the rest is iteration
                minkowskiDif[mDifIdx++] = fixV[offFix] - movV[offMov];
                IncrementLowerCrossP(fixV, movV, ref offFix, ref offMov, ref a, ref b, tolerance);
            }

            //for each edge of fixedPolygon which is coincident or parallel with one edge of movingPolygon,
            //we will have 1 vertex less than expected
            nfp.OverrideVertexCount(mDifIdx);
            //if both extreme edges of the 2 inputs (bottommost/topmost) are horizontal,
            //we create a co-linear edge at index 0 that our lockstep pointers miss, remove it
            if (!nfp.IsVertexConvex(0, tolerance)) { nfp.RemoveVertex(0); }
            return nfp;
        }

        private static void IncrementLowerCrossP(ReadOnlySpan<Vector2d> vertsA, ReadOnlySpan<Vector2d> vertsB, ref int offA, ref int offB, ref int a, ref int b, double tolerance)
        {
            //move both indices (pointers) in lockstep
            //this results in traversal sorted by polar angle because both polygons are convex
            int nextOffFix = offA + 1;
            nextOffFix = nextOffFix < vertsA.Length ? nextOffFix : 0;
            int nextOffMov = offB + 1;
            nextOffMov = nextOffMov < vertsB.Length ? nextOffMov : 0;
            double cross = (vertsA[nextOffFix] - vertsA[offA]).Cross(vertsB[offB] - vertsB[nextOffMov]);
            //if abs of cross is smaller than tolerance, we increment both indices
            //this effectively skips a vertex in the output that creates a colinear edge
            if (cross >= -tolerance)
            {
                offA = nextOffFix;
                a++;
            }
            if (cross <= tolerance)
            {
                offB = nextOffMov;
                b++;
            }
        }

        private static int IndexOfTopMostRight(ReadOnlySpan<Vector2d> vertsB)
        {
            int idx = 0;
            double maxY = vertsB[idx].y;
            for (int i = 1; i < vertsB.Length; i++)
            {
                double thisY = vertsB[i].y;
                if (thisY < maxY) { continue; }
                if (thisY > maxY) { idx = i; maxY = thisY; continue; }
                //thisY == maxY
                if (vertsB[i].x > vertsB[idx].x) { idx = i; }
            }
            return idx;
        }

        private static int IndexOfLowestLeft(ReadOnlySpan<Vector2d> vertsB)
        {
            int idx = 0;
            double minY = vertsB[idx].y;
            for (int i = 1; i < vertsB.Length; i++)
            {
                double thisY = vertsB[i].y;
                if (thisY > minY) { continue; }
                if (thisY < minY) { idx = i; minY = thisY; continue; }
                //thisY == maxY
                if (vertsB[i].x < vertsB[idx].x) { idx = i; }
            }
            return idx;
        }

        /// <summary>
        ///// find offset of most bottom left vertex
        ///// </summary>
        ///// <param name="vertsA"></param>
        ///// <returns></returns>
        //private static int IndexOfLowestLeft(ReadOnlySpan<Vector2d> vertsA)
        //{
        //    int idx = 0;
        //    for (int i = 1; i < vertsA.Length; i++)
        //    {
        //        if (vertsA[i].y < vertsA[idx].y
        //        || (vertsA[i].y == vertsA[idx].y && vertsA[i].x < vertsA[idx].x))
        //        { idx = i; }
        //    }
        //    return idx;
        //}
        #endregion

        public enum UnionResult
        {
            AlgorithmFailed,
            Separated,
            Intersecting,
            FullContainment,
        }

        public struct PolyUnionResult
        {
            public readonly Polygon2d unionPoly;
            public readonly UnionResult result;

            public PolyUnionResult(UnionResult result, Polygon2d unionPoly)
            {
                this.unionPoly = unionPoly;
                this.result = result;
            }

            public static readonly PolyUnionResult Failed = new PolyUnionResult(UnionResult.AlgorithmFailed, null);
            public static readonly PolyUnionResult Separated = new PolyUnionResult(UnionResult.Separated, null);
        }

        public static PolyUnionResult ConvexPolygonUnion(Polygon2d convexPolyA, Polygon2d convexPolyB, double tolerance = Constants.DEFAULT_LINE_IDENTITY_TOLERANCE, Polygon2d buffer = null)
        {
            Debug.Assert(convexPolyA.IsConvex(tolerance: 0.0));
            Debug.Assert(convexPolyB.IsConvex(tolerance: 0.0));
            Debug.Assert(!ReferenceEquals(convexPolyA, buffer));
            Debug.Assert(!ReferenceEquals(convexPolyB, buffer));

            // do rotating calipers traversal
            // finds the first vertex outside of a convex polygon (or proves full containment)
            ContainmentType result = IsContained(convexPolyA, convexPolyB, out Vector2i failIdxA, out Vector2i failIdxB, tolerance);
            switch (result)
            {
                case ContainmentType.IntersectingOrSeparated:
                    break;
                case ContainmentType.AInsideB:
                    return new PolyUnionResult(UnionResult.FullContainment, convexPolyB);
                case ContainmentType.BInsideA:
                case ContainmentType.Identical:
                    return new PolyUnionResult(UnionResult.FullContainment, convexPolyA);
            }

            // the method structure follows a basic O(n²) intersection test
            // but we try to skip whole loop iterations by exploiting the convexity property
            // if we can prove that both vertices of an edge are on the "outside" side of any convex polygon edge
            // the edge is not intersecting the convex polygon
            // this *should* result in linear runtime, because 2 convex polygons have a maximum of 4 intersection points
            ReadOnlySpan<Vector2d> vertsA = convexPolyA.VerticesAsReadOnlySpan;
            ReadOnlySpan<Vector2d> vertsB = convexPolyB.VerticesAsReadOnlySpan;
            int startA = failIdxA.x;
            int startB = failIdxA.y;
            ConvexTwoEdgeState twoEdges = new(vertsA, vertsB, startA, startB, tolerance: 0.0);

            Debug.Assert(!vertsB.Contains(vertsA[startA]));
            Debug.Assert(twoEdges.CurAOutsideEdgeB);
            bool followingA = true;
            bool foundIntersections = false;
            Segment2d segA, segB;
            double t1, t2;
            SearchDir searchDir = SearchDir.UNDETERMINED;
            buffer?.ClearVertices();
            Polygon2d union = buffer ?? new();
            union.AppendVertex(twoEdges.CurA);
            //2 convex polygons can have a maximum of 4 intersection points
            int maxVertexCount = convexPolyA.VertexCount * convexPolyB.VertexCount * 2;
            int a = 0, b = 0, curSearch = 0;
            do
            {
                if (union.VertexCount > maxVertexCount) return PolyUnionResult.Failed;
                if (followingA)
                {
                    while (twoEdges.NextAOutsideEdgeB)
                    {
                        union.AppendVertex(twoEdges.NextA);
                        a++;
                        twoEdges.IncrA();
                        searchDir = SearchDir.UNDETERMINED;
                        if (twoEdges.NextIdxA == startA && twoEdges.NextAOutsideEdgeB) goto breakOuter;
                    }
                }
                else
                {
                    while (twoEdges.NextBOutsideEdgeA)
                    {
                        union.AppendVertex(twoEdges.NextB);
                        b++;
                        twoEdges.IncrB();
                        searchDir = SearchDir.UNDETERMINED;
                        if (b > vertsB.Length) return PolyUnionResult.Failed;
                    }
                }

                segA = twoEdges.SegA;
                segB = twoEdges.SegB;
                bool intersects = segA.Intersects(segB, out t1, out t2, intervalThresh: tolerance);
                if (intersects)
                {
                    bool onAnyVertex;
                    if (followingA) onAnyVertex = !twoEdges.NextBOutsideEdgeA;
                    else onAnyVertex = !twoEdges.NextAOutsideEdgeB;
                    bool onVertexA = Math.Abs(Math.Abs(t1) - segA.Extent) < tolerance;
                    bool onVertexB = Math.Abs(Math.Abs(t2) - segB.Extent) < tolerance;
                    onAnyVertex |= onVertexA;
                    onAnyVertex |= onVertexB;

                    bool reallyIntersects = true;
                    if (!onAnyVertex)
                    {
                        Vector2d intersectionPoint = segA.PointAt(t1);
                        union.AppendVertex(intersectionPoint);
                    }
                    else
                    {
                        if(!onVertexA && !onVertexB) 
                        {
                            onVertexA = followingA;
                            onVertexB = !followingA;
                        }
                        // this code handles both segment-vertex (onVertexA ^ onVertexB)
                        // and vertex-vertex (onVertexA && onVertexB) intersections
                        // we do local search on the polygon we don't follow (A or B)
                        // => we might arrive at the intersection "backwards"
                        // => first create the same initial state, if resp. t<0 / t>0
                        if (onVertexA & t1 < 0) twoEdges.DecrA();
                        if (onVertexB & t2 < 0) twoEdges.DecrB();
                        bool bOutsideBefore, bOutsideAfter;
                        if (Math.Abs(twoEdges.CrossEdges) < tolerance)
                        {
                            //segments are co-linear
                            if (onVertexA) twoEdges.IncrA();
                            if (onVertexB) twoEdges.IncrB();
                            reallyIntersects = followingA ? twoEdges.NextBOutsideEdgeA : twoEdges.NextAOutsideEdgeB;                            
                        }
                        else if (onVertexA ^ onVertexB)
                        {
                            //segment-vertex intersection
                            if (onVertexA)
                            {
                                bOutsideBefore = twoEdges.CurAOutsideEdgeB;
                                twoEdges.IncrA();
                                bOutsideAfter = twoEdges.NextAOutsideEdgeB;
                                reallyIntersects = bOutsideBefore != bOutsideAfter;
                                if (reallyIntersects | followingA)
                                {
                                    union.AppendVertex(twoEdges.CurA);
                                    a++;
                                }
                                else
                                {
                                    //we have proven the segment intersects the convex polygon on a vertex only,
                                    //because of the convexity there can't be any other intersections
                                    twoEdges.IncrB();
                                    union.AppendVertex(twoEdges.CurB);
                                    b++;
                                }
                            }
                            else//onVertexB
                            {
                                bOutsideBefore = twoEdges.CurBOutsideEdgeA;
                                twoEdges.IncrB();
                                bOutsideAfter = twoEdges.NextBOutsideEdgeA;
                                reallyIntersects = bOutsideBefore != bOutsideAfter;
                                if (reallyIntersects | !followingA)
                                {
                                    union.AppendVertex(twoEdges.CurB);
                                    b++;
                                }
                                else
                                {
                                    //we have proven the segment intersects the convex polygon on a vertex only,
                                    //because of the convexity there can't be any other intersections
                                    if (twoEdges.NextIdxA == startA) goto breakOuter;
                                    twoEdges.IncrA();
                                    union.AppendVertex(twoEdges.CurA);
                                    a++;
                                }
                            }
                        }
                        else
                        {
                            //vertex-vertex intersection. We have to check both edges,
                            //if the other vertex is outside either one, it is outside
                            bOutsideBefore = twoEdges.CurBOutsideEdgeA;
                            twoEdges.IncrA();
                            bOutsideBefore = bOutsideBefore | twoEdges.CurBOutsideEdgeA;
                            twoEdges.IncrB();
                            bOutsideAfter = twoEdges.NextBOutsideEdgeA;
                            twoEdges.DecrA();
                            bOutsideAfter = bOutsideAfter | twoEdges.NextBOutsideEdgeA;
                            twoEdges.IncrA();
                            reallyIntersects = bOutsideBefore != bOutsideAfter;
                            if (reallyIntersects ^ followingA)
                            {
                                union.AppendVertex(twoEdges.CurA);
                                a++;
                            }
                            else
                            {
                                union.AppendVertex(twoEdges.CurB);
                                b++;
                            }
                        }

                        // traverser co-linear segments until the intersection is resolved
                        if (Math.Abs(twoEdges.CrossEdges) < tolerance)
                        {
                            do
                            {
                                double lenA = twoEdges.EdgeA.LengthSquared;
                                double lenB = twoEdges.EdgeB.LengthSquared;
                                twoEdges.IncrA();
                                twoEdges.IncrB();
                                if (twoEdges.IdxA == startA) goto breakOuter;
                                if (Math.Abs(lenA - lenB) < tolerance)
                                {
                                    union.AppendVertex(twoEdges.CurA);//is coincident with curB
                                    a++;
                                    b++;
                                    if (followingA) { reallyIntersects = !twoEdges.NextAOutsideEdgeB; }
                                    else { reallyIntersects = !twoEdges.NextBOutsideEdgeA; }
                                }
                                else if (lenA > lenB)
                                {
                                    union.AppendVertex(twoEdges.CurA);
                                    a++;
                                    reallyIntersects = !followingA;
                                    break;
                                }
                                else
                                {
                                    union.AppendVertex(twoEdges.CurB);
                                    b++;
                                    reallyIntersects = followingA;
                                    break;
                                }
                            } while (Math.Abs(twoEdges.CrossEdges) < tolerance);
                        }
                        else if (!reallyIntersects & onVertexA & onVertexB)
                        {
                            if (followingA)
                            {
                                if (twoEdges.NextIdxA == startA) goto breakOuter;
                                twoEdges.IncrA();
                                union.AppendVertex(twoEdges.CurA);
                                a++;
                            }
                            else
                            {
                                twoEdges.IncrB();
                                union.AppendVertex(twoEdges.CurB);
                                b++;
                            }
                        }
                    }

                    if (curSearch > maxVertexCount) return PolyUnionResult.Failed;
                    curSearch++;

                    if (reallyIntersects)
                    {
                        foundIntersections = true;
                        searchDir = SearchDir.UNDETERMINED;
                        followingA = !followingA;
                        if (followingA && twoEdges.NextIdxA == startA) goto breakOuter;
                    }
                    continue;
                }

                if (searchDir == SearchDir.UNDETERMINED)
                {
                    double curT = followingA ? t2 : t1;
                    searchDir = curT >= 0 ? SearchDir.FORWARD : SearchDir.BACKWARD;
                    curSearch = 0;
                }
                if(curSearch > (followingA ? vertsB.Length : vertsA.Length)) return PolyUnionResult.Failed;
                curSearch++;
                if (searchDir == SearchDir.BACKWARD)
                {
                    if (followingA)
                        twoEdges.DecrB();
                    else
                        twoEdges.DecrA();
                }
                else
                {
                    if (followingA)
                        twoEdges.IncrB();
                    else
                        twoEdges.IncrA();
                }
            } while (twoEdges.NextIdxA != startA || !followingA || !twoEdges.NextAOutsideEdgeB);
        breakOuter:

#if DEBUG
            if (foundIntersections)
            {
                Polygon2d[] offsetInput = [convexPolyA, convexPolyB];
                var rest = union.ClipBy(offsetInput.Offset(1e-3));
                if (rest.Length > 0) { }
            }
#endif
            return foundIntersections ? new PolyUnionResult(UnionResult.Intersecting, union) : new PolyUnionResult(UnionResult.Separated, null);
        }

        public static ContainmentType ContainsConvex(this Polygon2d concavePolyA, Polygon2d convexPolyB, double tolerance = g3.MathUtil.ZeroTolerance)
        {
            Debug.Assert(convexPolyB.IsConvex(tolerance: 0.0));

            if (!concavePolyA.Bounds.Contains(convexPolyB.Bounds)) { return ContainmentType.IntersectingOrSeparated; }

            if (PointInConvexPolygon(convexPolyB, concavePolyA[0], out _, tolerance))
            { return ContainmentType.IntersectingOrSeparated; }

            bool intersects = concavePolyA.IntersectsConvex(convexPolyB, tolerance);
            if (intersects) return ContainmentType.Intersecting;

            //there are no intersections, check if separated or contained
            bool bInsideA = concavePolyA.Contains(convexPolyB[0]);
#if DEBUG
            if (bInsideA)
            {
                Polygon2d[] clip = convexPolyB.ClipBy(concavePolyA);
                if (clip.Length > 0) { throw new Exception(); }
            }
#endif
            if (bInsideA) return ContainmentType.BInsideA;
            else return ContainmentType.Separated;
        }

        public static ContainmentType ContainsConvex(this Polygon2d[] concavePolys, Polygon2d convexPolyB, double tolerance = g3.MathUtil.ZeroTolerance)
        {
            Debug.Assert(convexPolyB.IsConvex(tolerance: 0.0));
            if (concavePolys?.Length == 0) return ContainmentType.Separated;

            //start with bounds checks to find if there are any candidates based on bounds
            //before doing any expensive intersection checks
            List<Polygon2d> potentialOuters = null;
            bool hasHoles = false;
            AxisAlignedBox2d boundsB = convexPolyB.Bounds;
            foreach (Polygon2d concavePolyA in concavePolys)
            {
                if (concavePolyA.IsHole)
                {
                    hasHoles = true;
                    continue;
                }
                AxisAlignedBox2d boundsA = concavePolyA.Bounds;
                if (!boundsA.Contains(boundsB)) continue;
                potentialOuters ??= new();//lazy alloc
                potentialOuters.Add(concavePolyA);
            }
            if(!(potentialOuters?.Count > 0)) return ContainmentType.IntersectingOrSeparated;

            if (hasHoles)
            {
                //the holes are likely less complex then the outers, so process them first
                foreach (Polygon2d concavePolyA in concavePolys)
                {
                    if (!concavePolyA.IsHole) continue;
                    AxisAlignedBox2d boundsA = concavePolyA.Bounds;
                    if (!boundsA.Intersects(boundsB)) continue;
                    if (concavePolyA.ContainsInclusive(convexPolyB[0])) return ContainmentType.IntersectingOrPolyTree;
                    bool intersects = concavePolyA.IntersectsConvex(convexPolyB, tolerance);
                    if (intersects) return ContainmentType.Intersecting;
                    //this PiP generates false negatives for multi layer general polygons...
                    //since our fallback is clipper2 that handles that lets "fix it later"
                    if (PointInConvexPolygon(convexPolyB, concavePolyA[0], out _, tolerance))
                        return ContainmentType.IntersectingOrPolyTree;
                }
            }

            foreach (Polygon2d concavePolyA in potentialOuters)
            {
                bool intersects = concavePolyA.IntersectsConvex(convexPolyB, tolerance);
                if (intersects) return ContainmentType.Intersecting;
                if (PointInConvexPolygon(convexPolyB, concavePolyA[0], out _, tolerance))
                    return ContainmentType.IntersectingOrPolyTree;
                //there are no intersections, check if separated or contained
                bool bInsideA = concavePolyA.Contains(convexPolyB[0]);
#if DEBUG
                if (bInsideA)
                {
                    Polygon2d[] clip = convexPolyB.ClipBy(concavePolyA);
                    if (clip.Length > 0 && clip.Sum(x => x.Area) > 1e-4) { concavePolyA.IntersectsConvex(convexPolyB, tolerance); throw new Exception(); }
                }
#endif
                if (bInsideA) return ContainmentType.BInsideA;
            }

            return ContainmentType.IntersectingOrSeparated;
        }

        public static bool IntersectsConvex(this Polygon2d concavePolyA, Polygon2d convexPolyB, double tolerance = g3.MathUtil.ZeroTolerance)
        {
            int startA = 0;
            int startB = 0;
            ReadOnlySpan<Vector2d> vertsA = concavePolyA.VerticesAsReadOnlySpan;
            ReadOnlySpan<Vector2d> vertsB = convexPolyB.VerticesAsReadOnlySpan;
            ConvexTwoEdgeState twoEdges = new(vertsA, vertsB, startA, startB, tolerance: 0.0);

            Span<Segment2d> segmentsB = vertsB.Length < 1024 ? stackalloc Segment2d[vertsB.Length] : new Segment2d[vertsB.Length];
            PrecalcSegments(vertsB, segmentsB);

            Segment2d segA, segB;
            double t1, t2;
            SearchDir searchDir = SearchDir.UNDETERMINED;

            int a = 0, b = 0, curSearch = 0;
            do
            {
                if (twoEdges.CurAOutsideOrOnEdgeB)
                {
                    Vector2d edgeB = twoEdges.EdgeB;
                    Vector2d curB = twoEdges.CurB;
                    while (edgeB.Cross(twoEdges.NextA - curB) < -tolerance)
                    {
                        twoEdges.IncrA(); a++;
                        searchDir = SearchDir.UNDETERMINED;
                        if (a >= vertsA.Length) return false;
                    }
                }

                segA = twoEdges.SegA;
                segB = segmentsB[twoEdges.IdxB];
                bool intersects = segA.Intersects(segB, out t1, out t2, intervalThresh: tolerance);
                if (intersects) { return true; }

                if (searchDir == SearchDir.UNDETERMINED)
                {
                    searchDir = t2 >= 0 ? SearchDir.FORWARD : SearchDir.BACKWARD;
                    curSearch = 0;
                }
                curSearch++;
                if (curSearch >= vertsB.Length)
                {
                    //does not intersect
                    twoEdges.IncrA(); a++;
                    b = searchDir == SearchDir.FORWARD ? b - vertsB.Length : b + vertsB.Length;
                    searchDir = SearchDir.UNDETERMINED;
                    curSearch = 0;
                }
                else if (searchDir == SearchDir.FORWARD)
                {
                    twoEdges.IncrB(); b++;
                }
                else
                {
                    twoEdges.DecrB(); b--;
                }
            } while (a < vertsA.Length);
            return false;
        }

        private static void PrecalcSegments(ReadOnlySpan<Vector2d> vertsB, Span<Segment2d> segmentsB)
        {
            for (int i = 0; i < vertsB.Length - 1; i++)
            {
                segmentsB[i] = new Segment2d(vertsB[i], vertsB[i + 1]);
            }
            segmentsB[^1] = new Segment2d(vertsB[^1], vertsB[0]);
        }

        private enum SearchDir
        {
            UNDETERMINED,
            FORWARD,
            BACKWARD
        }

        private ref struct ConvexTwoEdgeState
        {
            public readonly ReadOnlySpan<Vector2d> vertsA;
            public readonly ReadOnlySpan<Vector2d> vertsB;
            private int _idxA, _idxB;
            public int IdxA { get => _idxA; set { NextIdxA = value; IncrA(); } }
            public int IdxB { get => _idxB; set { NextIdxB = value; IncrB(); } }
            public int NextIdxA { get; private set; }
            public int NextIdxB { get; private set; }
            public readonly double tolerance;

            public Vector2d CurA => vertsA[IdxA];
            public Vector2d NextA => vertsA[NextIdxA];
            public Vector2d EdgeA => NextA - CurA;
            public Vector2d CurB => vertsB[IdxB];
            public Vector2d NextB => vertsB[NextIdxB];
            public Vector2d EdgeB => NextB - CurB;
            //mainly for debug view
            public PolyLine2d[] EdgesAsPolylines => [new PolyLine2d([CurA, NextA]), new PolyLine2d([CurB, NextB])];

            public ConvexTwoEdgeState(ReadOnlySpan<Vector2d> vertsA, ReadOnlySpan<Vector2d> vertsB, int startA, int startB, double tolerance) : this()
            {
                this.vertsA = vertsA;
                this.vertsB = vertsB;
                this.tolerance = tolerance;
                this.IdxA = startA;
                this.IdxB = startB;
            }

            //perf: ternary compiles to conditional move, faster than modulo (%)
            public void IncrA()
            {
                _idxA = NextIdxA;
                NextIdxA++;
                NextIdxA = NextIdxA < vertsA.Length ? NextIdxA : 0;
            }

            public void DecrA()
            {
                NextIdxA = _idxA;
                _idxA--;
                _idxA = _idxA >= 0 ? _idxA : vertsA.Length - 1;
            }

            public void IncrB()
            {
                _idxB = NextIdxB;
                NextIdxB++;
                NextIdxB = NextIdxB < vertsB.Length ? NextIdxB : 0;
            }

            public void DecrB()
            {
                NextIdxB = _idxB;
                _idxB--;
                _idxB = _idxB >= 0 ? _idxB : vertsB.Length - 1;
            }

            public bool CurBOutsideEdgeA => CrossCurBEdgeA < tolerance;
            public bool CurAOutsideEdgeB => CrossCurAEdgeB < tolerance;
            public bool NextBOutsideEdgeA => CrossNextBEdgeA < tolerance;
            public bool NextAOutsideEdgeB => CrossNextAEdgeB < tolerance;
            public bool CurBOutsideOrOnEdgeA => CrossCurBEdgeA < -tolerance;
            public bool CurAOutsideOrOnEdgeB => CrossCurAEdgeB < -tolerance;
            public bool NextBOutsideOrOnEdgeA => CrossNextBEdgeA < -tolerance;
            public bool NextAOutsideOrOnEdgeB => CrossNextAEdgeB < -tolerance;
            public double CrossEdges => EdgeA.Cross(EdgeB);
            public double CrossCurBEdgeA => Cross(CurA, NextA, CurB);
            public double CrossCurAEdgeB => Cross(CurB, NextB, CurA);
            public double CrossNextBEdgeA => Cross(CurA, NextA, NextB);
            public double CrossNextAEdgeB => Cross(CurB, NextB, NextA);
            public Segment2d SegA => new Segment2d(CurA, NextA);
            public Segment2d SegB => new Segment2d(CurB, NextB);

            public bool PointOutsideEdgeA(Vector2d point) => EdgeA.Cross(point - CurA) < tolerance;
            public bool PointOutsideOrOnEdgeA(Vector2d point) => EdgeA.Cross(point - CurA) <= -tolerance;
            public bool PointOutsideEdgeB(Vector2d point) => EdgeB.Cross(point - CurB) < tolerance;
            public bool PointOutsideOrOnEdgeB(Vector2d point) => EdgeB.Cross(point - CurB) <= -tolerance;

            public bool SegmentsIntersect => IntersectSegments(out _, out _);
            public bool IntersectSegments(out double t1, out double t2)
            {
                return SegA.IntersectsNotParallel(SegB, out t1, out t2, dotThresh: tolerance);
            }
        }

        /// <summary>
        /// Test whether point is inside polygon. Poly must be convex.
        /// </summary>
        private static bool PointInConvexPolygon(Polygon2d poly, Vector2d pt, out int idx, double tolerance = 1e-10)
        {
            var verts = poly.VerticesAsReadOnlySpan;
            idx = 0;
            do
            {
                Vector2d cur = verts[idx];
                int nextIdx = idx + 1;
                nextIdx = nextIdx < verts.Length ? nextIdx : 0;
                Vector2d next = verts[nextIdx];
                if (Cross(cur, next, pt) < -tolerance)
                    return false;

                //use modulo-indexing to iterate in a pseudo random fashion, likely speeds up the negative case
                idx = (idx + 31337) % verts.Length;
            } while (idx != 0);

            return true;
        }

        private static bool PointInConvexPolygon(ReadOnlySpan<Vector2d> verts, Vector2d pt, ref int idx, double tolerance = 1e-10)
        {
            int startIdx = idx;
            do
            {
                Vector2d cur = verts[idx];
                int nextIdx = idx + 1;
                nextIdx = nextIdx < verts.Length ? nextIdx : 0;
                Vector2d next = verts[nextIdx];
                if (Cross(cur, next, pt) < -tolerance)
                    return false;
                idx = nextIdx;
            } while (idx != startIdx);

            return true;
        }

        // Computes the cross product (b - a) x (c - a).
        // A positive result indicates that a, b, c are in counter clockwise order.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static double Cross(Vector2d a, Vector2d b, Vector2d c)
        {
            return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        }

        #region naive distance
        /// <summary>
        /// Calculate the minimum distance between two polygons by testing all vertices of p1
        /// against all segments of p2 and vice versa. Polygons must not overlap.
        /// </summary>
        /// <param name="polygon1"></param>
        /// <param name="polygon2"></param>
        /// <returns></returns>
        public static double NaiveDistance(Polygon2d polygon1, Polygon2d polygon2)
        {
            double minDistSquared = double.MaxValue;
            foreach (var vertex in polygon1.Vertices)
            {
                foreach (var segment in polygon2.SegmentItr())
                {
                    minDistSquared = Math.Min(minDistSquared, PointSegmentDistanceSquared(vertex, segment.P0, segment.P1));
                }
            }
            foreach (var vertex in polygon2.Vertices)
            {
                foreach (var segment in polygon1.SegmentItr())
                {
                    minDistSquared = Math.Min(minDistSquared, PointSegmentDistanceSquared(vertex, segment.P0, segment.P1));
                }
            }
            return Math.Sqrt(minDistSquared);
        }

        private static double PointSegmentDistanceSquared(Vector2d p, Vector2d a, Vector2d b)
        {
            Vector2d q = ProjectPointToSegment(p, a, b);
            return (p - q).LengthSquared;
        }

        private static Vector2d ProjectPointToSegment(Vector2d p, Vector2d a, Vector2d b)
        {
            double dx = b.x - a.x;
            double dy = b.y - a.y;

            if ((dx == 0) && (dy == 0)) return a; // a and b are the same point

            // Calculate the t that minimizes the distance
            double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / (dx * dx + dy * dy);

            // See if this represents one of the segment's end points or a point in the middle.
            if (t < 0) return a;
            else if (t > 1) return b;
            else return new Vector2d(a.x + t * dx, a.y + t * dy);
        }
        #endregion

        #region gjk distance
        /// <summary>
        /// Calculate the minimum distance between two polygons using the GJK algorithm.
        /// Polygons must be convex and must not overlap.
        /// </summary>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <returns></returns>
        public static double GJKDistance(Polygon2d polygon1, Polygon2d polygon2)
        {
            Debug.Assert(polygon1.IsConvex());
            Debug.Assert(polygon2.IsConvex());
            Debug.Assert(!ConvexPolygonsIntersect(polygon1, polygon2));

            float tolerance = 0.01f;
            var startDirection = polygon1.Centroid() - polygon2.Centroid();

            var simplex = new Simplex()
            {
                a = SupportFunction(polygon1, polygon2, startDirection),
                b = SupportFunction(polygon1, polygon2, -startDirection)
            };

            var d = ClosestPointToOrigin(simplex.a, simplex.b);

            double dMin = d.Length;

            var counter = 0;
            while (true)
            {
                d = -d;
                if (d.Length == 0)
                {
                    return 0;
                }

                var c = SupportFunction(polygon1, polygon2, d);
                var dc = c.Dot(d);
                var da = simplex.a.Dot(d);

                if (dc - da < tolerance)
                {
                    var distance = d.Length;
                    return distance;
                }

                var p1 = ClosestPointToOrigin(simplex.a, c);
                var p2 = ClosestPointToOrigin(c, simplex.b);

                if (p1.Length < p2.Length)
                {
                    simplex.b = c;
                    d = p1;
                }
                else
                {
                    simplex.a = c;
                    d = p2;
                }

                dMin = Math.Min(dMin, d.Length);

                if (counter > polygon1.VertexCount)
                {
                    return dMin; ;
                }
                counter++;
            }
        }

        private static Vector2d SupportFunction(Polygon2d polygon1, Polygon2d polygon2, Vector2d d)
        {
            var p1 = GetFarthestPointInDirection(polygon1, d);
            var p2 = GetFarthestPointInDirection(polygon2, -d);

            return p1 - p2;
        }

        private static Vector2d ClosestPointToOrigin(Vector2d a, Vector2d b)
        {
            var ab = b - a;
            var ao = Vector2d.Zero - a;

            var lerp = ao.Dot(ab) / ab.LengthSquared;
            lerp = Math.Clamp(lerp, 0, 1);

            var point = Vector2d.Lerp(a, b, lerp);
            return point;
        }

        // no idea what this is for
        //
        //private Vector2 GetDirection(Vector2 a, Vector2 b)
        //{
        //    var ab = b - a;
        //    var ao = Vector2.Zero - a;

        //    var v3A = new Vector3(a.X, a.Y, 0);
        //    var v3B = new Vector3(b.X, b.Y, 0);
        //    var v3AB = v3B - v3A;

        //    var test = Vector3.Cross(Vector3.Cross(v3AB, v3A), v3AB);

        //    return new Vector2(-(ab.X * ao.Y - ab.Y * ao.X) * ab.Y, (ab.X * ao.Y - ab.Y * ao.X) * ab.X);
        //}

        private static Vector2d GetFarthestPointInDirection(Polygon2d polygon, Vector2d direction)
        {
            var maxDistance = double.MinValue;
            var maxIndex = 0;

            for (int i = 0; i < polygon.VertexCount; i++)
            {
                var vertex = polygon[i];
                var distance = vertex.Dot(direction);
                if (distance > maxDistance)
                {
                    maxDistance = distance;
                    maxIndex = i;
                }
            }

            return polygon[maxIndex];
        }

        private struct Simplex
        {
            public Vector2d a;
            public Vector2d b;
        }
        #endregion



        /// <summary>
        /// Returns if the polygon is convex or concave
        /// </summary>
        /// <param name="polygon"></param>
        /// <returns></returns>
        public static bool IsConvex(this Polygon2d polygon, double tolerance = MathUtil.Epsilonf)
        {
            var vertices = polygon.VerticesAsReadOnlySpan;
            if (vertices.Length <= 3)
                return true;
            //do first wrap around already here
            Vector2d prev = vertices[^2];
            Vector2d center = vertices[^1];
            for (int i = 0; i < vertices.Length; i++)
            {
                Vector2d next = vertices[i];
                var cp = center - prev;
                var cn = center - next;
                double sign = cp.Perp.Dot(cn);

                if (sign <= tolerance)
                    return false;

                prev = center;
                center = next;
            }
            return true;
        }
    }
}
