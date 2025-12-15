using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Threading;

namespace g3
{
    public class Polygon2d : IDuplicatable<Polygon2d>, IEquatable<Polygon2d>
    {
        protected List<Vector2d> vertices;
        private double signedAreaCached;
        private AxisAlignedBox2d cachedBounds;

        public int Timestamp = 0;
        private volatile int boundsCachedTimeStamp = int.MinValue;
        private volatile int signedAreaTimestamp = int.MinValue;

        public Polygon2d()
        {
            vertices = new();
        }

        public Polygon2d(int verticesCapacity)
        {
            vertices = new(verticesCapacity);
        }

        public Polygon2d(Polygon2d copy)
        {
            vertices = [.. copy.vertices];

            Timestamp = copy.Timestamp;
            boundsCachedTimeStamp = copy.boundsCachedTimeStamp;
            signedAreaTimestamp = copy.signedAreaTimestamp;
            cachedBounds = copy.cachedBounds;
            signedAreaCached = copy.signedAreaCached;
        }

        public Polygon2d(IEnumerable<Vector2d> copy)
        {
            vertices = [.. copy];
        }

        public Polygon2d(Vector2d[] v)
        {
            vertices = [.. v];
        }

        public Polygon2d(VectorArray2d v)
        {
            vertices = [.. v.AsVector2d()];
        }

        public Polygon2d(double[] values)
        {
            int N = values.Length / 2;
            vertices = new List<Vector2d>(N);
            for (int k = 0; k < N; ++k)
                vertices.Add(new Vector2d(values[2 * k], values[2 * k + 1]));
        }

        public Polygon2d(Func<int, Vector2d> SourceF, int N)
        {
            vertices = new List<Vector2d>();
            for (int k = 0; k < N; ++k)
                vertices.Add(SourceF(k));
        }

        public virtual Polygon2d Duplicate()
        {
            Polygon2d copy = SharedPolyPool.Rent(vertices);
            copy.Timestamp = this.Timestamp;
            copy.boundsCachedTimeStamp = this.boundsCachedTimeStamp;
            copy.signedAreaTimestamp = this.signedAreaTimestamp;
            copy.cachedBounds = this.cachedBounds;
            copy.signedAreaCached = this.signedAreaCached;
            return copy;
        }

        public Polygon2d Reversed()
        {
            Polygon2d copy = Duplicate();
            copy.Reverse();
            return copy;
        }

        public Polygon2d(Vector2f[] v)
        {
            vertices = new List<Vector2d>(v.Length);
            foreach (var vertex in v)
            {
                vertices.Add(new Vector2d(vertex));
            }
        }

        public Vector2d this[int key]
        {
            get { return vertices[key]; }
            set { vertices[key] = value; Timestamp++; }
        }

        public Vector2d Start
        {
            get { return vertices[0]; }
        }

        public IReadOnlyList<Vector2d> Vertices
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => vertices;
        }

        public int VertexCount
        {
            get { return vertices.Count; }
        }

        /// <summary>
        /// Calculate the approximate size in memory (in bytes) of the polygon (on 64bit)
        /// </summary>
        /// <returns></returns>
        public int CalculateSize()
        {
            //fields: header + ListRef + (list obj + array header) + areaDouble + aabb + 3 timestamps
            int size = 16    + 8       + (16 + 8 + 2*4 + 24      ) + 8          + 4*8  + 3*4;
            size += vertices.Capacity * 16;//list contents
            return size;
        }

        public void AppendVertex(Vector2d v)
        {
            vertices.Add(v);
            Timestamp++;
        }
        public void AppendVertices(IEnumerable<Vector2d> v)
        {
            vertices.AddRange(v);
            Timestamp++;
        }

        public void RemoveVertex(int idx)
        {
            vertices.RemoveAt(idx);
            Timestamp++;
        }

        public void TrimVerticesTo(int newCount)
        {
            if (newCount >= VertexCount) return;
            vertices.TrimTo(newCount);
            Timestamp++;
        }

        public void SetVertices(List<Vector2d> newVertices, bool bTakeOwnership)
        {
            if (bTakeOwnership)
            {
                vertices = newVertices;
            }
            else
            {
                vertices.Clear();
                vertices.AddRange(newVertices);
            }
            Timestamp++;
        }

        public void RoundToFloatPrecision()
        {
            for (int i = 0; i < VerticesAsDoubleSpan.Length; i++)
            {
                VerticesAsDoubleSpan[i] = (float)VerticesAsDoubleSpan[i];
            }
        }

        public void Reverse()
        {
            vertices.Reverse();
            Timestamp++;
        }

        public Vector2d GetTangent(int i)
        {
            Vector2d next = vertices[(i + 1) % vertices.Count];
            Vector2d prev = vertices[i == 0 ? vertices.Count - 1 : i - 1];
            return (next - prev).Normalized;
        }

        /// <summary>
        /// Normal at vertex i, which is perp to tangent direction, which is not so 
        /// intuitive if edges have very different lengths. 
        /// Points "inward" for clockwise polygon, outward for counter-clockwise
        /// </summary>
		public Vector2d GetNormal(int i)
        {
            return GetTangent(i).Perp;
        }

        /// <summary>
        /// Construct normal at poly vertex by averaging face normals. This is
        /// equivalent (?) to angle-based normal, ie is local/independent of segment lengths.
        /// Points "inward" for clockwise polygon, outward for counter-clockwise
        /// </summary>
        public Vector2d GetNormal_FaceAvg(int i)
        {
            (Vector2d prev, Vector2d next) = EdgesAt(i);

            next.Normalize();
            prev.Normalize();

            Vector2d n = next.Perp - prev.Perp;
            double len = n.Normalize();
            if (len == 0)
            {
                return (next + prev).Normalized;   // this gives right direction for degenerate angle
            }
            else
            {
                return n;
            }
        }

        /// <summary>
        /// Returns the angle between vertices at index i in degrees
        /// </summary>
        /// <param name="polygon"></param>
        /// <param name="i"></param>
        /// <returns></returns>
        public double AngleDAtVertex(int i)
        {
            (var cp, var cn) = EdgesAt(i);
            return cp.Normalized.AngleD(cn.Normalized);
        }

        public double AngleRAtVertex(int i)
        {
            (var cp, var cn) = EdgesAt(i);
            return cp.Normalized.AngleR(cn.Normalized);
        }

        /// <summary>
        /// Returns if the polygon is convex or concave at vertex i.
        /// </summary>
        /// <param name="polygon"></param>
        /// <param name="i"></param>
        /// <returns></returns>
        public bool IsVertexConvex(int i, double tolerance = MathUtil.Epsilonf)
        {
            (var cp, var cn) = EdgesAt(i);
            double sign = cp.Perp.Dot(cn);
            return sign > tolerance;
        }

        /// <summary>
        /// Returns the polygons previous and next edge (as Vector2d) at vertex i.
        /// </summary>
        /// <param name="i">vertex index</param>
        /// <returns></returns>
        public (Vector2d centerPrevious, Vector2d centerNext) EdgesAt(int i)
        {
            var verts = VerticesAsReadOnlySpan;
            Vector2d center = verts[i];
            Vector2d prev = verts.PrevVertexFrom(i);
            Vector2d next = verts.NextVertexFrom(i);

            return (prev - center, next - center);
        }

        /// <summary>
        /// Returns the polygons previous and next edge (as Segment2d) at vertex i.
        /// </summary>
        /// <param name="i">vertex index</param>
        /// <returns></returns>
        public (Segment2d previousCenter, Segment2d centerNext) SegmentsAt(int i)
        {
            var verts = VerticesAsReadOnlySpan;
            Vector2d center = verts[i];
            Vector2d prev = verts.PrevVertexFrom(i);
            Vector2d next = verts.NextVertexFrom(i);

            return (new Segment2d(prev, center), new Segment2d(center, next));
        }

        /// <summary>
        /// Calculate the polygons axis aligned bounding box.
        /// Opposed to Bounds property, this method is not cached.
        /// </summary>
        /// <returns></returns>
        public AxisAlignedBox2d CalcBounds()
        {
            AxisAlignedBox2d box = AxisAlignedBox2d.Empty;
            box.Contain(VerticesAsReadOnlySpan);
            return box;
        }

        /// <summary>
        ///  Get the polygons axis aligned bounding box.
        ///  Cached implementation using polygon time stamp (thread safe).
        ///  Uses SIMD for large polygons.
        /// </summary>
        public AxisAlignedBox2d Bounds
        {
            get
            {
                if (Timestamp != boundsCachedTimeStamp)
                {
                    AxisAlignedBox2d calcedBounds;
                    int tStamp = Timestamp;
                    if (VertexCount < 300)
                        calcedBounds = CalcBounds();
                    else
                        calcedBounds = SIMDVectorOperations.Bounds2D(VerticesAsDoubleSpan);
                    cachedBounds = calcedBounds;
                    Interlocked.MemoryBarrier();
                    boundsCachedTimeStamp = tStamp;
                    return calcedBounds;
                }
                return cachedBounds;
            }
        }

        public IEnumerable<Segment2d> SegmentItr()
        {
            Vector2d prev = vertices[^1];
            for (int i = 0; i < vertices.Count; ++i)
            {               
                yield return new Segment2d(prev, vertices[i]);
                prev = vertices[i];
            }
        }

        public IEnumerable<Vector2d> VerticesItr(bool bRepeatFirstAtEnd)
        {
            int N = vertices.Count;
            for (int i = 0; i < N; ++i)
                yield return vertices[i];
            if (bRepeatFirstAtEnd)
                yield return vertices[0];
        }

        public bool IsClockwise => SignedArea < 0;
        public bool IsHole => IsClockwise;
        public double SignedArea
        {
            get
            {
                //this DOES NOT support concurrent edits that change the timestamp
                //only multi threaded "read" with an out of date cache (that all write the same area)
                if (signedAreaTimestamp != Timestamp)
                {
                    int computeTimeStamp = Timestamp;
                    double calcedArea;
                    if (VertexCount < 600)
                    {
                        double fArea = 0;
                        int N = vertices.Count;
                        if (N == 0)
                            return 0.0;
                        Vector2d v1 = vertices[0], v2 = Vector2d.Zero;
                        for (int i = 0; i < N; ++i)
                        {
                            v2 = vertices[(i + 1) % N];
                            fArea += v1.x * v2.y - v1.y * v2.x;
                            v1 = v2;
                        }
                        calcedArea = fArea * 0.5;
                    }
                    else
                        calcedArea = SIMDVectorOperations.Area2D(VerticesAsDoubleSpan);
                    signedAreaCached = calcedArea;
                    Interlocked.MemoryBarrier();
                    signedAreaTimestamp = computeTimeStamp;
                    return calcedArea;
                }
                return signedAreaCached;
            }
        }
        public double Area => Math.Abs(SignedArea);
        public double Perimeter
        {
            get
            {
                double fLength = 0;
                ReadOnlySpan<Vector2d> verts = VerticesAsReadOnlySpan;
                if (verts.Length == 0) return 0.0;
                Vector2d prev = verts[^1], cur;
                for (int i = 0; i < verts.Length; ++i)
                {
                    cur = vertices[i];
                    fLength += prev.Distance(cur);
                    prev = cur;
                }
                return fLength;
            }
        }
        public double ArcLength => Perimeter;

        public void NeighboursP(int iVertex, ref Vector2d p0, ref Vector2d p1)
        {
            int N = vertices.Count;
            p0 = vertices[(iVertex == 0) ? N - 1 : iVertex - 1];
            p1 = vertices[(iVertex + 1) % N];
        }
        public void NeighboursV(int iVertex, ref Vector2d v0, ref Vector2d v1, bool bNormalize = false)
        {
            int N = vertices.Count;
            v0 = vertices[(iVertex == 0) ? N - 1 : iVertex - 1] - vertices[iVertex];
            v1 = vertices[(iVertex + 1) % N] - vertices[iVertex];
            if (bNormalize)
            {
                v0.Normalize();
                v1.Normalize();
            }
        }

        public double OpeningAngleDeg(int iVertex)
        {
            Vector2d e0 = Vector2d.Zero, e1 = Vector2d.Zero;
            NeighboursV(iVertex, ref e0, ref e1, true);
            return Vector2d.AngleD(e0, e1);
        }

        /// <summary>
        /// Compute winding integral at point P
        /// </summary>
        public double WindingIntegral(Vector2d P)
        {
            double sum = 0;
            int N = vertices.Count;
            Vector2d a = vertices[0] - P, b = Vector2d.Zero;
            for (int i = 0; i < N; ++i)
            {
                b = vertices[(i + 1) % N] - P;
                sum += Math.Atan2(a.x * b.y - a.y * b.x, a.x * b.x + a.y * b.y);
                a = b;
            }
            return sum / MathUtil.TwoPI;
        }

        /// <summary>
        /// Compute the indices of the polygon vertices that form the visibility cone
        /// together with <paramref name="viewPoint"/>.  
        /// The cone is defined by two rays:
        /// <code>
        ///   vertices[idxLeft]    vertices[idxRight]
        ///            ^               ^
        ///             \             /
        ///              \           /
        ///               \         /
        ///                viewPoint
        /// </code>
        /// </summary>
        /// <param name="viewPoint">The origin of the cone.</param>
        /// <returns>
        /// A pair of indices (idxLeft, idxRight) delimiting the visible cone.
        /// </returns>
        public (int idxLeft, int idxRight) VisibilityCone(Vector2d viewPoint)
        {
            if (vertices.Count == 0)
                throw new InvalidOperationException("polygon has no vertices");

            ReadOnlySpan<Vector2d> verts = VerticesAsReadOnlySpan;
            int idxLeft = 0, idxRight = 0;
            Vector2d dir = verts[0] - viewPoint;
            Vector2d leftLimit = dir;
            Vector2d rightLimit = dir;

            //1. loop for convex case
            int i = 1;
            for (; i < verts.Length; i++)
            {
                dir = verts[i] - viewPoint;

                double crossLeft  =  leftLimit.Cross(dir);
                double crossRight = rightLimit.Cross(dir);
                bool isLeft  =  crossLeft > 0;
                bool isRight = crossRight < 0;

                // if both are true, viewPoint is within a concavity of the polygon
                // and the angle interval exceeds 180°
                if (isLeft && isRight) break;
                if (isLeft)  { leftLimit  = dir; idxLeft  = i; }
                if (isRight) { rightLimit = dir; idxRight = i; }
            }

            if (i == verts.Length)
                return (idxLeft, idxRight);

            // if we arrive here, we are sure the angle interval is larger than 180°
            // the left/right limit approach breaks down at that point because vertices
            // can be left of the left limit line and right of the right limit line at once
            // we now pick one of the limits as a new axis of an oblique coordinate system
            
            Line2d halfPlaneAxis = new Line2d(viewPoint, dir);
            Line2dEndpoints debugAxis = halfPlaneAxis.EndPoints;
            int uCoordSign, idxBack = 1;
            do
            {
                uCoordSign = halfPlaneAxis.WhichSide(verts[i - idxBack++]);
            } while (uCoordSign == 0);
                        
            // 2. loop continue for concave case, do all remaining iterations and the first wraparound (i=0)
            // did not manage to figure out a clean loop control for that, main loop and AdvanceLimitWhileInUncoveredHalfplane
            // both increment i and handle wrap around
            do
            {
                i = i < verts.Length ? i : 0;
                dir = verts[i] - viewPoint;

                //skip vertices in the already covered half plane
                if (halfPlaneAxis.Direction.Cross(dir) * uCoordSign < 0) continue;
                // when we enter the uncovered half plane,
                // we track from which side relative to viewPoint we enter and exit (u coordinate pos or neg)
                // if enter and exit sides differ, we have completed a full winding => full circle cover
                double entryUCoord = UCoord(verts, i);
                if (entryUCoord > 0)
                {
                    double exitUCoord = AdvanceLimitWhileInUncoveredHalfplane(verts, ref i, ref leftLimit, ref idxLeft, 1);
                    if (exitUCoord < 0) { idxRight = idxLeft; break; }
                }
                else if (entryUCoord < 0)
                {
                    double exitUCoord = AdvanceLimitWhileInUncoveredHalfplane(verts, ref i, ref rightLimit, ref idxRight, -1);
                    if (exitUCoord > 0) { idxLeft = idxRight; break; }
                }
                else { }//skip segments parallel to halfPlaneAxis and coincident with viewPoint
                //detect full circle cover when limits entered and exit at the same side, but crossed each other
                //if (rightLimit.Dot(halfPlaneAxis.Direction) * uCoordSign - leftLimit.Dot(halfPlaneAxis.Direction) * uCoordSign > MathUtil.ZeroTolerance)
                //{ idxRight = idxLeft; break; }
            } while (i++ != 0);

            return (idxLeft, idxRight);

            double AdvanceLimitWhileInUncoveredHalfplane(ReadOnlySpan<Vector2d> verts, ref int i, ref Vector2d limit, ref int limitIdx, int crossSign)
            {
                do
                {
                    if (limit.Cross(dir) * crossSign >= 0) { limit = dir; limitIdx = i; }
                    if (i == 0) { return 0; }
                    else if (++i == verts.Length) { i = 0; }
                    dir = verts[i] - viewPoint;
                } while (halfPlaneAxis.Direction.Cross(dir) * uCoordSign >= 0);
                return UCoord(verts, i);
            }

            double UCoord(ReadOnlySpan<Vector2d> verts, int i)
            {
                Vector2d prev = i == 0 ? verts[^1] : verts[i - 1];
                Line2d segmentLine = new Line2d(prev, verts[i] - prev);
                double intersectionT = halfPlaneAxis.IntersectionT(segmentLine);
                //parallel case: recourse backwards
                if (intersectionT == double.MaxValue) return 0.0;
                //    return UCoord(verts, i - 1, recursionCount++);
                return intersectionT * uCoordSign;
            }
        }

        public AngularInterval VisibilityConeAngles(Vector2d viewPoint)
        {
            var (idxLeft, idxRight) = VisibilityCone(viewPoint);
            if(idxLeft == idxRight) return AngularInterval.FullCircle;

            Vector2d dirLeft = vertices[idxLeft] - viewPoint;
            Vector2d dirRight = vertices[idxRight] - viewPoint;

            double angleLeft = Math.Atan2(dirLeft.y, dirLeft.x);
            double angleRight = Math.Atan2(dirRight.y, dirRight.x);
            
            var interval = AngularInterval.Normalized(angleRight, angleLeft);
            if (interval.Length > Math.PI * 2) return AngularInterval.FullCircle;
            return interval;
        }

        /// <summary>
        /// Returns true if point inside polygon, using fast winding-number computation
        /// Points on the polygon boundary (on a vertex or edges) return false (exclusive contains).
        /// Warning: in most practical applications, this method suffers from numerical issues for points near the boundary.
        /// Using ContainsInclusive or ContainsExclusive with a tolerance is highly recommended,
        /// except if it is known that
        /// <param name="P">Point to test for inclusion in polygon</param>
        /// </summary>
        public bool Contains(Vector2d P)
        {
            // based on http://geomalgorithms.com/a03-_inclusion.html	
            int nWindingNumber = 0;

            ReadOnlySpan<Vector2d> verts = VerticesAsReadOnlySpan;
            Vector2d a = verts[^1], b = Vector2d.Zero;
            for (int i = 0; i < verts.Length; ++i, a = b)
            {
                b = verts[i];
                //edge case: if the test point is coincident with a vertex,
                //it is not well defined whether this should increase the winding number or not
                //we catch this case by definition here, returning false for exclusive contains
                if (b == P) return false;

                if (a.y <= P.y)
                {   // y <= P.y (below)
                    if (b.y > P.y)
                    {                         // an upward crossing
                        if (CrossP(a, b, P) > 0)  // P left of edge
                            ++nWindingNumber;                                      // have a valid up intersect
                    }
                }
                else
                {    // y > P.y  (above)
                    if (b.y <= P.y)
                    {                        // a downward crossing
                        if (CrossP(a, b, P) < 0)  // P right of edge
                            --nWindingNumber;                                      // have a valid down intersect
                    }
                }
            }
            return nWindingNumber != 0;
        }

        /// <summary>
        /// Returns true if point P is inside the polygon or on the boundary (inclusive contains with epsilon tolerance)
        /// of the polygon, using fast winding-number computation.
        /// </summary>
        /// <param name="P">Point to test for inclusion in polygon</param>
        /// <param name="epsilon">tolerance for point on edge check</param>
        /// <returns></returns>
        public bool ContainsInclusive(in Vector2d P, double epsilon = MathUtil.ZeroTolerance)
            => ContainsInclusive(P, out _, epsilon);

        /// <summary>
        /// Returns true if point P is inside the polygon, but not on the boundary (exclusive contains with epsilon tolerance)
        /// of the polygon, using fast winding-number computation.
        /// </summary>
        /// <param name="P">Point to test for inclusion in polygon</param>
        /// <param name="epsilon">tolerance for point on edge check</param>
        /// <returns></returns>
        public bool ContainsExclusive(in Vector2d P, double epsilon = MathUtil.ZeroTolerance)
        {
            bool contains = ContainsInclusive(P, out bool isOnEdge, epsilon);
            return contains && !isOnEdge;
        }

        /// <summary>
        /// Returns true if point P is inside the polygon or on the boundary (inclusive contains with epsilon tolerance)
        /// of the polygon, using fast winding-number computation.
        /// </summary>
        /// <param name="P">Point to test for inclusion in polygon</param>
        /// <param name="epsilon">tolerance for point on edge check</param>
        /// <returns></returns>
        private bool ContainsInclusive(in Vector2d P, out bool isOnEdge, double epsilon = MathUtil.ZeroTolerance)
        {
            var verts = VerticesAsReadOnlySpan;
            int nWindingNumber = 0;
            isOnEdge = false;

            //remark: the edge case P on vertex
            //is handled implicitly here by the point on segment check with tolerance
            
            Vector2d a = verts[^1], b = Vector2d.Zero;

            /*
             * This is a special variant of the well-known winding number method that handles a tolerance.
             * There is a tolerance band in y around P. Only segment chains fully crossing the band are accounted for.
             * Vertices within the band (horizontals) cannot switch sides of the polygon (case P1/P2)
             * unless they are "on the line" (case P3) within tolerance, which earlies out.
             * Case a.y < upperYBound handles horizontals for point on segment check.
             * The winding number has an additional condition a.y <= lowerYBound that ensures
             * only the first segment (that enters the band) is used for it.
             * The other case a.y < upperYBound does not handle vertices in the band and thus does not need the check.
             * It uses the first segment entering the band to determine side and decrease winding number.
             * Then a.y < upperYBound case will take over and either
             * 1. not touch the winding number if the segment chain leaves below
             * 2. or increment it (undoing the decrement) if it leaves the band above
             * 
                                                        /
               +---------------------------------------/--------+
                                                      /
                                                     /            
               P1   x---------x   P3   x------------x        P2  
                   /           \      /                                   
                  /             x____x
               +-/----------------------------------------------+
                / 
               /   
             */
            double upperYBound = P.y + epsilon;
            double lowerYBound = P.y - epsilon;
            for (int i = 0; i < verts.Length; ++i, a = b)
            {
                b = verts[i];

                if (a.y <= upperYBound)
                {   // y <= P.y (below)
                    if (b.y >= lowerYBound || a.y >= lowerYBound)
                    {
                        // an upward crossing, or on the line within epsilon
                        (double cross, bool withinEpsilon) = CrossP(a, b, P, epsilon);
                        // P left of edge?
                        if (a.y <= P.y && b.y > P.y && cross > 0)
                            ++nWindingNumber;   // have a valid up intersect
                        else if(a.y > P.y && b.y <= P.y && cross < 0)
                            --nWindingNumber;   // have a valid down intersect

                        if (withinEpsilon)
                        { // on the line?
                            if (PointOnSegment(a, b, P, epsilon))
                            {
                                isOnEdge = true;
                                return true;
                            }
                        }
                    }
                }
                else // y > P.y  (above)
                {
                    if (b.y <= upperYBound)
                    {
                        // a downward crossing, or on the line within epsilon
                        (double cross, bool withinEpsilon) = CrossP(a, b, P, epsilon);
                        // P right of edge?
                        if (a.y > P.y && b.y <= P.y && cross < 0)
                            --nWindingNumber;  // have a valid down intersect

                        if (withinEpsilon)
                        {// on the line?
                            if (PointOnSegment(a, b, P, epsilon))
                            {
                                isOnEdge = true;
                                return true;
                            }
                        }
                    }
                }
            }
            return nWindingNumber != 0;
        }

        private static bool PointOnSegment(in Vector2d a, in Vector2d b, in Vector2d P, double epsilon)
        {
            Segment2d seg = new Segment2d(a, b);
            double extentTol = seg.Extent + epsilon;
            double dist = seg.Project(P);
            if (Math.Abs(dist) <= extentTol) return true;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static double CrossP(in Vector2d edgeP0, in Vector2d edgeP1, in Vector2d testP)
        {
            return (edgeP1.x - edgeP0.x) * (testP.y - edgeP0.y) - (testP.x - edgeP0.x) * (edgeP1.y - edgeP0.y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static (double cross, bool withinEpsilon) CrossP(in Vector2d P0, in Vector2d P1, in Vector2d P2, double epsilon)
        {
            double vx = P1.x - P0.x;
            double vy = P1.y - P0.y;
            double wx = P2.x - P0.x;
            double wy = P2.y - P0.y;
            double cross = vx * wy - vy * wx;
            double lenSq = vx * vx + vy * vy;
            bool withinEpsilon = cross * cross <= epsilon * epsilon * lenSq;
            return (cross, withinEpsilon);
        }

        public bool Contains(Polygon2d o)
        {

            // [TODO] fast bbox check?

            int N = o.VertexCount;
            for (int i = 0; i < N; ++i)
            {
                if (Contains(o[i]) == false)
                    return false;
            }

            if (Intersects(o))
                return false;

            return true;
        }

        /// <summary>
        /// Checks that all points on a segment are within the area defined by the Polygon2d.
        /// </summary>
        public bool Contains(Segment2d o)
        {
            // [TODO] Add bbox check
            if (Contains(o.P0) == false || Contains(o.P1) == false)
                return false;

            foreach (Segment2d seg in SegmentItr())
            {
                if (seg.Intersects(o))
                    return false;
            }
            return true;
        }

        public bool Intersects(Polygon2d o, double dotThresh = double.Epsilon, double intervalThresh = 0)
        {
            if (!this.Bounds.Intersects(o.Bounds))
                return false;

            foreach (Segment2d seg in SegmentItr())
            {
                foreach (Segment2d oseg in o.SegmentItr())
                {
                    if (seg.Intersects(oseg, dotThresh, intervalThresh))
                        return true;
                }
            }
            return false;
        }

        /// <summary>
        /// Checks if any point on a segment is within the area defined by the Polygon2d.
        /// </summary>
        public bool Intersects(Segment2d o, double dotThresh = double.Epsilon, double intervalThresh = 0)
        {
            if (!this.Bounds.Intersects(o))
                return false;

            if (Contains(o.P0) == true || Contains(o.P1) == true)
                return true;

            return IntersectsIgnoreContainment(o, dotThresh, intervalThresh);
        }

        public bool IntersectsIgnoreContainment(Segment2d o, double dotThresh = double.Epsilon, double intervalThresh = 0)
        {
            foreach (Segment2d item in SegmentItr())
            {
                if (item.Intersects(o, dotThresh, intervalThresh))
                {
                    return true;
                }
            }

            return false;
        }

        public List<Vector2d> FindIntersections(Polygon2d o)
        {
            List<Vector2d> v = new List<Vector2d>();
            if (!this.Bounds.Intersects(o.Bounds))
                return v;

            foreach (Segment2d seg in SegmentItr())
            {
                foreach (Segment2d oseg in o.SegmentItr())
                {
                    // this computes test twice for intersections, but seg.intersects doesn't
                    // create any new objects so it should be much faster for majority of segments (should profile!)
                    if (seg.Intersects(oseg))
                    {
                        IntrSegment2Segment2 intr = new IntrSegment2Segment2(seg, oseg);
                        if (intr.Find())
                        {
                            v.Add(intr.Point0);
                            if (intr.Quantity == 2)
                                v.Add(intr.Point1);
                        }
                    }
                }
            }
            return v;
        }

        public Segment2d Segment(int iSegment)
        {
            return new Segment2d(vertices[iSegment], vertices[(iSegment + 1) % vertices.Count]);
        }

        public Vector2d PointAt(int iSegment, double fSegT)
        {
            Segment2d seg = new Segment2d(vertices[iSegment], vertices[(iSegment + 1) % vertices.Count]);
            return seg.PointAt(fSegT);
        }

        public Vector2d GetNormal(int iSeg, double segT)
        {
            Segment2d seg = new Segment2d(vertices[iSeg], vertices[(iSeg + 1) % vertices.Count]);
            double t = ((segT / seg.Extent) + 1.0) / 2.0;

            Vector2d n0 = GetNormal(iSeg);
            Vector2d n1 = GetNormal((iSeg + 1) % vertices.Count);
            return ((1.0 - t) * n0 + t * n1).Normalized;
        }

        public double DistanceSquared(in Vector2d p, double tolerance = MathUtil.ZeroTolerance) => DistanceSquared(p, out _, out _, tolerance);
        public double DistanceSquared(in Vector2d p, out int iNearSeg, out double fNearSegT, double tolerance = MathUtil.ZeroTolerance) => DistanceSquared(p, out iNearSeg, out fNearSegT, out _, tolerance);
        public double DistanceSquared(in Vector2d p, out int iNearSeg, out double fNearSegT, out Vector2d nearestPoint, double tolerance = MathUtil.ZeroTolerance)
        {
            iNearSeg = -1;
            fNearSegT = double.MaxValue;
            double dist = double.MaxValue;
            nearestPoint = p;
            if (vertices.Count == 0) return double.NaN;
            Vector2d prev = vertices[^1];
            for (int vi = 0; vi < vertices.Count; ++vi)
            {
                Segment2d seg = new Segment2d(prev, vertices[vi]);
                prev = vertices[vi];

                double t = (p - seg.Center).Dot(seg.Direction);
                double d = double.MaxValue;
                Vector2d closestSegPt;
                if (t >= seg.Extent)
                    closestSegPt = seg.P1;
                else if (t <= -seg.Extent)
                    closestSegPt = seg.P0;
                else
                    closestSegPt = seg.PointAt(t);

                d = closestSegPt.DistanceSquared(p);
                if (d < dist)
                {
                    dist = d;
                    iNearSeg = vi;
                    fNearSegT = t;
                    nearestPoint = closestSegPt;
                    //early out, coincident
                    if (dist < tolerance) return 0.0;
                }
            }
            return dist;
        }

        /// <summary>
        /// Finds the nearest point of a polygon from p by iterating through the polygons segments.
        /// If many queries are done, Polygon2dBoxTree should be used instead.
        /// </summary>
        /// <param name="p"></param>
        /// <returns>nearest point</returns>
        public Vector2d NearestPoint(Vector2d p)
        {
            var minDist = double.MaxValue;
            Vector2d result = p;

            Vector2d prev = vertices[^1];
            for (int i = 0; i < vertices.Count; ++i)
            {
                var segment = new Segment2d(prev, vertices[i]);
                var point = segment.NearestPoint(p);
                var dist = point.DistanceSquared(p);
                if (dist < minDist)
                {
                    result = point;
                    minDist = dist;
                }
                prev = vertices[i];
            }
            return result;
        }

        public double AverageEdgeLength
        {
            get
            {
                double avg = 0; int N = vertices.Count;
                for (int i = 1; i < N; ++i)
                    avg += vertices[i].Distance(vertices[i - 1]);
                avg += vertices[N - 1].Distance(vertices[0]);
                return avg / N;
            }
        }

        public Polygon2d Translate(Vector2d translation)
        {
            if (VertexCount < 12)
            {
                int N = vertices.Count;
                for (int i = 0; i < N; ++i)
                    vertices[i] += translation;
                Timestamp++;
                return this;
            }

            else
            {
                SIMDVectorOperations.TranslateAsVector2(VerticesAsDoubleSpan, translation);
                Timestamp++;
            }
            return this;
        }

        public Polygon2d Rotate(Matrix2d rotation, Vector2d origin)
        {
            if (VertexCount < 12)
            {
                int N = vertices.Count;
                for (int i = 0; i < N; ++i)
                    vertices[i] = rotation * (vertices[i] - origin) + origin;
                Timestamp++;
                return this;
            }
            else
            {
                var coordSpan = VerticesAsDoubleSpan;
                if (origin.x == 0 && origin.y == 0)
                {
                    SIMDVectorOperations.RotateAsVector2(coordSpan, rotation);
                }
                else
                {
                    SIMDVectorOperations.TranslateAsVector2(coordSpan, -origin);
                    SIMDVectorOperations.RotateAsVector2(coordSpan, rotation);
                    SIMDVectorOperations.TranslateAsVector2(coordSpan, origin);
                }
                Timestamp++;
            }
            return this;
        }

        public Polygon2d Scale(Vector2d scale, Vector2d origin)
        {
            int N = vertices.Count;
            for (int i = 0; i < N; ++i)
                vertices[i] = scale * (vertices[i] - origin) + origin;
            Timestamp++;
            return this;
        }

        public Polygon2d Transform(Func<Vector2d, Vector2d> transformF)
        {
            int N = vertices.Count;
            for (int i = 0; i < N; ++i)
                vertices[i] = transformF(vertices[i]);
            Timestamp++;
            return this;
        }
        public void OverrideVertexCount(int count)
        {
            CollectionsMarshal.SetCount(vertices, count);
            Timestamp++;
        }
        public Polygon2d Transform(ITransform2 xform)
        {
            int N = vertices.Count;
            for (int k = 0; k < N; ++k)
                vertices[k] = xform.TransformP(vertices[k]);
            Timestamp++;
            return this;
        }

        /// <summary>
        /// Offset each point by dist along vertex normal direction (ie tangent-perp)
        /// CCW polygon offsets "outwards", CW "inwards".
        /// </summary>
        public void VtxNormalOffset(double dist, bool bUseFaceAvg = false)
        {
            Vector2d[] newv = new Vector2d[vertices.Count];
            if (bUseFaceAvg)
            {
                for (int k = 0; k < vertices.Count; ++k)
                    newv[k] = vertices[k] + dist * GetNormal_FaceAvg(k);
            }
            else
            {
                for (int k = 0; k < vertices.Count; ++k)
                    newv[k] = vertices[k] + dist * GetNormal(k);
            }
            for (int k = 0; k < vertices.Count; ++k)
                vertices[k] = newv[k];

            Timestamp++;
        }

        /// <summary>
        /// offset polygon by fixed distance, by offsetting and intersecting edges.
        /// CCW polygon offsets "outwards", CW "inwards".
        /// </summary>
        public void PolyOffset(double dist)
        {
            // [TODO] possibly can do with half as many normalizes if we do w/ sequential edges,
            //  rather than centering on each v?
            Vector2d[] newv = new Vector2d[vertices.Count];
            for (int k = 0; k < vertices.Count; ++k)
            {
                Vector2d v = vertices[k];
                Vector2d next = vertices[(k + 1) % vertices.Count];
                Vector2d prev = vertices[k == 0 ? vertices.Count - 1 : k - 1];
                Vector2d dn = (next - v).Normalized;
                Vector2d dp = (prev - v).Normalized;
                Line2d ln = new Line2d(v + dist * dn.Perp, dn);
                Line2d lp = new Line2d(v - dist * dp.Perp, dp);

                newv[k] = ln.IntersectionPoint(ref lp);
                if (newv[k] == Vector2d.MaxValue)
                {
                    newv[k] = vertices[k] + dist * GetNormal_FaceAvg(k);
                }
            }
            for (int k = 0; k < vertices.Count; ++k)
                vertices[k] = newv[k];

            Timestamp++;
        }

        // Polygon simplification
        // code adapted from: http://softsurfer.com/Archive/algorithm_0205/algorithm_0205.htm
        // simplifyDP():
        //  This is the Douglas-Peucker recursive simplification routine
        //  It just marks vertices that are part of the simplified polyline
        //  for approximating the polyline subchain v[j] to v[k].
        //    Input:  tol = approximation tolerance
        //            v[] = polyline array of vertex points
        //            j,k = indices for the subchain v[j] to v[k]
        //    Output: mk[] = array of markers matching vertex array v[]
        static void simplifyDP(double tol, Vector2d[] v, int j, int k, bool[] mk)
        {
            if (k <= j + 1) // there is nothing to simplify
                return;

            // check for adequate approximation by segment S from v[j] to v[k]
            int maxi = j;          // index of vertex farthest from S
            double maxd2 = 0;         // distance squared of farthest vertex
            double tol2 = tol * tol;  // tolerance squared
            Segment2d S = new Segment2d(v[j], v[k]);    // segment from v[j] to v[k]

            // test each vertex v[i] for max distance from S
            // Note: this works in any dimension (2D, 3D, ...)
            for (int i = j + 1; i < k; i++)
            {
                double dv2 = S.DistanceSquared(v[i]);
                if (dv2 <= maxd2)
                    continue;
                // v[i] is a new max vertex
                maxi = i;
                maxd2 = dv2;
            }
            if (maxd2 > tol2)
            {       // error is worse than the tolerance
                    // split the polyline at the farthest vertex from S
                mk[maxi] = true;      // mark v[maxi] for the simplified polyline
                                      // recursively simplify the two subpolylines at v[maxi]
                simplifyDP(tol, v, j, maxi, mk);  // polyline v[j] to v[maxi]
                simplifyDP(tol, v, maxi, k, mk);  // polyline v[maxi] to v[k]
            }
            // else the approximation is OK, so ignore intermediate vertices
            return;
        }

        public void Simplify(double clusterTol = 0.0001,
                              double lineDeviationTol = 0.01,
                              bool bSimplifyStraightLines = true, bool shiftStart = false)
        {
            if (vertices.Count <= 3 || lineDeviationTol <= 0)
                return;
            if (shiftStart)
            {
                var shiftIdx = VerticesShiftedStart();
                if (shiftIdx > 0) ShiftVertex0(shiftIdx);
            }
            //a line deviation tolerance larger than the minimum dimension of the polygon 
            //yields poor accuracy results while not simplifying the input more than a tolerance
            //equal to min bounds extent (half dimension) would
            //best case would be if we could use the minimal bounding box here
            //but that algorithm is way to expensive for using it here
            //we double the tolerance for a bit more safety, but that does not help if the polygons have a large aspect ratio
            //and are not bounds aligned
            if (!BoundsGreaterThan(lineDeviationTol / 4))
            {
                lineDeviationTol = Bounds.MinDim / 4;
            }


            int n = vertices.Count;
            if (n < 3)
                return;

            int i, k, pv;            // misc counters
            Vector2d[] vt = new Vector2d[n + 1];  // vertex buffer
            bool[] mk = new bool[n + 1];
            for (i = 0; i < n + 1; ++i)     // marker buffer
                mk[i] = false;

            // STAGE 1.  Vertex Reduction within tolerance of prior vertex cluster
            double clusterTol2 = clusterTol * clusterTol;
            vt[0] = vertices[0];              // start at the beginning
            for (i = 1, k = 1, pv = 0; i < n; i++)
            {
                if ((vertices[i] - vertices[pv]).LengthSquared < clusterTol2)
                    continue;
                vt[k++] = vertices[i];
                pv = i;
            }
            bool skip_dp = false;
            if (k == 1)
            {
                vt[k++] = vertices[1];
                vt[k++] = vertices[2];
                skip_dp = true;
            }
            else if (k == 2)
            {
                vt[k++] = vertices[0];
                skip_dp = true;
            }

            // push on start vertex again, because simplifyDP is for polylines, not polygons
            vt[k++] = vertices[0];

            // STAGE 2.  Douglas-Peucker polyline simplification
            int nv = 0;
            if (skip_dp == false && lineDeviationTol > 0)
            {
                mk[0] = mk[k - 1] = true;       // mark the first and last vertices
                simplifyDP(lineDeviationTol, vt, 0, k - 1, mk);
                for (i = 0; i < k - 1; ++i)
                {
                    if (mk[i])
                        nv++;
                }
            }
            else
            {
                for (i = 0; i < k; ++i)
                    mk[i] = true;
                nv = k - 1;
            }

            // polygon requires at least 3 vertices
            if (nv == 2)
            {
                for (i = 1; i < k - 1; ++i)
                {
                    if (mk[1] == false)
                        mk[1] = true;
                    else if (mk[k - 2] == false)
                        mk[k - 2] = true;
                }
                nv++;
            }
            else if (nv == 1)
            {
                mk[1] = true;
                mk[2] = true;
                nv += 2;
            }

            // copy marked vertices back to this polygon
            vertices = new List<Vector2d>();
            for (i = 0; i < k - 1; ++i)
            {   // last vtx is copy of first, and definitely marked
                if (mk[i])
                    vertices.Add(vt[i]);
            }

            Timestamp++;
            TrimExcess();
            return;
        }

        public void Chamfer(double chamfer_dist, double minConvexAngleDeg = 30, double minConcaveAngleDeg = 30)
        {
            if (IsClockwise)
                throw new Exception("must be ccw?");

            List<Vector2d> NewV = new List<Vector2d>();
            int N = Vertices.Count;

            int iCur = 0;
            do
            {
                Vector2d center = Vertices[iCur];

                int iPrev = (iCur == 0) ? N - 1 : iCur - 1;
                Vector2d prev = Vertices[iPrev];
                int iNext = (iCur + 1) % N;
                Vector2d next = Vertices[iNext];

                Vector2d cp = prev - center;
                double cpdist = cp.Normalize();
                Vector2d cn = next - center;
                double cndist = cn.Normalize();

                // if degenerate, skip this vert
                if (cpdist < MathUtil.ZeroTolerancef || cndist < MathUtil.ZeroTolerancef)
                {
                    iCur = iNext;
                    continue;
                }

                double angle = Vector2d.AngleD(cp, cn);
                // TODO document what this means sign-wise
                double sign = cp.Perp.Dot(cn);
                bool bConcave = (sign > 0);

                double thresh = (bConcave) ? minConcaveAngleDeg : minConvexAngleDeg;

                // ok not too sharp
                if (angle > thresh)
                {
                    NewV.Add(center);
                    iCur = iNext;
                    continue;
                }


                double prev_cut_dist = Math.Min(chamfer_dist, cpdist * 0.5);
                Vector2d prev_cut = center + prev_cut_dist * cp;
                double next_cut_dist = Math.Min(chamfer_dist, cndist * 0.5);
                Vector2d next_cut = center + next_cut_dist * cn;

                NewV.Add(prev_cut);
                NewV.Add(next_cut);
                iCur = iNext;
            } while (iCur != 0);

            vertices = NewV;
            Timestamp++;
        }

        /// <summary>
        /// Return minimal bounding box of vertices, computed to epsilon tolerance
        /// </summary>
        public Box2d MinimalBoundingBox(double epsilon = ConvexHull2.DefaultConvexHullTolerance)
        {
            return new ContMinBox2(this.ConvexHull(epsilon).vertices, epsilon, QueryNumberType.QT_DOUBLE, isConvexPolygon: true).MinBox;
        }

        public void ClearVertices()
        {
            vertices.Clear();
            Timestamp = 0;
            boundsCachedTimeStamp = int.MinValue;
            signedAreaTimestamp = int.MinValue;
        }

        public static Polygon2d MakeRectangle(Vector2d center, double width, double height)
        {
            var rect = new Polygon2d();
            var vertices = rect.VerticesAsSpanWithCount(4);
            vertices[0] = new Vector2d(center.x - width / 2.0, center.y - height / 2.0);
            vertices[1] = new Vector2d(center.x + width / 2.0, center.y - height / 2.0);
            vertices[2] = new Vector2d(center.x + width / 2.0, center.y + height / 2.0);
            vertices[3] = new Vector2d(center.x - width / 2.0, center.y + height / 2.0);
            return rect;
        }

        static public Polygon2d MakeCircle(double fRadius, int nSteps, double angleShiftRad = 0)
        {
            var circle = new Polygon2d();
            var vertices = circle.VerticesAsSpanWithCount(nSteps);

            for (int i = 0; i < nSteps; ++i)
            {
                double t = i / (double)nSteps;
                double a = MathUtil.TwoPI * t + angleShiftRad;
                vertices[i] = new Vector2d(fRadius * Math.Cos(a), fRadius * Math.Sin(a));
            }

            return circle;
        }

        static public Polygon2d MakeWedge(Vector2d center, double fRadius, int nSteps, double angleStartInRad, double angleEndInRad)
        {
            var wedge = new Polygon2d();
            Span<Vector2d> vertices = wedge.VerticesAsSpanWithCount(nSteps + 1);

            vertices[0] = center;
            double stepSize = (angleEndInRad - angleStartInRad) / (nSteps - 1);
            for (int i = 0; i < nSteps; ++i)
            {
                double t = angleStartInRad + i * stepSize;
                vertices[i + 1] = center + new Vector2d(fRadius * Math.Cos(t), fRadius * Math.Sin(t));
            }

            return wedge;
        }

        public int VerticesCapacity { get => vertices.Capacity; set => vertices.Capacity = value; }
        private Span<double> VerticesAsDoubleSpan => MemoryMarshal.Cast<Vector2d, double>(CollectionsMarshal.AsSpan(vertices));
        public Span<Vector2d> VerticesAsSpan => CollectionsMarshal.AsSpan(vertices);
        public Span<Vector2d> VerticesAsSpanWithCount(int count)
        {
            CollectionsMarshal.SetCount(vertices, count);
            return CollectionsMarshal.AsSpan(vertices);
        }
        public ReadOnlySpan<Vector2d> VerticesAsReadOnlySpan => CollectionsMarshal.AsSpan(vertices);

        public Circle2d MinBoundingCircle()
        {
            g3.ContMinCircle2 bCircle = new ContMinCircle2(vertices);
            return bCircle.Result;
        }

        /// <summary>
        /// Compute the centroid (center of mass) of the polygon
        /// </summary>
        public Vector2d Centroid()
        {
            //we calc the signed area of each triangle for this algorithm anyways
            //so might as well sum it and update the area cache
            double signedArea = 0.0;
            Vector2d centroid = Vector2d.Zero;
            var verts = VerticesAsReadOnlySpan;

            Vector2d v0 = verts[^1];
            Vector2d v1;
            for (int i = 0; i < verts.Length; i++)
            {
                v1 = verts[i];
                double a = v0.x * v1.y - v1.x * v0.y;
                signedArea += a;
                centroid += (v0 + v1) * a;
                v0 = v1;
            }

            signedArea *= 0.5;
            signedAreaCached = signedArea;
            signedAreaTimestamp = Timestamp;

            centroid /= (6.0 * signedArea);
            return centroid;
        }

        public void TrimExcess() => vertices.TrimExcess();

        /// <summary>
        /// Returns true if circle is inside polygon, using fast winding-number computation
        /// </summary>
        public bool Contains(Circle2d circle)
        {
            // this method is an adapted Contains for Vector2d from G3Sharp.Polygon2d
            // to be inside the polygon, all vertices of the polygon need to be outside the circle
            // the circle center has to be inside the polygon
            // and all polygon edges shall not intersect the circle

            // early out check, but only if bounds are cached
            if (boundsCachedTimeStamp == Timestamp && !cachedBounds.Contains(circle)) return false;

            Vector2d center = circle.Center;
            int nWindingNumber = 0;

            int N = vertices.Count;
            Vector2d a = vertices[0], b = Vector2d.Zero;
            for (int i = 0; i < N; ++i)
            {
                //fast fail; if the circle contains any of the vertices it can not be contained inside the polygon
                if (circle.Contains(a)) return false;

                b = vertices[(i + 1) % N];

                //check the height of the triangle spanned by the line segment and the circle center
                var ab = a - b;
                var ac = a - center;
                var triHeight = Math.Abs(ab.Cross(ac)) / ab.Length;
                if (triHeight < circle.Radius) return false;

                if (a.y <= center.y)
                {   // y <= P.y (below)
                    if (b.y > center.y)
                    {                         // an upward crossing
                        if (MathUtil.IsLeft(a, b, center) > 0)  // P left of edge
                            ++nWindingNumber;                                      // have a valid up intersect
                    }
                }
                else
                {    // y > P.y  (above)
                    if (b.y <= center.y)
                    {                        // a downward crossing
                        if (MathUtil.IsLeft(a, b, center) < 0)  // P right of edge
                            --nWindingNumber;                                      // have a valid down intersect
                    }
                }
                a = b;
            }
            return nWindingNumber != 0;
        }

        public void InsertVertex(int index, Vector2d vertex)
        {
            vertices.Insert(index, vertex);
            Timestamp++;
        }

        /// <summary>
        /// Intended to be used AFTER cleaning a self intersecting polygon with clipper.
        /// Check all the polygons vertices for duplicate coordinates.
        /// If there are duplicates, resolve them by moving the second vertex about moveDistance
        /// in normal direction to the polygon.
        /// This ensures all vertex coordinates are unique and there are no "touching" vertices.
        /// </summary>
        /// <param name="moveDistance"></param>
        public void MoveSelfTouchingVertices(double moveDistance = 0.0001)
        {
            var vertexSet = new HashSet<Vector2d>(vertices.Count);
            MoveSelfTouchingVertices(vertexSet, moveDistance);
        }

        /// <summary>
        /// See <see cref="MoveSelfTouchingVertices"/>, but with additional vertices passed in
        /// as vertexSet that are checked for duplicates.
        /// </summary>
        /// <param name="moveDistance"></param>
        public void MoveSelfTouchingVertices(HashSet<Vector2d> vertexSet, double moveDistance = 0.0001)
        {
            for (int i = 0; i < vertices.Count; i++)
            {
                while (!vertexSet.Add(vertices[i]))
                {
                    //duplicate vertex coordinates, slightly move the vertex
                    var normal = GetNormal_FaceAvg(i);
                    vertices[i] += normal * moveDistance;
                    Timestamp++;
                }
            }
        }

        public void RemoveConsecutiveCoincidentVertices(double tolerance = MathUtil.ZeroTolerance)
        {
            ReadOnlySpan<Vector2d> vertSpan = VerticesAsReadOnlySpan;
            double tolSquared = tolerance * tolerance;
            Vector2d previous = vertSpan[0];
            for (int i = vertSpan.Length - 1; i >= 0; i--)
            {
                if ((vertSpan[i] - previous).LengthSquared < tolSquared)
                {
                    //this operation on the list reflects in the readonly span as well
                    vertices.RemoveAt(i);
                    Timestamp++;
                }
                else
                {
                    previous = vertSpan[i];
                }
            }
        }

        public void RemoveColinearEdges(double tolerance = MathUtil.ZeroTolerancef)
        {
            for (int i = vertices.Count - 1; i >= 0; i--)
            {
                var angle = this.AngleRAtVertex(i);
                bool edgesColinear = Math.Abs(angle - Math.PI) < tolerance || Math.Abs(angle) < tolerance;
                if (edgesColinear)
                {
                    vertices.RemoveAt(i);
                    Timestamp++;
                }
            }
        }

        public bool HasColinearEdges(double tolerance = MathUtil.ZeroTolerancef)
        {
            for (int i = vertices.Count - 1; i >= 0; i--)
            {
                var angle = this.AngleRAtVertex(i);
                bool edgesColinear = Math.Abs(angle - Math.PI) < tolerance || Math.Abs(angle) < tolerance;
                if (edgesColinear) { return true; }
            }
            return false;
        }

        public bool RemoveCoincidentColinear(double coincidenceTol = MathUtil.ZeroTolerancef, double colinearTol = MathUtil.ZeroTolerance)
        {
            if (VertexCount <= 3) return false;
            int startTimeStamp = Timestamp;
            int timeStamp;
            do
            {
                timeStamp = Timestamp;
                RemoveConsecutiveCoincidentVertices(coincidenceTol);
                RemoveColinearEdges(colinearTol);
            } while (Timestamp > timeStamp && VertexCount > 3);
            return Timestamp > startTimeStamp;
        }

        public (Polygon2d poly1, Polygon2d poly2) Cut(Vector2i cutLine, double tolerance = Constants.DEFAULT_LINE_IDENTITY_TOLERANCE)
        {
            if (cutLine.x > cutLine.y)
            {
                var temp = cutLine.x;
                cutLine.x = cutLine.y;
                cutLine.y = temp;
            }
            var nv2 = cutLine.y - cutLine.x;
            var nv1 = vertices.Count - nv2;
            var array1 = new Vector2d[nv1 + 1];
            var array2 = new Vector2d[nv2 + 1];

            vertices.CopyTo(0, array1, 0, cutLine.x + 1);
            vertices.CopyTo(cutLine.x, array2, 0, nv2 + 1);
            vertices.CopyTo(cutLine.y, array1, cutLine.x + 1, vertices.Count - cutLine.y);

            var poly1 = new Polygon2d(array1);
            var poly2 = new Polygon2d(array2);
            poly1.RemoveCoincidentColinear(tolerance, tolerance);
            poly2.RemoveCoincidentColinear(tolerance, tolerance);
            if (poly1.IsHole || poly2.IsHole)
            { throw new ArgumentException("polygon to cut has self intersections"); }
            if (poly1.vertices.Count < 3 || poly2.vertices.Count < 3)
            { throw new ArgumentException("polygon cut result has less than 3 vertices"); }
            return (poly1, poly2);
        }

        private bool BoundsGreaterThan(double max)
        {
            if (boundsCachedTimeStamp == Timestamp)
                return cachedBounds.MinDim > max;

            //this is about failing fast, use modulo indexing to get an estimate of the bounds quicker
            var bounds = AxisAlignedBox2d.Empty;
            int tStamp = Timestamp;
            int mNumVertices = vertices.Count;
            int ii = 0;
            do
            {
                Vector2d v = vertices[ii];
                bounds.Contain(v);
                if (bounds.MinDim > max)
                    return true;
                ii = (ii + 31337) % mNumVertices;
            } while (ii != 0);
            cachedBounds = bounds;
            Interlocked.MemoryBarrier();
            boundsCachedTimeStamp = tStamp;
            return false;
        }

        private int VerticesShiftedStart()
        {
            //using the centroid yields consistent simplification results if polygons are rotated (e.g. in NFPCache)
            var centroid = VerticesAsReadOnlySpan.Centroid();
            int shiftIdx = 0;
            double distFromCentroid = 0;
            //optimize first vertex: select the vertex which is part of the bounds furthest from bounds center
            for (int idx = 0; idx < vertices.Count - 1; idx++)
            {
                var centerDist = (centroid - vertices[idx]).LengthSquared;
                if (centerDist > distFromCentroid)
                {
                    distFromCentroid = centerDist;
                    shiftIdx = idx;
                }
            }

            return shiftIdx;
        }

        /// <summary>
        /// Shifts the polygon starting point by rotating the vertices so that shiftIdx becomes the new Vertices[0]
        /// </summary>
        /// <param name="shiftIdx">the new Vertices[0]</param>
        public void ShiftVertex0(int shiftIdx)
        {
            var buffer = ShiftedBuffer(shiftIdx);
            vertices.Clear();
            vertices.AddRange(buffer);
        }

        private Vector2d[] ShiftedBuffer(int shiftIdx)
        {
            var buffer = new Vector2d[vertices.Count];
            int lastIdx = vertices.Count - shiftIdx;
            vertices.CopyTo(shiftIdx, buffer, 0, lastIdx);
            vertices.CopyTo(0, buffer, lastIdx, shiftIdx);
            return buffer;
        }

        public Line2d SegmentAsLine(int iSegment) => SegmentAsLine(vertices, iSegment);

        private Line2d SegmentAsLine(IList<Vector2d> vertices, int iSegment)
        {
            return Line2d.FromPoints(vertices[iSegment], vertices[(iSegment + 1) % vertices.Count]);
        }
        public static Polygon2d MakeCircle(Circle2d circle, int nSteps, double angleShiftRad = 0)
        {
            Polygon2d poly = SharedPolyPool.Rent(nSteps);

            double radius = circle.Radius;
            Vector2d center = circle.Center;
            for (double i = 0; i < nSteps; ++i)
            {
                double t = i / nSteps;
                double a = 2 * Math.PI * t + angleShiftRad;
                poly.AppendVertex(new Vector2d(center.x + radius * Math.Cos(a), center.y + radius * Math.Sin(a)));
            }

            return poly;
        }
        public override int GetHashCode()
        {
            Vector2d firstVert = VertexCount > 0 ? vertices[0] : Vector2d.Zero;
            return HashCode.Combine(VertexCount, firstVert);
        }

        public override bool Equals(object obj)
        {
            return Equals(obj as Polygon2d);
        }

        public bool Equals(Polygon2d other)
        {
            if (other is null) return false;
            if (VertexCount != other.VertexCount) return false;
            for (int i = 0; i < vertices.Count; i++)
            {
                if (vertices[i] != other.vertices[i]) return false;
            }
            return true;
        }

        public static bool operator ==(Polygon2d left, Polygon2d right)
        {
            return EqualityComparer<Polygon2d>.Default.Equals(left, right);
        }

        public static bool operator !=(Polygon2d left, Polygon2d right)
        {
            return !(left == right);
        }
    }

    /// <summary>
    /// Wrapper for a Polygon2d that provides minimal IParametricCurve2D interface
    /// </summary>
    public class Polygon2DCurve : IParametricCurve2d
    {
        public Polygon2d Polygon;

        public bool IsClosed { get { return true; } }

        // can call SampleT in range [0,ParamLength]
        public double ParamLength { get { return Polygon.VertexCount; } }
        public Vector2d SampleT(double t)
        {
            int i = (int)t;
            if (i >= Polygon.VertexCount - 1)
                return Polygon[Polygon.VertexCount - 1];
            Vector2d a = Polygon[i];
            Vector2d b = Polygon[i + 1];
            double alpha = t - i;
            return (1.0 - alpha) * a + (alpha) * b;
        }
        public Vector2d TangentT(double t)
        {
            throw new NotImplementedException("Polygon2dCurve.TangentT");
        }

        public bool HasArcLength { get { return true; } }
        public double ArcLength { get { return Polygon.ArcLength; } }
        public Vector2d SampleArcLength(double a)
        {
            throw new NotImplementedException("Polygon2dCurve.SampleArcLength");
        }

        public void Reverse()
        {
            Polygon.Reverse();
        }

        public IParametricCurve2d Clone()
        {
            return new Polygon2DCurve() { Polygon = this.Polygon.Duplicate() };
        }

		public bool IsTransformable { get { return true; } }
        public void Transform(ITransform2 xform)
        {
            Polygon.Transform(xform);
        }


    }
}
