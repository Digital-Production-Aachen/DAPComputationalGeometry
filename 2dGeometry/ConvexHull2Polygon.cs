using g3;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;

namespace g3
{

    public class ConvexHull2Polygon
    {
        private readonly DoubleEndedQueue<int> deque = new();

        public Polygon2d ConvexHull(Polygon2d poly, double tolerance = ConvexHull2.DefaultConvexHullTolerance)
        {
            var hull = ConvexHull(poly, deque, tolerance);
            deque.Clear();
            return hull;
        }

        /// <summary>
        /// Computes the convex hull of a simple polygon (an ordered vertex chain)
        /// using a linear-time algorithm (McCallum & Avis, 1979).
        /// </summary>
        /// <param name="poly"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentException">thrown when the polygon has less than 3 vertices</exception>
        public static Polygon2d ConvexHullPoly(Polygon2d poly, double tolerance = ConvexHull2.DefaultConvexHullTolerance)
            => ConvexHull(poly, new DoubleEndedQueue<int>(), tolerance);

        private static Polygon2d ConvexHull(Polygon2d poly, DoubleEndedQueue<int> deque, double tolerance)
        {
            //for anyone debugging this: do not use ConvexHullPoly on polygons that are not simple (=> are output of a Clipper2 operation)
            //pre-processing the polygons with clipper to use this method is NOT worth it runtime wise, use ConvexHull method (Quickhull algorithm) instead
            //also note this check is more expensive than the whole method, DO NOT use it in release mode!
            Debug.Assert(!poly.HasSelfIntersections());
            if (poly.VertexCount < 3)
                throw new ArgumentException("A convex hull requires at least three vertices.");

            poly.RemoveCoincidentColinear(tolerance / 4, tolerance);
            var verts = poly.VerticesAsReadOnlySpan;
            // Initialize the deque with the first three vertices.
            Vector2d p0 = verts[0];
            Vector2d p1 = verts[1];
            Vector2d p2 = verts[2];

            // Ensure the three vertices are in counterclockwise order.
            if (Orientation(p0, p1, p2) > 0)
            {
                // Counterclockwise order: initialize deque as { p2, p0, p1, p2 }.
                deque.AddToBack(2);
                deque.AddToBack(0);
                deque.AddToBack(1);
                deque.AddToBack(2);
            }
            else
            {
                // If clockwise, swap p0 and p1 to enforce counterclockwise order { p2, p1, p0, p2 }.
                deque.AddToBack(2);
                deque.AddToBack(1);
                deque.AddToBack(0);
                deque.AddToBack(2);
            }

            // Process each remaining vertex.
            for (int i = 3; i < verts.Length; i++)
            {
                p0 = poly.Vertices[i];

                // Check if pt is inside the current hull.
                // For a convex polygon with vertices in counterclockwise order, pt is inside if
                // it is to the left of the edge from the front to its next vertex
                // AND to the left of the edge from the second-last to the last vertex.
                if (Orientation(verts[deque.PeekFront()], verts[deque.PeekSecondFront()], p0) > tolerance &&
                    Orientation(verts[deque.PeekSecondBack()], verts[deque.PeekBack()], p0) > tolerance)
                {
                    continue;
                }

                double orientation;
                // Remove from the front until pt is to the left of the front edge.
                while ((orientation = Orientation(verts[deque.PeekFront()], verts[deque.PeekSecondFront()], p0)) < tolerance && deque.Count > 2)
                {
                    int poppedVtx = deque.RemoveFromFront();
                    //near colinear?
                    if (orientation > -tolerance)
                    {
                        // if both edges are co-linear, keep the longer one
                        double lengthSqrPopped = (p0 - verts[poppedVtx]).LengthSquared;
                        double lengthSqrPrev = (p0 - verts[deque.PeekFront()]).LengthSquared;
                        double lengthSqrNext = (verts[poppedVtx] - verts[deque.PeekFront()]).LengthSquared;
                        if (lengthSqrPopped > lengthSqrPrev && lengthSqrPopped > lengthSqrNext)
                        {
                            _ = deque.RemoveFromFront();
                            deque.AddToFront(poppedVtx);
                        }
                    }
                }
                deque.AddToFront(i);

                // Remove from the back until pt is to the left of the back edge.
                while ((orientation = Orientation(verts[deque.PeekSecondBack()], verts[deque.PeekBack()], p0)) < tolerance && deque.Count > 2)
                {
                    int poppedVtx = deque.RemoveFromBack();
                    //near colinear?
                    if (orientation > -tolerance)
                    {
                        // if both edges are co-linear, keep the longer one
                        double lengthSqrPopped = (p0 - verts[poppedVtx]).LengthSquared;
                        double lengthSqrPrev = (p0 - verts[deque.PeekBack()]).LengthSquared;
                        double lengthSqrNext = (verts[poppedVtx] - verts[deque.PeekBack()]).LengthSquared;
                        if (lengthSqrPopped > lengthSqrPrev && lengthSqrPopped > lengthSqrNext)
                        {
                            _ = deque.RemoveFromBack();
                            deque.AddToBack(poppedVtx);
                        }
                    }
                }
                deque.AddToBack(i);
            }

            // The first element is duplicated at the end, so we omit the duplicate.
            deque.RemoveFromBack();
            Polygon2d hull = new();
            Span<Vector2d> vtxSpan = hull.VerticesAsSpanWithCount(deque.Count);
            for (int i = 0; i < vtxSpan.Length; i++)
            {
                vtxSpan[i] = verts[deque.RemoveFromFront()];
            }
            hull.RemoveCoincidentColinear(tolerance / 4, tolerance);

            if(hull.Area < poly.Area - tolerance) { }
            Debug.Assert(hull.Area >= poly.Area - tolerance);
            return hull;
        }

        // Computes the cross product (b - a) x (c - a).
        // A positive result indicates that a, b, c are in counter clockwise order.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static double Orientation(Vector2d a, Vector2d b, Vector2d c)
        {
            return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        }

        /// <summary>
        /// Fast point-in-convex-polygon test.
        /// Assumes polygon.Vertices are in counter clockwise order.
        /// </summary>
        public static bool PointInConvexPolygon(Polygon2d poly, Vector2d pt, double tolerance = 1e-10)
        {
            var verts = poly.VerticesAsReadOnlySpan;
            Vector2d prev = verts[^1];
            for (int i = 0; i < verts.Length; i++)
            {
                Vector2d cur = verts[i];
                if (Orientation(prev, cur, pt) < -tolerance)
                    return false;
                prev = cur;
            }
            return true;
        }
    }
}
