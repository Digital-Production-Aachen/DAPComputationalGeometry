using System;
using System.Diagnostics;

namespace g3
{
    public static class PolygonDistance
    {
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
            lerp = Math.Max(0, lerp);
            lerp = Math.Min(1, lerp);

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
    }
}
