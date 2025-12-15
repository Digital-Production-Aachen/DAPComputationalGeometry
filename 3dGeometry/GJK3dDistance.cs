using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

namespace g3
{
    public class GJK3dDistance
    {
        private List<Vector3> _pointCloud;
        float lowestZ;

        public GJK3dDistance(List<Vector3> pointCloud)
        {
            _pointCloud = pointCloud;
            lowestZ = pointCloud.Min(x => x.Z);
        }

        public float GetDistance(GJK3dDistance shape2)
        {
            float tolerance = 0.1f;
            //var startDirection = _contourPolyline.GetCentroid() - shape2._contourPolyline.GetCentroid();

            var center = GetCenter(_pointCloud);
            var center2 = GetCenter(shape2._pointCloud);

            var startDirection = center - center2;

            var simplex = new Simplex3d();
            simplex.A = SupportFunction(shape2, startDirection);
            simplex.B = SupportFunction(shape2, -startDirection);

            var d = ClosestPointToOrigin(simplex.A, simplex.B);

            float dMin = d.Length();

            var counter = 0;
            while (true)
            {
                d = Vector3.Negate(d);
                if (d.Length() == 0)
                {
                    return 0;
                }

                var c = SupportFunction(shape2, d);
                var dc = Vector3.Dot(c, d);
                var da = Vector3.Dot(simplex.A, d);

                if (dc - da < tolerance)
                {
                    var distance = d.Length();
                    return distance;
                }

                var p1 = ClosestPointToOrigin(simplex.A, c);
                var p2 = ClosestPointToOrigin(c, simplex.B);

                if (p1.Length() < p2.Length())
                {
                    simplex.B = c;
                    d = p1;
                }
                else
                {
                    simplex.A = c;
                    d = p2;
                }

                dMin = Math.Min(dMin, d.Length());

                if (counter > _pointCloud.Count * 1)
                {
                    return dMin; ;
                }
                counter++;
            }
        }

        private Vector3 GetCenter(List<Vector3> vectors)
        {
            Vector3 max = vectors[0];
            Vector3 min = vectors[0];
            for (int i = 0; i < vectors.Count; i++)
            {
                max = Vector3.Max(max, vectors[i]);
                min = Vector3.Min(min, vectors[i]);
            }

            var center = min + (max - min) * 0.5f;

            return center;
        }

        private bool CheckIntersect()
        {
            return false;
        }

        private Vector3 SupportFunction(GJK3dDistance shape2, Vector3 d)
        {
            var p1 = this.GetFarthestPointInDirection(d);
            var p2 = shape2.GetFarthestPointInDirection(-d);

            return p1 - p2;
        }

        private Vector3 ClosestPointToOrigin(Vector3 a, Vector3 b)
        {
            var ab = b - a;
            var ao = Vector3.Zero - a;

            var lerp = Vector3.Dot(ao, ab) / Vector3.Dot(ab, ab);
            lerp = Math.Max(0, lerp);
            lerp = Math.Min(1, lerp);

            var point = Vector3.Lerp(a, b, lerp);
            if (point.Z < 0)
            {

            }
            return point;
        }

        private Vector3 GetDirection(Vector3 a, Vector3 b)
        {
            var ab = b - a;
            var ao = Vector3.Zero - a;

            var v3A = new Vector3(a.X, a.Y, 0);
            var v3B = new Vector3(b.X, b.Y, 0);
            var v3AB = v3B - v3A;

            return Vector3.Cross(Vector3.Cross(ab, ao), ab);
        }

        private Vector3 GetFarthestPointInDirection(Vector3 direction)
        {
            var maxDistance = float.MinValue;
            var maxIndex = 0;

            for (int i = 0; i < _pointCloud.Count; i++)
            {
                var vertex = _pointCloud[i];
                var distance = Vector3.Dot(vertex, direction);
                if (distance > maxDistance)
                {
                    maxDistance = distance;
                    maxIndex = i;
                }
            }

            return _pointCloud[maxIndex];
        }
    }

    public class Simplex3d
    {
        public Vector3 A;
        public Vector3 B;
        public Vector3 C;
    }
}
