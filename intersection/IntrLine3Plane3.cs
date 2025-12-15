using System;
using System.Diagnostics;
using System.Diagnostics.SymbolStore;

namespace g3
{
    // ported from Bery0za/geometry3Sharp
    // ported from WildMagic5 IntrPlane3Plane3
    // use Test() for fast boolean query, does not compute intersection info
    // use Find() to compute full information
    public class IntrLine3Plane3
    {
        Line3d line;
        public Line3d Line
        {
            get { return line; }
            set { line = value; Result = IntersectionResult.NotComputed; }
        }

        Plane3d plane;
        public Plane3d Plane
        {
            get { return plane; }
            set { plane = value; Result = IntersectionResult.NotComputed; }
        }
        
        // result flags
		public int Quantity = 0;
		public IntersectionResult Result = IntersectionResult.NotComputed;
		public IntersectionType Type = IntersectionType.Empty;
        
        // only valid if intersection type is Point
        public double LineParameter;
        
        // only valid if intersection type is Point
        public Vector3d Point;
        
        // only valid if intersection type is Line
        public Line3d CoincidentLine;

		public IntrLine3Plane3(Line3d l, Plane3d p)
		{
		    line = l;
		    plane = p;
		}

        public IntrLine3Plane3 Compute()
        {
            Find();
            return this;
        }

        public bool Find()
        {
            if (Result != IntersectionResult.NotComputed)
                return Result != IntersectionResult.NoIntersection;
            
            double DdN = line.Direction.Dot(plane.Normal);
            double signedDistance = plane.DistanceTo(line.Origin);
            
            if (Math.Abs(DdN) > MathUtil.ZeroTolerance)
            {
                // The line is not parallel to the plane, so they must intersect.
                Quantity = 1;
                LineParameter = -signedDistance / DdN;
                Point = Line.PointAt(LineParameter);
                Type = IntersectionType.Point;
                Result = IntersectionResult.Intersects;
                return true;
            }

            // The line and plane are parallel. Determine if they are numerically
            // close enough to be coincident.
            if (Math.Abs(signedDistance) <= MathUtil.ZeroTolerance)
            {
                // The line is coincident with the plane, so choose t = 0 for the
                // parameter.
                Quantity = 1;
                Point = Line.PointAt(0);
                CoincidentLine = new Line3d(line.Origin, line.Direction);
                Result = IntersectionResult.Intersects;
                Type = IntersectionType.Line;
                return true;
            }

            Result = IntersectionResult.NoIntersection;
            Type = IntersectionType.Empty;
            return false;
        }

        public bool Test()
        {
            double DdN = line.Direction.Dot(plane.Normal);
            if (Math.Abs(DdN) > MathUtil.ZeroTolerance)
            {
                // The line is not parallel to the plane, so they must intersect.
                // The line parameter is *not* set, since this is a test-intersection
                // query.
                Quantity = 1;
                Type = IntersectionType.Point;
                return true;
            }

            // The line and plane are parallel. Determine if they are numerically
            // close enough to be coincident.
            double signedDistance = plane.DistanceTo(line.Origin);
            if (Math.Abs(signedDistance) <= MathUtil.ZeroTolerance)
            {
                Quantity = 1;
                Type = IntersectionType.Line;
                return true;
            }

            Result = IntersectionResult.NoIntersection;
            Type = IntersectionType.Empty;
            return false;
        }
    }
}
