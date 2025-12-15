using System;
using System.Diagnostics;
using System.Diagnostics.SymbolStore;

namespace g3
{
    // ported from Bery0za/geometry3Sharp
    // ported from WildMagic5 IntrPlane3Plane3
    // use Test() for fast boolean query, does not compute intersection info
    // use Find() to compute full information
    public class IntrRay3Plane3
    {
        Ray3d ray;
        public Ray3d Ray
        {
            get { return ray; }
            set { ray = value; Result = IntersectionResult.NotComputed; }
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
        public double RayParameter;
        
        // only valid if intersection type is Point
        public Vector3d Point;
        
        // only valid if intersection type is Line
        public Ray3d CoincidentRay;

		public IntrRay3Plane3(Ray3d r, Plane3d p)
		{
		    ray = r;
		    plane = p;
		}

        public IntrRay3Plane3 Compute()
        {
            Find();
            return this;
        }

        public bool Find()
        {
            if (Result != IntersectionResult.NotComputed)
                return Result != IntersectionResult.NoIntersection;
            
            Line3d line = new Line3d(ray.Origin, ray.Direction);
            IntrLine3Plane3 intr = new IntrLine3Plane3(line, plane);
            
            if (intr.Find())
            {
                // The line intersects the plane, but possibly at a point that is
                // not on the ray.
                if (intr.Type == IntersectionType.Line)
                {
                    Result = IntersectionResult.Intersects;
                    Type = IntersectionType.Line;
                    Point = intr.Point;
                    CoincidentRay = new Ray3d(ray.Origin, ray.Direction);
                    return true;
                }
                
                if (intr.Type == IntersectionType.Point)
                {
                    if (intr.LineParameter >= 0)
                    {
                        Type = IntersectionType.Point;
                        RayParameter = intr.LineParameter;
                        Point = intr.Point;
                        return true;
                    }
                }
            }

            Result = IntersectionResult.NoIntersection;
            Type = IntersectionType.Empty;
            return false;
        }

        public bool Test()
        {
            return Find();
        }
    }
}
