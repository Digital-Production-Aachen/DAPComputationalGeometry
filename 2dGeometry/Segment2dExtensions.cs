using g3;
using System;

namespace g3
{
    public static class Segment2dExtensions
    {
        public static bool Intersects(this ref Segment2d segment, AxisAlignedBox2d aabb, double tolerance = 0)
        {
            var p0 = segment.P0;
            var p1 = segment.P1;

            //check segment bounds
            if ((p0.x > aabb.Max.x && p1.x > aabb.Max.x) ||
                (p0.y > aabb.Max.y && p1.y > aabb.Max.y) ||
                (p0.x < aabb.Min.x && p1.x < aabb.Min.x) ||
                (p0.y < aabb.Min.y && p1.y < aabb.Min.y))
                return false;

            //check intersection
            var halfWidth = aabb.Width / 2;
            var halfHeight = aabb.Height / 2;
            var center = aabb.Center;
            var seg1 = new Segment2d(new Vector2d(aabb.Max.x, center.y), Vector2d.AxisY, halfHeight);
            if(seg1.Intersects(segment, tolerance, tolerance)) return true;
            var seg2 = new Segment2d(new Vector2d(aabb.Min.x, center.y), Vector2d.AxisY, halfHeight);
            if (seg2.Intersects(segment, tolerance, tolerance)) return true;
            var seg3 = new Segment2d(new Vector2d(center.x, aabb.Max.y), Vector2d.AxisX, halfWidth);
            if (seg3.Intersects(segment, tolerance, tolerance)) return true;
            var seg4 = new Segment2d(new Vector2d(center.x, aabb.Min.y), Vector2d.AxisX, halfWidth);
            return seg4.Intersects(segment, tolerance, tolerance);
        }
    }
}