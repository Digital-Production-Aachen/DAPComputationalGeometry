using System;
using System.Collections.Generic;

namespace g3
{
    public static class BoxExtensions
    {
        public static void Contain(this ref Box2d box, g3.Polygon2d poly)
        {
            foreach (var vert in poly.Vertices) { box.Contain(vert); }
        }

        public static void Contain(this ref Box2d box, IEnumerable<Polygon2d> polys)
        {
            foreach (var poly in polys) { box.Contain(poly); }
        }

        public static void Contain(this ref Box2d box, GeneralPolygon2d genPoly) => box.Contain(genPoly.Outer);

        public static void Contain(this ref Box2d box, IEnumerable<GeneralPolygon2d> genPolys)
        {
            foreach (var poly in genPolys) { box.Contain(poly.Outer); }
        }

        public static bool Intersects(this Box3d box1, Box3d box2)
        {
            if (box1.Contains(box2.Center))
                return true;
            if (box2.Contains(box1.Center))
                return true;

            foreach (var corner in box1.ComputeVertices())
                if (box2.Contains(corner))
                    return true;
            foreach (var corner in box2.ComputeVertices())
                if (box1.Contains(corner))
                    return true;

            Frame3f frame1 = box1.GetFrame();
            var center2 = frame1.ToFrameP((Vector3f)box2.Center);
            var extent2 = frame1.ToFrameV(box2.Extent);

            // Calculate the absolute minimum and maximum extents of the two boxes in the local space of box1
            Vector3d min1 = -box1.Extent;
            Vector3d max1 = box1.Extent;
            Vector3d min2 = center2 - extent2;
            Vector3d max2 = center2 + extent2;

            return IntervalOverlaps(min1, max1, min2, max2);
        }

        public static bool Intersects(this AxisAlignedBox3d box1, AxisAlignedBox3d box2)
            => IntervalOverlaps(box1.Min, box1.Max, box2.Min, box2.Max);

        private static bool IntervalOverlaps(Vector3d min1, Vector3d max1, Vector3d min2, Vector3d max2)
        {
            // Check for overlap in each dimension
            bool overlapX = (min1.x <= max2.x && max1.x >= min2.x);
            bool overlapY = (min1.y <= max2.y && max1.y >= min2.y);
            bool overlapZ = (min1.z <= max2.z && max1.z >= min2.z);

            // If there's overlap in all three dimensions, the boxes intersect
            return overlapX && overlapY && overlapZ;
        }

        public static Frame3f GetFrame(this Box3d box)
        {
            var frame = new Frame3f();
            frame.Origin = (Vector3f)box.Center;
            if (box.AxisX != Vector3d.AxisX)
                frame.Rotation = new Quaternionf(Vector3f.AxisX, (Vector3f)box.AxisX);
            else if (box.AxisY != Vector3d.AxisY)
                frame.Rotation = new Quaternionf(Vector3f.AxisY, (Vector3f)box.AxisY);
            else if (box.AxisZ != Vector3d.AxisZ)
                frame.Rotation = new Quaternionf(Vector3f.AxisZ, (Vector3f)box.AxisZ);
            return frame;
        }

        public static void ExpandXY(this AxisAlignedBox3d box, double fRadius)
        {
            box.Min.x -= fRadius;
            box.Min.y -= fRadius;
            box.Max.x += fRadius;
            box.Max.y += fRadius;
        }

        public static bool ApproximatelyEquals(this AxisAlignedBox3d aabb1, AxisAlignedBox3d aabb2, double tolerance = 1e-6f)
        {
            return
                Math.Abs(aabb1.Min.x - aabb2.Min.x) < tolerance
                && Math.Abs(aabb1.Min.y - aabb2.Min.y) < tolerance
                && Math.Abs(aabb1.Min.z - aabb2.Min.z) < tolerance
                && Math.Abs(aabb1.Max.x - aabb2.Max.x) < tolerance
                && Math.Abs(aabb1.Max.y - aabb2.Max.y) < tolerance
                && Math.Abs(aabb1.Max.z - aabb2.Max.z) < tolerance;
        }

        public static double RotZInDeg(this Box2d box) => Utils.Rad2Deg(Math.Atan(box.AxisX.y / box.AxisX.x));


        public static double AspectRatio(this AxisAlignedBox2d aabb) => aabb.MaxDim / aabb.MinDim;
        public static double AspectRatio(this AxisAlignedBox3d aabb) => aabb.MaxDim / aabb.MinDim();
        public static double AspectRatio(this Box2d obb) => obb.MaxExtent / obb.MinExtent;
        public static double AspectRatio(this Box3d obb) => obb.MaxExtent / obb.MinExtent;

        public static double MinDim(this AxisAlignedBox3d aabb) => Math.Min(aabb.Width, Math.Min(aabb.Height, aabb.Depth));
        public static double MinDimXY(this AxisAlignedBox3d aabb) => Math.Min(aabb.Width, aabb.Height);

        public static Line2d MaxExtendAxis(this Box2d box) => new Line2d(box.Center, box.Extent.x >= box.Extent.y ? box.AxisX : box.AxisY);

        public static Line2d MinExtendAxis(this Box2d box) => new Line2d(box.Center, box.Extent.x >= box.Extent.y ? box.AxisY : box.AxisX);
    }
}
