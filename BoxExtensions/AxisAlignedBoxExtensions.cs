using System.Linq;

namespace g3
{
    public static class AxisAlignedBoxExtensions
    {
        public static bool Contains(this AxisAlignedBox2d aabb, g3.Circle2d circle)
        {
            return
                circle.Center.x + circle.Radius < aabb.Max.x &&
                circle.Center.x - circle.Radius > aabb.Min.x &&
                circle.Center.y + circle.Radius < aabb.Max.y &&
                circle.Center.y - circle.Radius > aabb.Min.y;
        }

        public static double AreaXY(this AxisAlignedBox3d aabb)
        {
            return aabb.Width * aabb.Height;
        }

        public static bool Contains(this AxisAlignedBox2d aabb, Segment2d segment)
        {
            var p0 = segment.P0;
            var p1 = segment.P1;
            return aabb.Contains(p0) && aabb.Contains(p1);
        }

        public static double Distance(this AxisAlignedBox2d aabb, AxisAlignedBox2d other)
        {
            if(aabb.Contains(other) || other.Contains(aabb)) 
                return 0;
            if (aabb.Intersects(other) || other.Intersects(aabb))
                return 0;
            return Enumerable.Range(0, 4).Min(i => aabb.Distance(other.GetCorner(i)));
        }

        public static bool IsEmpty(this AxisAlignedBox2d aabb) => aabb.Max == AxisAlignedBox2d.Empty.Max;

        /// <summary>
        /// Compute the bounds of the trajectory when moving movedBounds along path.
        /// </summary>
        /// <param name="movedBounds">bounding box to move</param>
        /// <param name="path">2d movement path</param>
        /// <returns></returns>
        public static AxisAlignedBox3d TrajectoryBounds(this AxisAlignedBox3d movedBounds, PolyLine2d path)
        {
            AxisAlignedBox2d pathBounds = path.Bounds;
            movedBounds.Max += (pathBounds.Max - path[0]).To3d();
            movedBounds.Min -= (pathBounds.Min - path[0]).To3d();
            return movedBounds;
        }

        /// <summary>
        /// Compute the bounds of the trajectory when moving movedBounds along path.
        /// </summary>
        /// <param name="movedBounds">bounding box to move</param>
        /// <param name="path">3d movement path</param>
        /// <returns></returns>
        public static AxisAlignedBox3d TrajectoryBounds(this AxisAlignedBox3d movedBounds, PolyLine3d path)
        {
            AxisAlignedBox3d pathBounds = path.GetBounds();
            movedBounds.Max += pathBounds.Max - path[0];
            movedBounds.Min -= pathBounds.Min - path[0];
            return movedBounds;
        }
    }
}
