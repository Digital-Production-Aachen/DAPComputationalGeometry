using OpenVectorFormat;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace g3
{
    public static class TypeConversions
    {
        /// <summary>
        /// Convert a set of euler angles to a Quaternion.
        /// See https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles.
        /// Order of rotations is extrinsic XYZ (same as intrinsic ZYX)
        /// </summary>
        /// <param name="roll">rotation around x-axis in degree</param>
        /// <param name="pitch">rotation around y-axis in degree</param>
        /// <param name="yaw">rotation around z-axis in degree</param>
        /// <returns>Quaternionf representing the rotation</returns>
        public static Quaternionf EulerDToQuaternion(float roll, float pitch, float yaw)
        {
            // degree to radians
            yaw *= (float)Math.PI / 180;
            roll *= (float)Math.PI / 180;
            pitch *= (float)Math.PI / 180;

            // convert to quaternion
            float cr = (float)Math.Cos(roll * 0.5);
            float sr = (float)Math.Sin(roll * 0.5);
            float cp = (float)Math.Cos(pitch * 0.5);
            float sp = (float)Math.Sin(pitch * 0.5);
            float cy = (float)Math.Cos(yaw * 0.5);
            float sy = (float)Math.Sin(yaw * 0.5);

            Quaternionf q;
            q.w = cr * cp * cy + sr * sp * sy;
            q.x = sr * cp * cy - cr * sp * sy;
            q.y = cr * sp * cy + sr * cp * sy;
            q.z = cr * cp * sy - sr * sp * cy;
            return q;
        }

        public static IEnumerable<Vector2> Vector2Vertices(this g3.Polygon2d polygon2D)
        {
            return polygon2D.Vertices.Select(vertex => new Vector2((float)vertex.x, (float)vertex.y));
        }

        public static Vector2f[] VerticesAsF(this g3.Polygon2d polygon)
        {
            var vertices = polygon.Vertices;
            var array = new Vector2f[vertices.Count];
            for (int i = 0; i < vertices.Count; i++)
            {
                array[i] = new Vector2f(vertices[i]);
            }
            return array;
        }

        public static Polygon2d[] ToPolygon2d(this Vector2f[][] verticesF)
        {
            var polygon2ds = new Polygon2d[verticesF.Length];
            for (int i = 0; i < verticesF.Length; i++)
            {
                polygon2ds[i] = new Polygon2d(verticesF[i]);
            }
            return polygon2ds;
        }

        public static Vector2f[][] ToVerticesF(this IList<Polygon2d> polygons)
        {
            var verticesF = new Vector2f[polygons.Count][];
            for (int i = 0; i < polygons.Count; i++)
            {
                verticesF[i] = polygons[i].VerticesAsF();
            }
            return verticesF;
        }

        public static Polygon2d ToPolygon2d(this IEnumerable<Vector2> polygon)
        {
            return new Polygon2d(polygon.Select(x => x.ToVector2d()));
        }

        public static AxisAlignedBox2f To2DG3AABBf(this AxisAlignedBox2D aabb)
        {
            return new AxisAlignedBox2f(aabb.XMin, aabb.YMin, aabb.XMax, aabb.YMax);
        }

        public static AxisAlignedBox2d ToAAB(this Vector2d.Information vectorInfo)
        {
            return new AxisAlignedBox2d(
                    new Vector2d(vectorInfo.mMin.x, vectorInfo.mMin.y),
                    new Vector2d(vectorInfo.mMax.x, vectorInfo.mMax.y));
        }

        public static AxisAlignedBox2d As2d(this AxisAlignedBox3d aabb)
        {
            return new AxisAlignedBox2d(aabb.Min.To2d(), aabb.Max.To2d());
        }

        public static AxisAlignedBox3d As3d(this AxisAlignedBox2d aabb)
        {
            return new AxisAlignedBox3d(aabb.Min.To3d(), aabb.Max.To3d());
        }

        public static AxisAlignedBox2f ToAxisAlignedBox2f(this AxisAlignedBox2d aabb)
        {
            return new AxisAlignedBox2f((float)aabb.Min.x, (float)aabb.Min.y,
                (float)aabb.Max.x, (float)aabb.Max.y);
        }

        public static Box2d ToBox(this Vector2d.Information vectorInfo, double inflation)
        {
            var center2D = vectorInfo.mMin + (vectorInfo.mMax - vectorInfo.mMin) / 2;
            return new Box2d(center2D,
                new Vector2d(vectorInfo.mDirection0.x, vectorInfo.mDirection0.y),
                new Vector2d(vectorInfo.mDirection1.x, vectorInfo.mDirection1.y),
                new Vector2d(vectorInfo.mMaxRange / 2, inflation / 2));
        }

        public static Box3d To3d(this Box2d box, double depth)
        {
            return new Box3d(new Vector3d(box.Center.x, box.Center.y, depth / 2), new Vector3d(box.Extent.x, box.Extent.y, depth / 2));
        }

        /// <summary>
        /// Convert polygon to OVF line sequence
        /// </summary>
        /// <param name="polygon"></param>
        /// <returns></returns>
        public static VectorBlock.Types.LineSequence ToOVF(this g3.Polygon2d polygon)
        {
            var lineSequence = new VectorBlock.Types.LineSequence();
            var verts = polygon.Vertices;
            lineSequence.Points.Capacity = verts.Count * 2 + 2;
            foreach (var vert in verts)
            {
                lineSequence.Points.Add((float)vert.x);
                lineSequence.Points.Add((float)vert.y);
            }
            RepeatStart(lineSequence);
            return lineSequence;
        }

        //ToDo
        public static Polygon2d ToPolygon2d(this VectorBlock.Types.LineSequence lineSequence)
        {
            throw new NotImplementedException();
        }

        public static PolyLine2d ToPolyLine(this Line2d line, double length = 1)
        {
            return new PolyLine2d(new Vector2d[2] { line.Origin, line.Origin + line.Direction * length });
        }

        /// <summary>
        /// Convert polygon to OVF line sequence and transform vertices to local coordinate system of frame.
        /// </summary>
        /// <param name="polygon"></param>
        /// <param name="frame"></param>
        /// <returns></returns>
        public static VectorBlock.Types.LineSequence ToOVF(this g3.Polygon2d polygon, Frame3d frame)
        {
            var lineSequence = new VectorBlock.Types.LineSequence();
            foreach (var vert in polygon.Vertices)
            {
                Vector3d transformed = frame.ToFrameP(vert.To3d());
                lineSequence.Points.Add((float)transformed.x);
                lineSequence.Points.Add((float)transformed.y);
            }
            RepeatStart(lineSequence);
            return lineSequence;
        }

        /// <summary>
        /// g3 polys are implicitly closed, OVF LineSequences might be open
        /// and have to be closed by repeating the first/last vertex
        /// </summary>
        /// <param name="lineSeq"></param>
        private static void RepeatStart(VectorBlock.Types.LineSequence lineSeq)
        {
            if (lineSeq.Points.Count >= 2)
            {
                lineSeq.Points.Add(lineSeq.Points[0]);
                lineSeq.Points.Add(lineSeq.Points[1]);
            }
        }

        public static IEnumerable<VectorBlock.Types.LineSequence> ToOVF(this g3.GeneralPolygon2d polygon)
        {
            yield return polygon.Outer.ToOVF();
            foreach (var hole in polygon.Holes) yield return hole.ToOVF();
        }

        public static VectorBlock.Types.LineSequence ToOVF(this AxisAlignedBox2d aabb)
        {
            var lineSequence = new VectorBlock.Types.LineSequence();
            var points = lineSequence.Points;
            points.Capacity = 10;

            Vector2f min = (Vector2f)aabb.Min, max = (Vector2f)aabb.Max;

            points.Add(min.x);
            points.Add(min.y);
            points.Add(max.x);
            points.Add(min.y);
            points.Add(max.x);
            points.Add(max.y);
            points.Add(min.x);
            points.Add(max.y);
            points.Add(min.x);
            points.Add(min.y);

            return lineSequence;
        }

        public static VectorBlock MakeBlock(this VectorBlock.Types.Arcs arcs, int laserIndex = 0, int partIndex = 0)
        {
            var block = InitVectorBlock(laserIndex, partIndex);
            block.LpbfMetadata.PartArea = VectorBlock.Types.PartArea.Contour;
            block.Arcs = arcs;
            return block;
        }

        public static VectorBlock MakeBlock(this VectorBlock.Types.Hatches hatches, int laserIndex = 0, int partIndex = 0)
        {
            var block = InitVectorBlock(laserIndex, partIndex);
            block.LpbfMetadata.PartArea = VectorBlock.Types.PartArea.Volume;
            block.Hatches = hatches;
            return block;
        }

        public static VectorBlock MakeBlock(this VectorBlock.Types.LineSequence lineSequence, int laserIndex = 0, int partIndex = 0)
        {
            var block = InitVectorBlock(laserIndex, partIndex);
            block.LpbfMetadata.PartArea = VectorBlock.Types.PartArea.Contour;
            block.LineSequence = lineSequence;
            return block;
        }

        public static VectorBlock InitVectorBlock(int laserIndex = 0, int partIndex = 0)
        {
            return new VectorBlock()
            {
                MetaData = new VectorBlock.Types.VectorBlockMetaData() { PartKey = partIndex },
                LpbfMetadata = new VectorBlock.Types.LPBFMetadata(),
                LaserIndex = laserIndex
            };
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2 ToVector2(this Vector2d vector)
        {
            return new Vector2((float)vector.x, (float)vector.y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2 ToVector2(this Vector2f vector)
        {
            return new Vector2(vector.x, vector.y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2f ToVector2f(this Vector2 vector)
        {
            return new Vector2f(vector.X, vector.Y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2d ToVector2d(this Vector2 vector)
        {
            return new Vector2d(vector.X, vector.Y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d To3d(this Vector2d vector)
        {
            return new Vector3d(vector.x, vector.y, 0);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d To3d(this Vector2d vector, double z)
        {
            return new Vector3d(vector.x, vector.y, z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 ToVector3(this Vector3d vector)
        {
            return new Vector3((float)vector.x, (float)vector.y, (float)vector.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 ToVector3(this Vector3f vector)
        {
            return new Vector3(vector.x, vector.y, vector.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3f ToVector3f(this Vector3 vector)
        {
            return new Vector3f(vector.X, vector.Y, vector.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d ToVector3d(this Vector3 vector)
        {
            return new Vector3d(vector.X, vector.Y, vector.Z);
        }

        /// <summary>
        /// Project the vertices of mesh with the given indices into the x-y-plane (set z=0).
        /// </summary>
        /// <param name="mesh"></param>
        /// <param name="indices"></param>
        /// <returns>the resulting polygon</returns>
        public static Polygon2d PolygonFromVtxIndices(DMesh3 mesh, int[] indices)
        {
            var polygon = SharedPolyPool.Rent(indices.Length);
            foreach (var idx in indices)
            {
                if (!mesh.IsVertex(idx)) continue;
                var vtx = mesh.GetVertex(idx);
                //project into x/y plane
                polygon.AppendVertex(new Vector2d(vtx.x, vtx.y));
            }
            return polygon;
        }

        public static Polygon2d ToPolygon2d(this Box2d box) => new Polygon2d(box.ComputeVertices());

        public static Polygon2d ToPolygon2d(this AxisAlignedBox2d aabb)
        {
            var vertices = new Vector2d[4];
            vertices[0] = aabb.GetCorner(0);
            vertices[1] = aabb.GetCorner(1);
            vertices[2] = aabb.GetCorner(2);
            vertices[3] = aabb.GetCorner(3);
            return new Polygon2d(vertices);
        }

        public static PolyLine2d ToPolyLine(this Segment2d seg)
        {
            return new PolyLine2d([seg.P0, seg.P1]);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2d To2d(this Vector3d vector)
        {
            return new Vector2d(vector.x, vector.y);
        }
    }
}
