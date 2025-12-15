using System;
using System.Collections.Generic;
using System.Linq;

namespace g3
{
    public static class DMesh3Extensions
    {
        public static DMesh3 ToMeshMarchingCubes(this BoundedImplicitFunction3d root, int numcells)
        {
            MarchingCubes c = new MarchingCubes();
            c.Implicit = root;
            c.RootMode = MarchingCubes.RootfindingModes.LerpSteps;      // cube-edge convergence method
            c.RootModeSteps = 5;                                        // number of iterations
            c.Bounds = root.Bounds();
            c.CubeSize = c.Bounds.MaxDim / numcells;
            c.Bounds.Expand(3 * c.CubeSize);                            // leave a buffer of cells
            c.Generate();
            MeshNormals.QuickCompute(c.Mesh);                           // generate normals
            return c.Mesh;
        }

        public static (double volume, double area) VolumeArea(this DMesh3 mesh) 
        {
            var volArea = MeshMeasurements.VolumeArea(mesh, mesh.TriangleIndices(), mesh.GetVertex);
            return (volArea.x, volArea.y);
        }        

        public static Polygon2d ConvexShadow(this DMesh3 mesh, double epsilon = ConvexHull2.DefaultConvexHullTolerance)
        {
            List<Vector2d> projectedVertices = new List<Vector2d>(mesh.VertexCount);
            projectedVertices.AddRange(mesh.Vertices().Select(v => v.To2d()));
            return projectedVertices.ConvexHull(epsilon);
        }

        public static Polygon2d ConvexShadow(this IList<Vector3d> vertices, double epsilon = ConvexHull2.DefaultConvexHullTolerance)
        {
            List<Vector2d> projectedVertices = new List<Vector2d>(vertices.Count);
            projectedVertices.AddRange(vertices.Select(v => v.To2d()));
            return projectedVertices.ConvexHull(epsilon);
        }

        public static bool TriIsUpSkin(this DMesh3 dmesh, int triID)
        {
            return Vector3d.Dot(Vector3d.AxisZ, dmesh.GetTriNormal(triID)) > MathUtil.ZeroTolerancef;
        }

        public static bool TriIsDownSkin(this DMesh3 dmesh, int triID)
        {
            return Vector3d.Dot(Vector3d.AxisZ, dmesh.GetTriNormal(triID)) < MathUtil.ZeroTolerancef;
        }

        public static void GetTriVertices2D(this DMesh3 mesh, int tID, ref Vector2d v0, ref Vector2d v1, ref Vector2d v2)
        {
            var triangles = mesh.TrianglesBuffer;
            var vertices = mesh.VerticesBuffer;
            int aIdx = 3 * triangles[3 * tID];
            v0.x = vertices[aIdx];
            v0.y = vertices[aIdx + 1];
            int bIdx = 3 * triangles[3 * tID + 1];
            v1.x = vertices[bIdx];
            v1.y = vertices[bIdx + 1];
            int cIdx = 3 * triangles[3 * tID + 2];
            v2.x = vertices[cIdx];
            v2.y = vertices[cIdx + 1];
        }

        /// <summary>
        /// Approximately minimal oriented 3d bounding box (guaranties to fit all mesh vertices).
        /// Fits mesh vertices with a Gaussian distribution. The center is the mean of the
        /// points, the axes are the eigenvectors of the covariance matrix, and the
        /// extents are the eigenvalues of the covariance matrix and are returned in
        /// increasing order.
        /// </summary>
        /// <param name="mesh"></param>
        /// <returns></returns>
        public static Box3d OrientedBounds(this DMesh3 mesh) => OrientedBounds(mesh.Vertices());

        public static Box3d OrientedBounds(this IEnumerable<Vector3d> points)
        {
            GaussPointsFit3 gaussPointsFit = new GaussPointsFit3(points);
            if (!gaussPointsFit.ResultValid)
                throw new InvalidOperationException($"failed to compute oriented bounds 3d");
            //guarantee box fits all vertices
            gaussPointsFit.Box.Contain(points);
            return gaussPointsFit.Box;
        }

        public static BoundedImplicitFunction3d ToImplicitFunctionDGridTriL(this DMesh3 meshIn, int numcells, double max_offset) {
            double meshCellsize = meshIn.CachedBounds.MaxDim / numcells;
            MeshSignedDistanceGrid levelSet = new MeshSignedDistanceGrid(meshIn, meshCellsize);
            levelSet.ExactBandWidth = (int)(Math.Abs(max_offset) / meshCellsize) + 1;
            levelSet.Compute();
            return new DenseGridTrilinearImplicit(levelSet.Grid, levelSet.GridOrigin, levelSet.CellSize);
        }

        public static DMesh3 QuickSubmesh(this DMesh3 mesh, IEnumerable<int> triangles, MeshComponents WantComponents = MeshComponents.All, int nTriEstimate = 0)
        {
            var submesh = new DSubmesh3(mesh);
            if (triangles is ICollection<int> collection) 
                nTriEstimate = collection.Count;
            submesh.WantComponents = WantComponents;
            submesh.Compute(triangles, nTriEstimate);
            if (submesh.SubMesh == null) { }
            return submesh.SubMesh;
        }

        public static DMesh3 FilteredTrianglesSubmesh(this DMesh3 mesh, Func<DMesh3, int, bool> triangleFilter, MeshComponents WantComponents = MeshComponents.All)
        {
            var submesh = new DSubmesh3(mesh);
            submesh.WantComponents = WantComponents;
            //this injects the filter condition using the enumerator without using additional memory
            submesh.Compute(MeshIterators.FilteredTriangles(mesh, triangleFilter));
            return submesh.SubMesh;
        }

        public static DMesh3 FilteredVerticesSubmesh(this DMesh3 mesh, Func<DMesh3, int, bool> vertexFilter, MeshComponents WantComponents = MeshComponents.All)
        {
            var submesh = new DSubmesh3(mesh);
            submesh.WantComponents = WantComponents;
            //this injects the filter condition using the enumerator without using additional memory
            submesh.Compute(MeshIterators.FilteredVertices(mesh, vertexFilter));
            return submesh.SubMesh;
        }

        public static DMesh3 FilteredEdgesSubmesh(this DMesh3 mesh, Func<DMesh3, int, bool> edgesFilter, MeshComponents WantComponents = MeshComponents.All)
        {
            var submesh = new DSubmesh3(mesh);
            submesh.WantComponents = WantComponents;
            //this injects the filter condition using the enumerator without using additional memory
            submesh.Compute(MeshIterators.FilteredEdges(mesh, edgesFilter));
            return submesh.SubMesh;
        }

        public static DMesh3[] Separate(this DMesh3 mesh)
        {
            return MeshConnectedComponents.Separate(mesh);
        }
    }
}
