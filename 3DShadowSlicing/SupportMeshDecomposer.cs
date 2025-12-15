using System;
using System.Collections.Generic;

namespace g3
{
    public struct OpenMeshDecompose
    {
        public DMesh3 baseMesh;
        public int[] triangleIndices;
        public HashSet<int> uniqueVertices;
        public AxisAlignedBox3d AABB;
        public Box3d OneDimBounds;
        public List<Vector2d> vertices2d;
        public int dimension;

        public static DMesh3 SimplifySupportMesh(DMesh3 supports, double lineThickness)
        {
            var simpleSupports = new DMesh3(MeshComponents.None);
            MeshConnectedComponents submeshes = new MeshConnectedComponents(supports);
            submeshes.FindConnectedT();
            MeshEditor editor = new MeshEditor(simpleSupports);
            foreach(var decomp in ConnectedComponentsAs2D(supports, lineThickness))
            {
                bool submeshAppended = false;

                if (decomp.dimension == 1)
                {
                    //the submesh shadow is a line (support), replace with inflated AABB
                    TrivialBox3Generator boxGen = new TrivialBox3Generator() { WantUVs = false, WantNormals = false, WantGroups = false };
                    boxGen.Box = decomp.OneDimBounds;
                    boxGen.Generate();
                    var box = boxGen.MakeSimpleMesh();
                    editor.AppendMesh(box);
                    submeshAppended = true;
                }
                else if (decomp.dimension == 0)
                {
                    //only possible way for this is if a triangle has all its vertices colinear to z-axis
                    //hopefully no software produces something that degenerate
                    //if it really happens we can probably throw it out
                    throw new ApplicationException("triangle has all its vertices colinear parallel to z-axis => is a point");
                }

                //if we have added nothing else, just add the triangles. As long as the projection dimension is 2, we can handle it in 2D
                if (!submeshAppended)
                    editor.AppendMesh(DSubmesh3.QuickSubmesh(supports, decomp.triangleIndices));
            }
            return simpleSupports;
        }

        public static IEnumerable<OpenMeshDecompose> ConnectedComponentsAs2D(DMesh3 openMesh, double lineThickness)
        {
            MeshConnectedComponents submeshes = new MeshConnectedComponents(openMesh);
            submeshes.FindConnectedT();
            foreach(var submesh in submeshes)
            {
                OpenMeshDecompose component = new OpenMeshDecompose();
                component.baseMesh = openMesh;
                component.triangleIndices = submesh.Indices;

                HashSet<int> uniqueVertices = new HashSet<int>(submesh.Indices.Length * 2);
                foreach (var triIdx in submesh.Indices)
                {
                    var vertIndices = openMesh.GetTriangle(triIdx);
                    //ignore duplicates (add returns false)
                    uniqueVertices.Add(vertIndices.a);
                    uniqueVertices.Add(vertIndices.b);
                    uniqueVertices.Add(vertIndices.c);
                }
                component.uniqueVertices = uniqueVertices;

                List<Vector2d> vert2d = new List<Vector2d>(uniqueVertices.Count);
                double zMin = Double.MaxValue;
                double zMax = Double.MinValue;
                foreach (var idx in uniqueVertices)
                {
                    var vertex = openMesh.GetVertex(idx);
                    zMin = Math.Min(zMin, vertex.z);
                    zMax = Math.Max(zMax, vertex.z);
                    vert2d.Add(new Vector2d(vertex.x, vertex.y));
                }
                component.vertices2d = vert2d;

                Vector2d.GetInformation(vert2d, MathUtil.ZeroTolerancef, out var vectorInfo);
                component.dimension = vectorInfo.mDimension;

                component.AABB = new AxisAlignedBox3d(
                    new Vector3d(vectorInfo.mMin.x, vectorInfo.mMin.y, zMin),
                    new Vector3d(vectorInfo.mMax.x, vectorInfo.mMax.y, zMax));

                if (vectorInfo.mDimension == 1)
                {
                    //the submesh shadow is a line (support), construct inflated AABB
                    var center2D = vectorInfo.mMin + (vectorInfo.mMax - vectorInfo.mMin) / 2;
                    var center = new Vector3d(center2D.x, center2D.y, zMin + (zMax - zMin) / 2);
                    component.OneDimBounds = new Box3d(center,
                        new Vector3d(vectorInfo.mDirection0.x, vectorInfo.mDirection0.y, 0),
                        new Vector3d(vectorInfo.mDirection1.x, vectorInfo.mDirection1.y, 0),
                        Vector3d.AxisZ,
                        new Vector3d(vectorInfo.mMaxRange / 2 + lineThickness / 2, lineThickness / 2, (zMax - zMin) / 2));
                }
                yield return component;
            }
        }
    }
}
