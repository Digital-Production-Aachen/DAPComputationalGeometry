using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace g3
{
    public struct MeshZPartition
    {
        public DMesh3 mesh;
        public double zStart;
        public double zEnd;
        public ConcurrentBag<int> triangleIndices;

        public DMesh3 MakeSubmesh(MeshComponents components = MeshComponents.All)
        {
            DMesh3 subMesh = mesh.QuickSubmesh(triangleIndices, components);
            subMesh.PartName = PartitionName;
            return subMesh;
        }

        public static IEnumerable<MeshZPartition> PartionMeshInZ(DMesh3 mesh, double layerThickness, double layerOverlap, int roundingDigits = 15)
        {
            double zStart = Math.Round(mesh.CachedBounds.Min.z, roundingDigits);
            zStart = Utils.SnapToIntervalFloor(zStart, layerThickness, roundingDigits);

            //we pull geometry from below only to realize the overlap, which only extends to other / upper bounds
            double zEnd = Math.Round(mesh.CachedBounds.Max.z + layerOverlap, roundingDigits);
            zEnd = Utils.SnapToIntervalCeiling(zEnd, layerThickness, roundingDigits);
            double maxRoundingError = 0.5 * Math.Pow(10, -roundingDigits);

            int numLayers = Convert.ToInt32((zEnd - zStart) / layerThickness);
            var partitions = new MeshZPartition[numLayers];

            for(int i = 0; i < numLayers; i++)
            {
                partitions[i].mesh = mesh;
                partitions[i].zStart = zStart + i * layerThickness - layerOverlap;
                partitions[i].zEnd = zStart + (i + 1) * layerThickness;
                partitions[i].triangleIndices = new ConcurrentBag<int>();
            }

            var triangles = mesh.TrianglesBuffer;
            var vertices = mesh.VerticesBuffer;
            var pOpts = new ParallelOptions() { MaxDegreeOfParallelism = triangles.Length > 1024 * 1024 ? Environment.ProcessorCount : 1 };
            Parallel.ForEach(mesh.TriangleIndices(), pOpts, tID =>
            {
                //low level compare directly on the buffers, adapted from mesh.GetTriBounds()
                double z = vertices[3 * triangles[3 * tID] + 2];
                double minz = z, maxz = z;
                for (int i = 1; i < 3; ++i)
                {
                    z = vertices[3 * triangles[3 * tID + i] + 2];
                    if (z < minz) minz = z; else if (z > maxz) maxz = z;
                }

                int layerNr = Math.Max((int)((minz - layerOverlap - zStart) / layerThickness), 0);
                while (layerNr < partitions.Length && partitions[layerNr].zStart < maxz + maxRoundingError)
                {
                    partitions[layerNr].triangleIndices.Add(tID);
                    layerNr++;
                }
            });

            return partitions.Where(x => x.triangleIndices.Count > 0);
        }

        public string PartitionName
        {
            get
            {
                string partName = mesh.PartName;
                return $"{partName}_slice_{zStart.ToString("F3")}-{zEnd.ToString("F3")}";
            }
        }

        public override string ToString() => PartitionName;
    }
}
