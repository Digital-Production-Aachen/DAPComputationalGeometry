using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Runtime.InteropServices;

namespace g3
{
	public class PolyLine3d : IEnumerable<Vector3d>, IBoundsProvider
	{
		protected List<Vector3d> vertices;
		public int Timestamp;
        private int boundsCachedTimeStamp;
        private AxisAlignedBox3d cachedBounds;

        public PolyLine3d() {
			vertices = new List<Vector3d>();
			Timestamp = 0;
		}

		public PolyLine3d(PolyLine3d copy)
		{
			vertices = new List<Vector3d>(copy.vertices);
			Timestamp = 0;
		}

		public PolyLine3d(Vector3d[] v)
		{
			vertices = new List<Vector3d>(v);
			Timestamp = 0;
		}
		public PolyLine3d(VectorArray3d v)
		{
			vertices = new List<Vector3d>(v.AsVector3d());
			Timestamp = 0;
		}

		public Vector3d this[int key]
		{
			get { return vertices[key]; }
			set { vertices[key] = value; Timestamp++; }
		}

		public Vector3d Start {
			get { return vertices[0]; }
		}
		public Vector3d End {
			get { return vertices[vertices.Count-1]; }
		}


		public ReadOnlyCollection<Vector3d> Vertices {
			get { return vertices.AsReadOnly(); }
		}

        public void SetVerticesCapacity(int capacity) => vertices.Capacity = capacity;
        public Span<Vector3d> VerticesAsSpan => CollectionsMarshal.AsSpan(vertices);
        private Span<double> VerticesAsDoubleSpan => MemoryMarshal.Cast<Vector3d, double>(CollectionsMarshal.AsSpan(vertices));
        public Span<Vector3d> VerticesAsSpanWithCount(int count)
        {
            CollectionsMarshal.SetCount(vertices, count);
            return CollectionsMarshal.AsSpan(vertices);
        }
        public ReadOnlySpan<Vector3d> VerticesAsReadOnlySpan => CollectionsMarshal.AsSpan(vertices);

        public int VertexCount
		{
			get { return vertices.Count; }
		}

		public void AppendVertex(Vector3d v)
		{
			vertices.Add(v);
			Timestamp++; 
		}


		public Vector3d GetTangent(int i)
		{
			if (i == 0)
				return (vertices[1] - vertices[0]).Normalized;
			else if (i == vertices.Count - 1)
				return (vertices[vertices.Count - 1] - vertices[vertices.Count - 2]).Normalized;
			else
				return (vertices[i + 1] - vertices[i - 1]).Normalized;
		}

        public AxisAlignedBox3d GetBounds() => Bounds;

        public AxisAlignedBox3d CalcBounds()
        {
			var vertices = VerticesAsReadOnlySpan;
            if (vertices.Length == 0)
                return AxisAlignedBox3d.Empty;
            AxisAlignedBox3d box = new AxisAlignedBox3d(vertices[0]);
            for (int i = 1; i < vertices.Length; ++i)
                box.Contain(vertices[i]);
            return box;
        }

        public AxisAlignedBox3d Bounds
        {
            get
            {
                if (Timestamp > boundsCachedTimeStamp)
                {
                    AxisAlignedBox3d calcedBounds;
                    int tStamp = Timestamp;
                    calcedBounds = CalcBounds();
                    cachedBounds = calcedBounds;
                    System.Threading.Interlocked.MemoryBarrier();
                    boundsCachedTimeStamp = tStamp;
                    return calcedBounds;
                }
                return cachedBounds;
            }
        }

        public double ArcLength
        {
            get
            {
                double fLength = 0;
                ReadOnlySpan<Vector3d> verts = VerticesAsReadOnlySpan;
                if (verts.Length == 0) return 0.0;
                Vector3d prev = verts[0], cur;
                for (int i = 1; i < verts.Length; ++i)
                {
                    cur = vertices[i];
                    fLength += prev.Distance(cur);
                    prev = cur;
                }
                return fLength;
            }
        }

        public IEnumerable<Segment3d> SegmentItr() {
			for ( int i = 0; i < vertices.Count-1; ++i )
				yield return new Segment3d( vertices[i], vertices[i+1] );
		}

		public IEnumerator<Vector3d> GetEnumerator() {
			return vertices.GetEnumerator();
		}
		IEnumerator IEnumerable.GetEnumerator() {
			return vertices.GetEnumerator();
		}
	}
}
