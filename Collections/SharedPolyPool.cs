using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace g3
{
    public static class SharedPolyPool
    {
        static int MaxCapacity => IdxToSize(_bucketLimits.Length - 1);
        static int IdxToSize(int idx) => 4 << idx;
        static int SizeToIdx(int size) => BitOperations.Log2((uint)size) - 2;
        static int RoundUpCapa(int capacity) => capacity <= 4 ? 4 : (int)BitOperations.RoundUpToPowerOf2((uint)Math.Min(capacity, MaxCapacity));

        /// <summary>
        /// The maximum number of Polygon2ds stored in the size buckets (vertex count).
        /// Buckets start at length 4 and increase by powers of two (4, 8, 16, 32 ...).
        /// </summary>
        public static int[] BucketLimits
        {
            get => _bucketLimits;
            set
            {
                ArgumentNullException.ThrowIfNull(value, nameof(BucketLimits));
                if (BucketLimits.Length <= 0 || BucketLimits.Length > 30)
                    throw new ArgumentOutOfRangeException(nameof(BucketLimits));
                _bucketLimits = value;
            }
        }

        private static int[] _bucketLimits =
        {
            1024, // 4
            256, // 8
            256, // 16
            128, // 32
            128, // 64
            64,  // 128
            64,  // 256
            32,  // 512
            32   // 1024
        };

        [ThreadStatic]
        private static Stack<Polygon2d>[] _localBuckets;

        public static Polygon2d Rent(IReadOnlyCollection<Vector2d> vertexCollection) => Rent(vertexCollection, vertexCollection.Count);
        private static Polygon2d Rent(IEnumerable<Vector2d> vertexCollection, int count)
        {
            var poly = Rent(count);
            poly.AppendVertices(vertexCollection);
            return poly;
        }

        public static Polygon2d Rent(IReadOnlyList<double> coordinates)
        {
            if (coordinates.Count % 2 != 0) throw new ArgumentException("coordinates length must be even");
            var poly = Rent(coordinates.Count / 2);
            var verts = poly.VerticesAsSpanWithCount(coordinates.Count / 2);
            for (int i = 0; i < verts.Length; i++)
            {
                verts[i] = new Vector2d(coordinates[i * 2], coordinates[i * 2 + 1]);
            }
            return poly;
        }

        public static Polygon2d Rent(int requiredCapacity)
        {
            // only use the pool if it has been initialized (by returning Polygons to it)
            // if we don't have anything pooled (or requested size is too large) return desired capacity exactly
            if (requiredCapacity > MaxCapacity || _localBuckets is null)
            {
                return new Polygon2d(requiredCapacity);
            }
            ArgumentOutOfRangeException.ThrowIfLessThan(requiredCapacity, 0, nameof(requiredCapacity));

            requiredCapacity = RoundUpCapa(requiredCapacity);
            int bucketIndex = SizeToIdx(requiredCapacity);
            Stack<Polygon2d> bucket = _localBuckets[bucketIndex];
            if (bucket?.Count > 0)
            {
                Polygon2d poly = bucket.Pop();
                poly.ClearVertices();
                return poly;
            }

            return new Polygon2d(requiredCapacity);
        }

        public static void Return(Polygon2d poly)
        {
            int capacity = poly.VerticesCapacity;
            if (capacity > MaxCapacity || !BitOperations.IsPow2(capacity))
                return;

            // search the right bucket, lazy init as required
            int bucketIndex = SizeToIdx(capacity);
            _localBuckets ??= new Stack<Polygon2d>[BucketLimits.Length];
            ref Stack<Polygon2d> bucket = ref _localBuckets[bucketIndex];
            bucket ??= new();

            if (bucket.Count < _bucketLimits[bucketIndex])
                bucket.Push(poly);
        }
    }
}
