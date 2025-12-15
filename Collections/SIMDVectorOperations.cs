using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace g3
{
    public static class SIMDVectorOperations
    {
        /// <summary>
        /// Interprets a span of doubles as an array of Vector2 structs [x0 y0 x1 y1 ...]
        /// and translates all the coordinates by translation, using SIMD hardware acceleration if available.
        /// </summary>
        /// <param name="coordinates"></param>
        /// <param name="translation"></param>
        /// <exception cref="ArgumentException"></exception>
        public static void TranslateAsVector2(Span<double> coordinates, g3.Vector2d translation)
        {
            Debug.Assert(coordinates.Length % 2 == 0);

            //did some benchmarks (on AVX2 capable hardware) to estimate the threshold of the overhead
            if (coordinates.Length > Vector<double>.Count * 6 && Vector.IsHardwareAccelerated && Vector<double>.Count % 2 == 0)
            {
                var vecSpan = MemoryMarshal.Cast<double, Vector<double>>(coordinates);
                int chunkSize = Vector<double>.Count;

                Span<double> inputVec = stackalloc double[chunkSize];

                for (int i = 0; i < chunkSize - 1; i += 2)
                {
                    inputVec[i] = translation.x;
                    inputVec[i + 1] = translation.y;
                }

                var addVec = new Vector<double>(inputVec);

                for (int i = 0; i < vecSpan.Length; i++)
                {
                    vecSpan[i] += addVec;
                }

                //slice to let the fallback handle the rest
                coordinates = coordinates.Slice(vecSpan.Length * chunkSize);
            }

            var vec2Span = MemoryMarshal.Cast<double, g3.Vector2d>(coordinates);
            for (int i = 0; i < vec2Span.Length; i++)
            {
                vec2Span[i] += translation;
            }
        }

        /// <summary>
        /// Interprets a span of doubles as an array of Vector2 structs [x0 y0 x1 y1 ...]
        /// and translates x and y the coordinates by translation.
        /// Uses SIMD hardware acceleration if Avx2 is available.
        /// </summary>
        /// <param name="coordinates"></param>
        /// <param name="angleRad"></param>
        /// <exception cref="ArgumentException"></exception>
        public static void RotateAsVector2(Span<double> coordinates, Matrix2d rotation)
        {
            Debug.Assert(coordinates.Length % 2 == 0);

            int noSIMDStartIndex = 0;

            //the variable size Vector does not have permute/shuffle methods that we need for a matrix multiplication
            //so instead we use Avx2 with Vector256 if available (and did not bother to create a separate Vector128 code path)
            if (Avx2.IsSupported & coordinates.Length > Vector256<double>.Count * 2)
            {
                int chunkSize = Vector256<double>.Count;
                var vec1 = Vector256.Create(rotation.m00, rotation.m11, rotation.m00, rotation.m11);
                var vec2 = Vector256.Create(rotation.m10, rotation.m01, rotation.m10, rotation.m01);
                const byte shuffleMask = 0b10110001;//(1, 0, 3, 2);

                var vec256Span = MemoryMarshal.Cast<double, Vector256<double>>(coordinates);
                for (int i = 0; i < vec256Span.Length; i++)
                {
                    var sumCos = Avx2.Multiply(vec256Span[i], vec1);
                    var sumSin = Avx2.Multiply(vec256Span[i], vec2);
                    var sumSinShuffled = Avx2.Permute4x64(sumSin, shuffleMask);
                    vec256Span[i] = Avx2.Add(sumCos, sumSinShuffled);
                }
                noSIMDStartIndex = vec256Span.Length * chunkSize;
            }

            for (int i = noSIMDStartIndex; i < coordinates.Length - 1; i += 2)
            {
                var x = coordinates[i];
                var y = coordinates[i + 1];
                coordinates[i    ] = x * rotation.m00 + y * rotation.m01;
                coordinates[i + 1] = x * rotation.m10 + y * rotation.m11;
            }
        }

        /// <summary>
        /// Interprets a span of floats as an array of Vector2 structs [x0 y0 x1 y1 ...]
        /// and calculates the 2D (x any y) axis aligned bounding box of the coordinates.
        /// Uses SIMD hardware acceleration if available.
        /// </summary>
        /// <param name="coordinates"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentException"></exception>
        public static AxisAlignedBox2d Bounds2D(ReadOnlySpan<double> coordinates)
        {
            Debug.Assert(coordinates.Length % 2 == 0);

            int noSIMDStartIdx = 2;
            var bounds = AxisAlignedBox2d.Empty;

            if (coordinates.Length >= Vector<double>.Count)
            {
                var vecSpan = MemoryMarshal.Cast<double, Vector<double>>(coordinates);

                var minVector = vecSpan[0];
                var maxVector = vecSpan[0];

                for (int i = 1; i < vecSpan.Length; i++)
                {
                    minVector = Vector.Min(minVector, vecSpan[i]);
                    maxVector = Vector.Max(maxVector, vecSpan[i]);
                }

                noSIMDStartIdx = vecSpan.Length * Vector<double>.Count;

                bounds.Min.x = minVector[0];
                bounds.Min.y = minVector[1];
                bounds.Max.x = maxVector[0];
                bounds.Max.y = maxVector[1];

                for (int i = 2; i < Vector<double>.Count - 1; i += 2)
                {
                    bounds.Min.x = Math.Min(bounds.Min.x, minVector[i]);
                    bounds.Min.y = Math.Min(bounds.Min.y, minVector[i + 1]);
                    bounds.Max.x = Math.Max(bounds.Max.x, maxVector[i]);
                    bounds.Max.y = Math.Max(bounds.Max.y, maxVector[i + 1]);
                }
            }
            else if(coordinates.Length >= 2)
            {
                bounds.Min.x = coordinates[0];
                bounds.Min.y = coordinates[1];
                bounds.Max.x = coordinates[0];
                bounds.Max.y = coordinates[1];
            }

            for (int i = noSIMDStartIdx; i < coordinates.Length - 1; i += 2)
            {
                if (coordinates[i] < bounds.Min.x) bounds.Min.x = coordinates[i];
                if (coordinates[i + 1] < bounds.Min.y) bounds.Min.y = coordinates[i + 1];
                if (coordinates[i] > bounds.Max.x) bounds.Max.x = coordinates[i];
                if (coordinates[i + 1] > bounds.Max.y) bounds.Max.y = coordinates[i + 1];
            }

            return bounds;
        }

        public static double Area2D(ReadOnlySpan<double> coordinates)
        {
            if (coordinates.Length < 6) return 0;
            Debug.Assert(coordinates.Length % 2 == 0);

            int noSIMDStartIndex = 0;
            double area = 0;
            //the variable size Vector does not have permute/shuffle methods that we need for a matrix multiplication
            //so instead we use Avx2 with Vector256 if available (and did not bother to create a separate Vector128 code path)
            if (Avx2.IsSupported & coordinates.Length > Vector256<double>.Count * 2)
            {
                int chunkSize = Vector256<double>.Count;

                var vec256Span = MemoryMarshal.Cast<double, Vector256<double>>(coordinates);
                var vec256SpanShifted = MemoryMarshal.Cast<double, Vector256<double>>(coordinates.Slice(2));

                const byte shuffleMask = 0b10110001;//(1, 0, 3, 2)

                var accumulator = Vector256<double>.Zero;
                for (int i = 0; i < vec256SpanShifted.Length; i++)
                {
                    var vecBase = vec256Span[i];
                    var vecNext = vec256SpanShifted[i];
                    var vecPermute = Avx2.Permute4x64(vecNext, shuffleMask);
                    var product = Avx2.Multiply(vecBase, vecPermute);
                    accumulator = Avx2.Add(accumulator, product);
                }

                area = accumulator[0] - accumulator[1] + accumulator[2] - accumulator[3];
                noSIMDStartIndex = vec256SpanShifted.Length * chunkSize;
            }

            var iLast = coordinates.Length - 1;
            area += -coordinates[0] * coordinates[iLast] + coordinates[1] * coordinates[iLast - 1];
            for (int i = noSIMDStartIndex; i < coordinates.Length - 3; i += 2)
            {
                area += coordinates[i] * coordinates[i + 3] - coordinates[i + 1] * coordinates[i + 2];
            }
            return area / 2;
        }

        public static void CopyToDouble(ReadOnlySpan<int> source, Span<double> destination)
        {
            if (destination.Length < source.Length)
                throw new ArgumentException("Destination too short.");

            int i = 0;

            if (Avx512F.IsSupported)
            {
                var srcVec = MemoryMarshal.Cast<int, Vector256<int>>(source);
                var dstVec = MemoryMarshal.Cast<double, Vector512<double>>(destination);

                for (; i < srcVec.Length; i++)
                {
                    dstVec[i] = Avx512F.ConvertToVector512Double(srcVec[i]);
                }

                i *= Vector256<int>.Count;
            }
            else if (Avx2.IsSupported)
            {
                var srcVec = MemoryMarshal.Cast<int, Vector128<int>>(source);
                var dstVec = MemoryMarshal.Cast<double, Vector256<double>>(destination);

                for (; i < srcVec.Length; i++)
                {
                    dstVec[i] = Avx.ConvertToVector256Double(srcVec[i]);
                }

                i *= Vector128<int>.Count;
            }

            // remainder / fallback
            for (; i < source.Length; i++)
                destination[i] = source[i];
        }

        public static void Multiply<T>(Span<T> span, T factor) where T : unmanaged, INumber<T>
        {
            int i = 0;
            if (Vector.IsHardwareAccelerated)
            {
                Vector<T> factorVec = new Vector<T>(factor);
                var vecSpan = MemoryMarshal.Cast<T, Vector<T>>(span);
                for (; i < vecSpan.Length; i++)
                {
                    vecSpan[i] *= factorVec;
                }

                i *= Vector<T>.Count;
            }

            // remainder / fallback
            for (; i < span.Length; i++)
                span[i] *= factor;
        }

        public static void Add<T>(Span<T> span1, ReadOnlySpan<T> span2) where T : unmanaged, INumber<T>
        {
            if (span1.Length != span2.Length)
                throw new ArgumentException(nameof(span1),"span lengths not equal");

            int i = 0;
            if (Vector.IsHardwareAccelerated)
            {
                var vecSpan1 = MemoryMarshal.Cast<T, Vector<T>>(span1);
                var vecSpan2 = MemoryMarshal.Cast<T, Vector<T>>(span2);
                for (; i < vecSpan1.Length; i++)
                {
                    vecSpan1[i] += vecSpan2[i];
                }

                i *= Vector<T>.Count;
            }

            // remainder / fallback
            for (; i < span1.Length; i++)
                span1[i] += span2[i];
        }
    }
}
