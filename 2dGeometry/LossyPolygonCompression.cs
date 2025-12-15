using Clipper2Lib;
using g3;
using g3.Proto;
using Google.Protobuf;
using Google.Protobuf.Collections;
using OpenVectorFormat.Utils;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;
using System.Threading;

namespace g3.Compression
{
    public readonly ref struct OutBufferView
    {
        public readonly ReadOnlySpan<byte> ByteBuffer;

        public OutBufferView(ReadOnlySpan<byte> byteBuffer)
        {
            this.ByteBuffer = byteBuffer;
        }

        public int Blocks32Written => (ByteBuffer.Length + 3) / 4;
        public int ByteLength => ByteBuffer.Length;
        public static implicit operator ReadOnlySpan<byte>(OutBufferView v) => v.ByteBuffer;
        public static OutBufferView Empty => new OutBufferView(Span<byte>.Empty);
    }

    public static class LossyPolygonCompression
    {
        private const int scaleDenominator = 32;
        public static bool EnableDeltaCompression = true;

        internal static Polygon2d[] ToPolygon2Ds(this IReadOnlyCollection<PackedPolygon> packedPolys)
        {
            return packedPolys.SelectToArray(x => x.ToPolygon2d());
        }

        internal static Polygon2d ToPolygon2d(this PackedPolygon polygon)
        {
            Polygon2d poly2d;
            if (polygon.DeltaCoords.Count > 0)
            {
                var coords = polygon.DeltaCoords;
                if (coords.Count % 2 != 1) { throw new ArgumentException("decode error: invalid delta coordinate count"); }
                int count = (polygon.DeltaCoords.Count - 1) / 2;
                poly2d = SharedPolyPool.Rent(count);
                var verts = poly2d.VerticesAsSpanWithCount(count);
                int j = 0;

                // first int is the scale denominator
                double scale = (double)scaleDenominator / coords[0];
                Vector2d vert = Vector2d.Zero;
                for (int i = 1; i < coords.Count; i += 2)
                {
                    vert.x += coords[i] * scale;
                    vert.y += coords[i + 1] * scale;
                    verts[j++] = vert;
                }
            }
            else if (polygon.FloatCoords.Count > 0)
            {
                var coords = polygon.FloatCoords;
                if (coords.Count % 2 != 0) { throw new ArgumentException("decode error: invalid float coordinate count"); }
                int count = coords.Count / 2;
                poly2d = SharedPolyPool.Rent(count);
                var verts = poly2d.VerticesAsSpanWithCount(count);
                int j = 0;

                Vector2d vert = Vector2d.Zero;
                for (int i = 0; i < coords.Count; i += 2)
                {
                    vert.x = coords[i];
                    vert.y = coords[i + 1];
                    verts[j++] = vert;
                }
            }
            else return SharedPolyPool.Rent(0);// empty polygon

            return poly2d;
        }

        /// <summary>
        /// Returns the minimum buffer size that is required to guarantee successful compression of the input.
        /// Actual compression likely uses less space.
        /// </summary>
        /// <param name="polygons"></param>
        /// <returns></returns>
        public static int EncodeByteBufferSize(IEnumerable<PolyWithProps> polygons)
        {
            int sum = 0;
            foreach (var polygon in polygons)
            {
                sum += 5;//header worst case, 35bit = 5*7(varint) 3 flags + 32bit size
                sum += 4 * 4;//4 props
                sum += polygon.polygon.VertexCount * 8;//worst case: float encoding
            }
            return sum;
        }

        public static byte[] ToByteArray(this IEnumerable<PolyWithProps> polygons, float maxError)
        {
            int bSize = EncodeByteBufferSize(polygons);
            if (bSize < 1024)
            {
                Span<byte> buffer = stackalloc byte[bSize];
                var res = EncodeIntoBuffer(polygons, buffer, maxError);
                return res.ByteBuffer.ToArray();
            }
            else
            {
                var buffer = System.Buffers.ArrayPool<byte>.Shared.Rent(bSize);
                var res = EncodeIntoBuffer(polygons, buffer, maxError);
                var array = res.ByteBuffer.ToArray();
                System.Buffers.ArrayPool<byte>.Shared.Return(buffer);
                return array;
            }
        }

        public static OutBufferView EncodeIntoBuffer(this IEnumerable<PolyWithProps> polygons, Span<byte> buffer, float maxError)
        {
            if (!polygons.Any()) return OutBufferView.Empty;
            //notice ToPackedPolygons cannot be performed in the per poly loop below because it performs a topology check of the polygon collection after rounding coordinates
            PackedPolygons packedP = polygons.Select(p => p.polygon).ToPackedPolygons(maxError);
            var polyPropEnum = polygons.GetEnumerator();
            int outPos = 0;
            for (int i = 0; i < packedP.Polygons.Count; i++)
            {
                PackedPolygon polygon = packedP.Polygons[i];
                polyPropEnum.MoveNext();
                PolyWithProps inputP = polyPropEnum.Current;
                bool isFloat = polygon.DeltaCoords.Count == 0;
                bool hasUpProps = inputP.UpProps.Item1 != inputP.UpProps.Item2 || inputP.UpProps.Item1 != 0;
                bool hasDownProps = inputP.DownProps.Item1 != inputP.DownProps.Item2 || inputP.DownProps.Item1 != 0;
                if (!isFloat)
                {
                    //scale is at idx 0 already
                    //ZigZag encode, but only the coordinates, props are unsigned anyways
                    uint vertexCount;
                    {
                        var coordSpan = polygon.DeltaCoords.AsSpan().Slice(1);
                        vertexCount = (uint)coordSpan.Length / 2;
                        ZigZagEncoding.Encode(coordSpan);
                    }
                    if (hasDownProps)
                    {
                        polygon.DeltaCoords.Insert(1, (int)inputP.DownProps.Item2);
                        polygon.DeltaCoords.Insert(1, (int)inputP.DownProps.Item1);
                    }
                    if (hasUpProps)
                    {
                        polygon.DeltaCoords.Insert(1, (int)inputP.UpProps.Item2);
                        polygon.DeltaCoords.Insert(1, (int)inputP.UpProps.Item1);
                    }
                    //estimate the header size, in rare cases we estimate wrong and have to memmove the data
                    //(because we don't know exactly how many bytes will be written beforehand, and that MIGHT change the header varint size)
                    int headerSizeEst = PolygonHeader.EstimateEncodedByteSize(polygon.DeltaCoords.Count);
                    int vertexCountSize = VarIntEncoding.EncodedByteSize(vertexCount);
                    //we can cast unsigned to signed because we ZigZag encoded before
                    var integerSpan = MemoryMarshal.Cast<int, uint>(polygon.DeltaCoords.AsSpan());
                    var output = buffer.Slice(outPos + headerSizeEst + vertexCountSize);

                    OutBufferView result = Simple16Encoding.Compress(integerSpan, output);
                    PolygonHeader polygonHeader = new PolygonHeader(isFloat, hasUpProps, hasDownProps, result.ByteLength);
                    int realHeaderSize = VarIntEncoding.EncodedByteSize(polygonHeader.PackedData);
                    if (realHeaderSize != headerSizeEst)
                    {
                        //estimate failed. memmove the compressed output to the corrected offset
                        result.ByteBuffer.CopyTo(buffer.Slice(outPos + realHeaderSize + vertexCountSize));
                    }
                    int headerBytesWritten = VarIntEncoding.WriteVarint32(buffer.Slice(outPos), polygonHeader.PackedData);
                    Debug.Assert(headerBytesWritten == realHeaderSize);
                    outPos += headerBytesWritten;
                    int vertexCountBytesWritten = VarIntEncoding.WriteVarint32(buffer.Slice(outPos), vertexCount);
                    outPos += vertexCountBytesWritten + result.ByteLength;
                }
                else
                {
                    int vertexCount = polygon.FloatCoords.Count / 2;
                    if (hasUpProps)
                    {
                        VarIntEncoding.EncodedByteSize(inputP.UpProps.Item1);
                        VarIntEncoding.EncodedByteSize(inputP.UpProps.Item2);
                    }
                    if (hasDownProps)
                    {
                        VarIntEncoding.EncodedByteSize(inputP.DownProps.Item1);
                        VarIntEncoding.EncodedByteSize(inputP.DownProps.Item2);
                    }
                    PolygonHeader polygonHeader = new PolygonHeader(isFloat, hasUpProps, hasDownProps, vertexCount);
                    int headerSize = VarIntEncoding.EncodedByteSize(polygonHeader.PackedData);

                    outPos += VarIntEncoding.WriteVarint32(buffer.Slice(outPos), polygonHeader.PackedData);
                    if (hasUpProps)
                    {
                        outPos += VarIntEncoding.WriteVarint32(buffer.Slice(outPos), inputP.UpProps.Item1);
                        outPos += VarIntEncoding.WriteVarint32(buffer.Slice(outPos), inputP.UpProps.Item2);
                    }
                    if (hasDownProps)
                    {
                        outPos += VarIntEncoding.WriteVarint32(buffer.Slice(outPos), inputP.DownProps.Item1);
                        outPos += VarIntEncoding.WriteVarint32(buffer.Slice(outPos), inputP.DownProps.Item2);
                    }
                    var floatBytes = MemoryMarshal.Cast<float, byte>(polygon.FloatCoords.AsSpan());
                    floatBytes.CopyTo(buffer.Slice(outPos));
                    outPos += floatBytes.Length;
                }
            }

#if DEBUG//in debug unpack right away, to detect problems instantly and have the input that caused it on the stack
            UnpackPolygonsFromBuffer(buffer.Slice(0, outPos));
#endif
            return new OutBufferView(buffer.Slice(0, outPos));
        }

        public static PackedPolygons ToPackedPolygons(this IEnumerable<Polygon2d> polygons, float maxError)
        {
            var packedPolys = new PackedPolygons();
            int count = polygons.Count();
            if (count == 0) return packedPolys;
            packedPolys.Polygons.Capacity = count;
            List<Polygon2d> topologyCheck = new(count);
            foreach (var polygon in polygons)
            {
                packedPolys.Polygons.Add(ToPackedPolygon(polygon, maxError, out Polygon2d repackedPoly));
                topologyCheck.Add(repackedPoly);
            }

            if (topologyCheck.HaveIntersections(useThreadLocalClipper: true))
            {
                //compression changed the poly assembly topology, fallback to float
                packedPolys.Polygons.Clear();
                foreach (var poly in polygons)
                {
                    packedPolys.Polygons.Add(poly.ToFloatPoly());
                }
                //if rounding to float also changes the topology, we cannot represent the input in sufficient precision in compressed form
                //this must be caught upstream of this algorithm, or compression could change the polygon count which is very unexpected behavior
                Debug.Assert(!packedPolys.Polygons.ToPolygon2Ds().HaveIntersections());
            }

            return packedPolys;
        }

        public static PackedPolygon ToPackedPolygon(Polygon2d polygon, float maxError)
           => ToPackedPolygon(polygon, maxError, out _);

        private static PackedPolygon ToPackedPolygon(Polygon2d polygon, float maxError, out Polygon2d reunpackedPoly)
        {
            bool asPacked = false;
            reunpackedPoly = polygon;
            PackedPolygon packedPolygon = null;
            try
            {
                if (maxError > 1.0E-5 && EnableDeltaCompression)
                {
                    MinMaxManhattenEdgeLength(polygon, out double min, out double max, maxError);
                    if (max > min)
                    {
                        // min steps is set to 64 for 1 byte varint encoding, less cannot save us any bytes
                        int steps = Math.Max(Convert.ToInt32(Math.Ceiling(max / min)), 16);

                        // choose resolution scale based on the data and max error
                        // if resolution is insufficient, double it
                        packedPolygon = new PackedPolygon();
                        for (; steps <= 1 << 25; steps = (int)(steps * 1.5))
                        {
                            bool success = FillPackedPoly(polygon, packedPolygon.DeltaCoords, maxError, max, steps - 1);
                            if (!success) { continue; }
                            //the compression might introduce self intersections if part of the polygon dimensions are in range of the rounding errors
                            //the only way to detect this is to unpack the polygon again and use clipper2 to see if the result has self intersections
                            //this test ensures we do not change the topology of the polygon
                            reunpackedPoly = packedPolygon.ToPolygon2d();
                            success = !reunpackedPoly.HasSelfIntersections(useThreadLocalClipper: true);
                            if (!success) { continue; }
                            //ensure compression does not change poly orientation
                            success = polygon.IsHole == reunpackedPoly.IsHole;
                            if (!success) { continue; }
                            else { asPacked = true; break; }
                        }
                    }
                }
            }
            catch (OverflowException e) { ILogging.Logger?.Warning(e, "delta compression integer overflow, falling back to float"); asPacked = false; }
            catch (Exception e) { ILogging.Logger?.Error(e, "error in delta compression, falling back to float"); asPacked = false; }

            if (asPacked)
            {
                int size = packedPolygon.CalculateSize();
                int floatSize = CodedOutputStream.ComputeFloatSize(0.0f) * polygon.VertexCount * 2
                    + CodedOutputStream.ComputeLengthSize(polygon.VertexCount * 2)
                    + CodedOutputStream.ComputeTagSize(PackedPolygon.FloatCoordsFieldNumber);
                if (size < floatSize)
                {
                    return packedPolygon;
                }
            }

            //fallback: do not pack
            return polygon.ToFloatPoly();
        }

        private static bool FillPackedPoly(Polygon2d polygon, IList<int> deltaCoords, float maxError, double max, int steps)
        {
            deltaCoords.Clear();
            //deltaCoords.Capacity = polygon.VertexCount * 2 + 1;
            double scale = max / steps;
            int scaleNumerator = Convert.ToInt32(Math.Ceiling(scaleDenominator / scale));
            double roundedScale = scaleNumerator / (double)scaleDenominator;
            deltaCoords.Add(scaleNumerator);
            Vector2d curPosRounded = Vector2d.Zero;
            foreach (Vector2d vertex in polygon.Vertices)
            {
                var diff = vertex - curPosRounded;
                Vector2i diffScaled = new Vector2i(
                    Convert.ToInt32(diff.x * roundedScale),
                    Convert.ToInt32(diff.y * roundedScale));
                // if we produce coincident points, we may change the poly topology
                if (diffScaled == Vector2i.Zero) { return false; }
                curPosRounded.x += diffScaled.x / roundedScale;
                curPosRounded.y += diffScaled.y / roundedScale;
                var errorSqr = (vertex - curPosRounded).LengthSquared;
                if (errorSqr > maxError * maxError) { return false; }

                deltaCoords.Add(diffScaled.x);
                deltaCoords.Add(diffScaled.y);
            }
            return true;
        }

        private static PackedPolygon ToFloatPoly(this Polygon2d polygon)
        {
            var floatPoly = new PackedPolygon();
            floatPoly.FloatCoords.Capacity = polygon.VertexCount * 2;
            foreach (var vert in polygon.Vertices)
            {
                floatPoly.FloatCoords.Add((float)vert.x);
                floatPoly.FloatCoords.Add((float)vert.y);
            }
            return floatPoly;
        }

        private static void EnsureCapacity<T>(this RepeatedField<T> repField, int minCapacity)
        {
            if (repField.Capacity < minCapacity)
            {
                repField.Capacity = (int)BitOperations.RoundUpToPowerOf2((uint)minCapacity);
            }
        }

        private static Polygon2d ToPolygon2d(this ReadOnlySpan<float> coords)
        {
            if (coords.Length % 2 != 0) { throw new ArgumentException("decode error: invalid float coordinate count"); }
            int count = coords.Length / 2;
            Polygon2d poly2d = SharedPolyPool.Rent(count);
            var verts = poly2d.VerticesAsSpanWithCount(count);
            int j = 0;

            Vector2d vert = Vector2d.Zero;
            for (int i = 0; i < coords.Length; i += 2)
            {
                vert.x = coords[i];
                vert.y = coords[i + 1];
                verts[j++] = vert;
            }
            return poly2d;
        }

        private static PolyWithProps UnpackPolyWithProps(Span<uint> input, in PolygonHeader header, int propsHeaderSize)
        {
            PolyWithProps result = new();
            PropsHeader propsHeader = new();
            Span<uint> propsHeaderAsSpan = MemoryMarshal.Cast<PropsHeader, uint>(MemoryMarshal.CreateSpan(ref propsHeader, 1));
            input[..propsHeaderSize].CopyTo(propsHeaderAsSpan);
            //props header has the expected layout in order
            //only exception case is if upProps are missing but downProps are there
            if (header.HasDownProps && !header.HasUpProps)
            {
                result.DownProps = propsHeader.UpProps;
            }
            else
            {
                result.UpProps = propsHeader.UpProps;
                result.DownProps = propsHeader.DownProps;
            }

            Span<int> intCoords = MemoryMarshal.Cast<uint, int>(input[propsHeaderSize..]);
            int vertexCount = intCoords.Length / 2;
            Polygon2d poly2d = SharedPolyPool.Rent(vertexCount);
            result.polygon = poly2d;
            var verts = poly2d.VerticesAsSpanWithCount(vertexCount);
            Span<double> doubleCoords = MemoryMarshal.Cast<Vector2d, double>(verts);
            double scale = (double)scaleDenominator / propsHeader.ScaleDenominator;

            ZigZagEncoding.Decode(intCoords);  
            SIMDVectorOperations.CopyToDouble(intCoords, doubleCoords);
            SIMDVectorOperations.Multiply(doubleCoords, scale);

            Span<Vector128<double>> coordsVec128 = MemoryMarshal.Cast<double, Vector128<double>>(doubleCoords);
            Vector128<double> delta = coordsVec128[0];
            for (int i = 1; i < coordsVec128.Length; i++)
            {
                delta += coordsVec128[i];
                coordsVec128[i] = delta;
            }
            return result;
        }

        private static void MinMaxManhattenEdgeLength(Polygon2d poly, out double min, out double max, float tolerance)
        {
            min = double.MaxValue;
            max = double.MinValue;
            Vector2d prev = Vector2d.Zero;
            foreach (var vert in poly.VerticesAsReadOnlySpan)
            {
                MinMax(vert.x - prev.x, ref min, ref max, tolerance);
                MinMax(vert.y - prev.y, ref min, ref max, tolerance);
                prev = vert;
            }
        }

        private static void MinMax(double num, ref double min, ref double max, float tolerance)
        {
            double diff = Math.Abs(num);
            if (diff > tolerance)
            {
                min = min < diff ? min : diff;
                max = max > diff ? max : diff;
            }
        }

        private readonly struct PolygonHeader
        {
            private readonly uint _bitData;

            // Masks (flags in lowest bits, ByteLength in upper bits)
            private const uint IsFloatMask = 1u << 0;
            private const uint HasUpPropsMask = 1u << 1;
            private const uint HasDownPropsMask = 1u << 2;
            private const int ByteLengthShift = 3;
            private const uint ByteLengthMask = 0x1FFFFFFFu << ByteLengthShift; // 29 bits

            public bool IsFloat => (_bitData & IsFloatMask) != 0;
            public bool HasUpProps => (_bitData & HasUpPropsMask) != 0;
            public bool HasDownProps => (_bitData & HasDownPropsMask) != 0;
            public int ByteLength => (int)(_bitData >> ByteLengthShift);
            public uint PackedData => _bitData;

            public static int EstimateEncodedByteSize(int numInts)
            {
                int byteSizeEstimate = numInts * 4;
                //estimated typical compression is compress by 70%
                uint bytes = Convert.ToUInt32(byteSizeEstimate * 0.3);
                return VarIntEncoding.EncodedByteSize(bytes << ByteLengthShift);
            }

            public PolygonHeader(bool isFloat, bool hasUpProps, bool hasDownProps, int byteLength)
            {
                if (byteLength < 0 || byteLength > (int)(ByteLengthMask >> ByteLengthShift))
                    throw new ArgumentOutOfRangeException(nameof(byteLength), $"ByteLength must be between 0 and {(ByteLengthMask >> ByteLengthShift)}");

                _bitData = ((uint)byteLength << ByteLengthShift)
                        | (isFloat ? IsFloatMask : 0)
                        | (hasUpProps ? HasUpPropsMask : 0)
                        | (hasDownProps ? HasDownPropsMask : 0);
            }

            public PolygonHeader(uint bitData)
            {
                _bitData = bitData;
            }
        }

        //memory layout for decoding poly properties from Span<uint>
        [StructLayout(LayoutKind.Sequential, Pack = 32)]
        private struct PropsHeader
        {
            public uint ScaleDenominator;
            private uint upProps1;
            private uint upProps2;
            private uint downProps1;
            private uint downProps2;
            public (uint, uint) UpProps => (upProps1, upProps2);
            public (uint, uint) DownProps => (downProps1, downProps2);
        }

        public class PolyWithProps
        {
            public Polygon2d polygon;
            public (uint, uint) UpProps;
            public (uint, uint) DownProps;
        }

        /// <summary>
        /// Unpacks compressed polygons from byte data buffer.
        /// </summary>
        /// <param name="data"></param>
        /// <returns></returns>
        public static List<PolyWithProps> UnpackPolygonsFromBuffer(ReadOnlySpan<byte> data)
        {
            List<PolyWithProps> output = new();
            int parsePos = 0;
            //reserve a fixed size 2kb buffer on the stack, if this is not enough we have to rent buffers from pool
            Span<uint> stackBuffer = stackalloc uint[512];

            while (parsePos < data.Length)
            {
                uint headerBitData = VarIntEncoding.ParseVarint32(data, ref parsePos);
                var header = new PolygonHeader(headerBitData);
                if (header.IsFloat)
                {
                    PolyWithProps nextPoly = new();
                    if (header.HasUpProps)
                    {
                        nextPoly.UpProps.Item1 = VarIntEncoding.ParseVarint32(data, ref parsePos);
                        nextPoly.UpProps.Item2 = VarIntEncoding.ParseVarint32(data, ref parsePos);
                    }
                    if (header.HasDownProps)
                    {
                        nextPoly.DownProps.Item1 = VarIntEncoding.ParseVarint32(data, ref parsePos);
                        nextPoly.DownProps.Item2 = VarIntEncoding.ParseVarint32(data, ref parsePos);
                    }
                    int floatBytes = header.ByteLength * 4 * 2;//for float ByteLength is vertex count
                    var floatSpan = MemoryMarshal.Cast<byte, float>(data.Slice(parsePos, floatBytes));
                    nextPoly.polygon = floatSpan.ToPolygon2d();
                    parsePos += floatBytes;
                    output.Add(nextPoly);
                }
                else
                {
                    int vertexCount = (int)VarIntEncoding.ParseVarint32(data, ref parsePos);
                    var simple16Bytes = data.Slice(parsePos, header.ByteLength);
                    parsePos += simple16Bytes.Length;

                    int propsHeaderSize = 1 + Convert.ToInt32(header.HasUpProps) * 2 + Convert.ToInt32(header.HasDownProps) * 2;
                    int expected = propsHeaderSize + vertexCount * 2;
                    int expectedPadded = expected + 8;//Vec256 stores need 7 padding max
                    PolyWithProps nextPoly;
                    if (expectedPadded <= stackBuffer.Length)
                    {
                        stackBuffer.Slice(expected, 8).Clear();//clear the padding span
                        int decompressed = Simple16Encoding.Decompress(simple16Bytes, stackBuffer);         
                        nextPoly = UnpackPolyWithProps(stackBuffer[..expected], header, propsHeaderSize);
                    }
                    else
                    {
                        //casting lets us share the pool with encode
                        RentedBuffer<uint> buffer = new(expectedPadded);
                        Span<uint> uintSpan = buffer.Span;
                        uintSpan.Slice(expected, 8).Clear();//clear the padding span
                        int decompressed = Simple16Encoding.Decompress(simple16Bytes, uintSpan);
                        nextPoly = UnpackPolyWithProps(uintSpan[..expected], header, propsHeaderSize);
                        buffer.Return();
                    }
                    output.Add(nextPoly);
                }
            }
            return output;
        }

        private static (int start, int end) ExtractPropsHeader(Span<uint> vec256Buffer, ref PropsHeader propsHeader, in PolygonHeader header, in ReadOnlySpan<uint> input, ref int blockIdx)
        {
            int headerReadCnt = 0;
            int numOut = 0, toCopy = 0;
            int expected = 1;
            if (header.HasUpProps) expected += 2;
            if (header.HasDownProps) expected += 2;
            Span<uint> propsHeaderAsSpan = MemoryMarshal.Cast<PropsHeader, uint>(MemoryMarshal.CreateSpan(ref propsHeader, 1));
            propsHeaderAsSpan.Clear();

            while (blockIdx < input.Length)
            {
                numOut = Simple16Encoding.DecompressBlock(input[blockIdx++], vec256Buffer);
                toCopy = Math.Min(numOut, expected - headerReadCnt);
                vec256Buffer.Slice(0, toCopy).CopyTo(propsHeaderAsSpan.Slice(headerReadCnt));
                headerReadCnt += toCopy;
                if (headerReadCnt == expected) break;
            }

            if (headerReadCnt < expected)
            {
                throw new ApplicationException("parts of the header are in last padded block. Assumed that is impossible");
            }

            //props header has the expected layout in order
            //only exception case is if upProps are missing but downProps are there
            if (header.HasDownProps && !header.HasUpProps)
            {
                var upProps = propsHeaderAsSpan.Slice(1, 2);
                var downProps = propsHeaderAsSpan.Slice(3, 2);
                upProps.CopyTo(downProps);
                upProps.Clear();
            }

            return (toCopy, numOut);
        }
    }

    public static class VarIntEncoding
    {
        /// <summary>
        /// Writes a 32 bit value as a varint. Returns bytes written to buffer.
        /// </summary>
        public static int WriteVarint32(Span<byte> buffer, uint value)
        {
            int pos = 0;
            // Optimize for the common case of a single byte value
            if (value < 128 && buffer.Length >= 1)
            {
                buffer[pos++] = (byte)value;
                return pos;
            }

            while (pos < buffer.Length)
            {
                if (value > 127)
                {
                    buffer[pos++] = (byte)((value & 0x7F) | 0x80);
                    value >>= 7;
                }
                else
                {
                    buffer[pos++] = (byte)value;
                    break;
                }
            }
            if (value > 127) throw new ArgumentException("insufficient buffer capacity", nameof(buffer));
            return pos;
        }

        /// <summary>
        /// Parses a raw Varint from buffer. Advances bufferPos ref.
        /// </summary>
        public static uint ParseVarint32(ReadOnlySpan<byte> buffer, ref int bufferPos)
        {
            uint value = 0;
            int shift = 0;
            while (shift < 32 && bufferPos < buffer.Length)
            {
                uint b = buffer[bufferPos++];
                value |= (b & 0x7F) << shift;
                if ((b & 0x80) == 0) return value;
                shift += 7;
            }
            throw new FormatException("varint format invalid");
        }

        /// <summary>
        /// Computes the size of the input in bytes when encoded as Base128 varint
        /// </summary>
        /// <param name="input">integer to encode</param>
        /// <returns>size in bytes when the input is encoded as varint</returns>
        public static int EncodedByteSize(uint input)
        {
            int size = 1;
            while (input >= 0x80)
            {
                size++;
                input >>= 7;
            }
            return size;
        }
    }

    public static class MortonEncoding
    {
        /// <summary>
        /// Packs two non‑negative integers into a single ulong by interleaving their bits:
        /// bit 0 of a → bit 0 of result, bit 0 of b → bit 1 of result,
        /// bit 1 of a → bit 2 of result, bit 1 of b → bit 3 of result, etc.
        /// </summary>
        public static ulong Pack(uint a, uint b)
        {
            ulong c = 0;
            int shift = 0;
            // continue while either has remaining bits
            while (a != 0 || b != 0)
            {
                // interleave a’s lowest bit at pos 2*shift
                c |= (ulong)(a & 1) << (2 * shift);
                // interleave b’s lowest bit at pos 2*shift+1
                c |= (ulong)(b & 1) << (2 * shift + 1);
                a >>= 1;
                b >>= 1;
                shift++;
            }
            return c;
        }

        /// <summary>
        /// Unpacks a Morton‑interleaved ulong back into (a,b).
        /// </summary>
        public static (uint a, uint b) Unpack(ulong c)
        {
            uint a = 0;
            uint b = 0;
            int shift = 0;
            // continue until c is exhausted
            while (c != 0)
            {
                // extract bit 2*shift  → goes to a’s bit[shift]
                a |= (uint)((c >> (2 * shift)) & 1) << shift;
                // extract bit 2*shift+1 → goes to b’s bit[shift]
                b |= (uint)((c >> (2 * shift + 1)) & 1) << shift;
                // clear the two bits we just consumed
                c &= ~(1UL << (2 * shift));
                c &= ~(1UL << (2 * shift + 1));
                shift++;
            }
            return (a, b);
        }
    }

    public static class Simple16Encoding
    {
        /// <summary>
        /// Compress uints from input into output buffer blocks.
        /// Returns a view of the output buffer with passing bytes trimmed.
        /// </summary>
        /// <param name="input"></param>
        /// <param name="output"></param>
        /// <returns>view of the output buffer as byte span, with padding zeros sliced</returns>
        public static OutBufferView Compress(ReadOnlySpan<uint> input, Span<byte> output)
        {
            return Compress(input, MemoryMarshal.Cast<byte, uint>(output));
        }

        /// <summary>
        /// Compress uints from input into output buffer blocks.
        /// Returns a view of the output buffer with passing bytes trimmed.
        /// </summary>
        /// <param name="input"></param>
        /// <param name="output"></param>
        /// <returns>view of the output buffer as byte span, with padding zeros sliced</returns>
        public static OutBufferView Compress(ReadOnlySpan<uint> input, Span<uint> output)
        {
            int curOut = 0;
            int count = 0;
            while (input.Length > count)
            {
                input = input.Slice(count);
                (uint block, count) = CompressBlock(input);
                output[curOut++] = block;
            }

            ReadOnlySpan<byte> byteBuffer = MemoryMarshal.Cast<uint, byte>(output);
            curOut *= 4;
            while (curOut > 0 && byteBuffer[curOut - 1] == 0)
                curOut--;
            return new OutBufferView(byteBuffer.Slice(0, curOut));
        }

        /// <summary>
        /// Decompress blocks from input byte buffer into output buffer.
        /// </summary>
        /// <param name="input"></param>
        /// <param name="output"></param>
        /// <returns>the number of decompressed uints, including padding zeros</returns>
        public static int Decompress(ReadOnlySpan<byte> input, Span<uint> output)
        {
            ReadOnlySpan<uint> inputNoPadding = MemoryMarshal.Cast<byte, uint>(input);
            int total = Decompress(inputNoPadding, output);
            int remainder = input.Length & 3;
            if (remainder == 0) { return total; }

            // add padding to decompress the last block
            Span<byte> padded = stackalloc byte[4];
            padded.Clear();
            input[^remainder..].CopyTo(padded);
            uint lastBlock = MemoryMarshal.Read<uint>(padded);
            return total + DecompressBlock(lastBlock, output.Slice(total));
        }

        /// <summary>
        /// Decompress blocks from input into output buffer.
        /// </summary>
        /// <param name="input"></param>
        /// <param name="output"></param>
        /// <returns>the number of decompressed uints, including padding zeros</returns>
        public static int Decompress(ReadOnlySpan<uint> input, Span<uint> output)
        {
            int total = 0;
            int i = 0;

            for (; i < input.Length - 7 && output.Length >= 64; i += 8)
            {
                for (int j = 0; j < 8; j++)
                {
                    int numOut = DecompressBlock(input[i + j], output);
                    output = output.Slice(numOut);
                    total += numOut;
                }
            }

            for (; i < input.Length; i++)
            {
                int numOut = DecompressBlock(input[i], output);
                output = output.Slice(numOut);
                total += numOut;
            }
            return total;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int DecompressBlock(uint block, Span<uint> output)
        {
            uint selector = block & SEL_MASK;
            if (Avx2.IsSupported)
            {
                Vector256<uint> vPayload = Vector256.Create(block);
                (Vector256<uint> shifts, Vector256<uint> masks) = ShiftsAndMasks[selector];
                Vector256<uint> shifted = Avx2.ShiftRightLogicalVariable(vPayload, shifts);
                Vector256<uint> masked = Avx2.And(shifted, masks);
                StoreVectorToSpan(masked, output);
                return ModeCounts[selector];
            }
            else
            {
                ReadOnlySpan<byte> bits = GetBits(selector);
                int bitOffset = TAG_BITS;
                int numInts = bits.Length;

                for (int i = 0; i < numInts; i++)
                {
                    int b = bits[i];
                    output[i] = (block >> bitOffset) & ((1u << b) - 1);
                    bitOffset += b;
                }

                return numInts;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void StoreVectorToSpan(Vector256<uint> vec, Span<uint> outSpan)
        {
            var vSpan = MemoryMarshal.Cast<uint, Vector256<uint>>(outSpan);
            vSpan[0] = vec;
        }

        public static (uint block, int count) CompressBlock(ReadOnlySpan<uint> input)
        {
            for (uint tblIdx = TableStartIdx(input[0]); tblIdx < 16; tblIdx++)
            {
                ReadOnlySpan<byte> bits = GetBits(tblIdx);

                int numEnc = bits.Length;
                uint block = tblIdx;
                int usedBits = TAG_BITS;
                int j = 0;

                for (j = 0; j < numEnc; j++)
                {
                    int b = bits[j];
                    uint limit = 1u << b;
                    //pad input with zeros if at end of stream
                    uint val = j >= input.Length ? 0 : input[j];
                    if (val >= limit)
                        break;

                    block |= val << usedBits;
                    usedBits += b;
                }

                if (j == numEnc)
                {
                    return (block, numEnc);
                }
            }

            throw new ArgumentOutOfRangeException("input value exceeds Simple16 codec range of 2^28");
        }

        // Per-selector single-block tables (16 entries)
        private static readonly byte[] ModeCounts = CalcModeCounts();
        private static readonly byte[][] BitWidths = CalcBitWidths();
        private static readonly (Vector256<uint> shift, Vector256<uint> mask)[] ShiftsAndMasks = MakeShiftsAndMasks();

        private static byte[] CalcModeCounts()
        {
            byte[] modeCounts = new byte[MODE_COUNT];
            for (int i = 0; i < MODE_COUNT; i++) modeCounts[i] = (byte)GetBits((uint)i).Length;
            return modeCounts;
        }

        private static byte[][] CalcBitWidths()
        {
            byte[][] modeCounts = new byte[MODE_COUNT][];
            for (uint i = 0; i < MODE_COUNT; i++)
            {
                modeCounts[i] = GetBits(i).ToArray();
            }
            return modeCounts;
        }

        private static (Vector256<uint> shift, Vector256<uint> mask)[] MakeShiftsAndMasks()
        {
            var table = new (Vector256<uint> shift, Vector256<uint> mask)[MODE_COUNT];
            Span<uint> shifts = stackalloc uint[8];
            Span<uint> masks = stackalloc uint[8];
            // build per-selector single-block vectors
            for (uint sel = 0; sel < MODE_COUNT; sel++)
            {
                var widths = GetBits(sel);
                uint running = 0;
                for (int i = 0; i < 8; i++)
                {
                    if (i < widths.Length)
                    {
                        int w = widths[i];
                        shifts[i] = TAG_BITS + running;
                        masks[i] = w >= 32 ? uint.MaxValue : ((1u << w) - 1u);
                        running += (uint)w;
                    }
                    else
                    {
                        shifts[i] = 0;
                        masks[i] = 0u;  // mask zero makes lane zero
                    }
                }
                table[(int)sel] = (shifts.ToVec256(), masks.ToVec256());
            }
            return table;
        }

        private static Vector256<uint> ToVec256(this Span<uint> span) => MemoryMarshal.Cast<uint, Vector256<uint>>(span)[0];

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static ReadOnlySpan<byte> GetBits(uint idx) => idx switch
        {
            // codec table that defines count and bit width of encoded integers for each selector
            0 => [4, 4, 4, 4, 4, 4, 4],
            1 => [6, 6, 6, 5, 5],
            2 => [7, 7, 7, 7],
            3 => [9, 1, 9, 9],
            4 => [9, 9, 1, 9],
            5 => [9, 9, 9, 1],
            6 => [10, 9, 9],
            7 => [11, 11, 6],
            8 => [11, 6, 11],
            9 => [6, 11, 11],
            10 => [12, 8, 8],
            11 => [8, 12, 8],
            12 => [8, 8, 12],
            13 => [14, 14],
            14 => [18, 10],
            15 => [28],
            _ => throw new ArgumentOutOfRangeException(nameof(idx))
        };
        const int TAG_BITS = 4;
        const int MODE_COUNT = 1 << TAG_BITS;
        const uint SEL_MASK = (1u << TAG_BITS) - 1;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static uint TableStartIdx(uint input)
        {
            return input switch
            {
                < (1u << 4) => 0,
                < (1u << 6) => 1,
                < (1u << 7) => 2,
                < (1u << 9) => 3,
                < (1u << 10) => 6,
                < (1u << 11) => 7,
                < (1u << 12) => 10,
                < (1u << 14) => 13,
                < (1u << 18) => 14,
                _ => 15,
            };
        }

        /// <summary>
        /// The maximum number of elements encoded into a single 32bit block.
        /// Denominator of the maximum compression ratio.
        /// Gives a safe upper bound for buffer sizes.
        /// </summary>
        public static readonly int MaxCountPerBlock = GetBits(0).Length;
    }

    //this is WIP, must be tested like stable Simple16 above
    internal static class Simple64Encoding
    {
        /// <summary>
        /// Compress unsigned integers from input into output buffer blocks.
        /// Returns a view of the output buffer with passing bytes trimmed.
        /// </summary>
        /// <param name="input"></param>
        /// <param name="output"></param>
        /// <returns>view of the output buffer as byte span, with padding zeros sliced</returns>
        public static OutBufferView Compress<T>(ReadOnlySpan<T> input, Span<ulong> output) where T : IBinaryInteger<T>
        {
            int curOut = 0;
            int count = 0;
            while (input.Length > count)
            {
                input = input.Slice(count);
                (ulong block, count) = CompressBlock(input);
                output[curOut++] = block;
            }

            ReadOnlySpan<byte> byteBuffer = MemoryMarshal.Cast<ulong, byte>(output);
            curOut *= 8;
            while (curOut > 0 && byteBuffer[curOut - 1] == 0)
                curOut--;
            return new OutBufferView(byteBuffer.Slice(0, curOut));
        }

        public readonly ref struct OutBufferView
        {
            public readonly ReadOnlySpan<byte> ByteBuffer;

            public OutBufferView(ReadOnlySpan<byte> byteBuffer)
            {
                this.ByteBuffer = byteBuffer;
            }

            public int Blocks64Written => (ByteBuffer.Length + 7) / 8;
            public int ByteLength => ByteBuffer.Length;
            public static implicit operator ReadOnlySpan<byte>(OutBufferView v) => v.ByteBuffer;
        }

        /// <summary>
        /// Decompress blocks from input byte buffer into output buffer.
        /// </summary>
        /// <param name="input"></param>
        /// <param name="output"></param>
        /// <returns>the number of decompressed uints, including padding zeros</returns>
        //public static int Decompress(ReadOnlySpan<byte> input, Span<ulong> output)
        //{
        //    ReadOnlySpan<ulong> inputNoPadding = MemoryMarshal.Cast<byte, ulong>(input);
        //    int total = Decompress(inputNoPadding, output);
        //    int remainder = input.Length & 3;
        //    if (remainder == 0) { return total; }

        //    // add padding to decompress the last block
        //    Span<byte> padded = stackalloc byte[8];
        //    padded.Clear();
        //    input[^remainder..].CopyTo(padded);
        //    uint lastBlock = MemoryMarshal.Read<ulong>(padded);
        //    return total + DecompressBlock(lastBlock, output.Slice(total));
        //}

        /// <summary>
        /// Decompress blocks from input into output buffer.
        /// </summary>
        /// <param name="input"></param>
        /// <param name="output"></param>
        /// <returns>the number of decompressed uints, including padding zeros</returns>
        //public static int Decompress(ReadOnlySpan<ulong> input, Span<ulong> output)
        //{
        //    int total = 0;
        //    int i = 0;

        //    for (; i < input.Length - 7 && output.Length >= 64; i += 8)
        //    {
        //        for (int j = 0; j < 8; j++)
        //        {
        //            int numOut = DecompressBlock(input[i + j], output);
        //            output = output.Slice(numOut);
        //            total += numOut;
        //        }
        //    }

        //    for (; i < input.Length; i++)
        //    {
        //        int numOut = DecompressBlock(input[i], output);
        //        output = output.Slice(numOut);
        //        total += numOut;
        //    }
        //    return total;
        //}

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public static int DecompressBlock(ulong block, Span<ulong> output)
        //{
        //    ulong selector = block & SEL_MASK;
        //    if (Avx2.IsSupported)
        //    {
        //        Vector256<ulong> vPayload = Vector256.Create(block);
        //        (Vector256<ulong> shifts, Vector256<ulong> masks) = ShiftsAndMasks[selector];
        //        Vector256<ulong> shifted = Avx2.ShiftRightLogicalVariable(vPayload, shifts);
        //        Vector256<ulong> masked = Avx2.And(shifted, masks);
        //        StoreVectorToSpan(masked, output);
        //        return ModeCounts[selector] & 0b00000111;
        //    }
        //    else
        //    {
        //        ReadOnlySpan<byte> bits = GetBits(selector);
        //        int bitOffset = TAG_BITS;
        //        int numInts = bits.Length;

        //        for (int i = 0; i < numInts; i++)
        //        {
        //            int b = bits[i];
        //            output[i] = (block >> bitOffset) & ((1u << b) - 1);
        //            bitOffset += b;
        //        }

        //        return numInts;
        //    }
        //}

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void StoreVectorToSpan(Vector256<uint> vec, Span<uint> outSpan)
        {
            var vSpan = MemoryMarshal.Cast<uint, Vector256<uint>>(outSpan);
            vSpan[0] = vec;
        }

        public static (ulong block, int count) CompressBlock<T>(ReadOnlySpan<T> input) where T : IBinaryInteger<T>
        {
            for (uint tblIdx = TableStartIdx(ulong.CreateChecked(input[0])); tblIdx < MODE_COUNT; tblIdx++)
            {
                ReadOnlySpan<byte> bits = BitWidths[tblIdx];

                int numEnc = bits.Length;
                ulong block = tblIdx;
                int usedBits = TAG_BITS;
                int j = 0;

                for (j = 0; j < numEnc; j++)
                {
                    int b = bits[j];
                    ulong limit = 1u << b;
                    //pad input with zeros if at end of stream
                    ulong val = j >= input.Length ? 0 : ulong.CreateChecked(input[j]);
                    if (val >= limit)
                        break;

                    block |= val << usedBits;
                    usedBits += b;
                }

                if (j == numEnc)
                {
                    SelectorCounts[tblIdx]++;
                    return (block, numEnc);
                }
            }

            throw new OverflowException("input value exceeds Simple64 codec range, max is 2^58");
        }

        public static ulong[] SelectorCounts = new ulong[MODE_COUNT];

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static uint TableStartIdx(ulong input)
        {
            //we need log2 ceiling to encode => + 1
            int log2c = BitOperations.Log2(input) + 1;
            return encodeStartIdcs[log2c];
        }

        // Per-selector single-block tables (16 entries)
        private static readonly byte[][] BitWidths = [
            //10 or more values
            [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
            [2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2],
            [4,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3],
            //14 or less values, fits Vector512
            [5,5,4,4,4,4,4,4,4,4,4,4,4,4],
            [6,6,6,5,5,5,5,5,5,5,5],//4
            [6,6,6,6,6,6,6,6,5,5],
            
            //9 values
            [7,7,7,7,6,6,6,6,6],
            [7,2,7,7,7,7,7,7,7],
            [7,7,2,7,7,7,7,7,7],//8
            [7,7,7,2,7,7,7,7,7],
            [7,7,7,7,2,7,7,7,7],
            [7,7,7,7,7,2,7,7,7],
            [7,7,7,7,7,7,2,7,7],//12
            [7,7,7,7,7,7,7,2,7],
            [7,7,7,7,7,7,7,7,2],

            //8 values
            [8,8,7,7,7,7,7,7],
            [8,2,8,8,8,8,8,8],//16
            [8,8,2,8,8,8,8,8],
            [8,8,8,2,8,8,8,8],
            [8,8,8,8,2,8,8,8],
            [8,8,8,8,8,2,8,8],//20
            [8,8,8,8,8,8,2,8],
            [8,8,8,8,8,8,8,2],

            [9,7,7,7,7,7,7,7],
            [7,9,7,7,7,7,7,7],//24
            [7,7,9,7,7,7,7,7],
            [7,7,7,9,7,7,7,7],
            [7,7,7,7,9,7,7,7],
            [7,7,7,7,7,9,7,7],//28
            [7,7,7,7,7,7,9,7],
            [7,7,7,7,7,7,7,9],

            //7 values
            [9,9,8,8,8,8,8],
            [9,4,9,9,9,9,9],//32
            [9,9,4,9,9,9,9],
            [9,9,9,4,9,9,9],
            [9,9,9,9,4,9,9],
            [9,9,9,9,9,4,9],//36
            [9,9,9,9,9,9,4],

            [10,8,8,8,8,8,8],
            [8,10,8,8,8,8,8],
            [8,8,10,8,8,8,8],//40
            [8,8,8,10,8,8,8],
            [8,8,8,8,10,8,8],
            [8,8,8,8,8,10,8],
            [8,8,8,8,8,8,10],//44
            [16,7,7,7,7,7,7],

            //6 values
            [10,10,10,10, 9, 9],
            [13, 9, 9, 9, 9, 9],
            [11, 3,11,11,11,11],//48
            [11,11, 3,11,11,11],
            [11,11,11, 3,11,11],
            [11,11,11,11, 3,11],
            [11,11,11,11,11, 3],//52
            
            //5 values
            [12,12,12,11,11],
            [14,11,11,11,11],

            //4 or less values
            [15,15,14,14],
            [17, 7,17,17],//56
            [17,17, 7,17],
            [17,17,17, 7],
            [20,19,19],
            [23,23,12],//60
            [23,12,23],
            [29,29],
            [58],//63
            ];
        private static readonly byte[] encodeStartIdcs;
        private static readonly (Vector256<uint> shift, Vector256<uint> mask)[] ShiftsAndMasks = MakeShiftsAndMasks();

        static Simple64Encoding()
        {
            encodeStartIdcs = new byte[MODE_COUNT];
            for (byte i = 0; i < MODE_COUNT; i++)
            {
                int idx = BitWidths.IndexOfFirst(w => w[0] >= i);
                encodeStartIdcs[i] = (byte)(idx != -1 ? idx : BitWidths.Length - 1);
            }
            //MakeShiftsAndMasks();
        }

        private static (Vector256<uint> shift, Vector256<uint> mask)[] MakeShiftsAndMasks()
        {
            var table = new (Vector256<uint> shift, Vector256<uint> mask)[MODE_COUNT];
            Span<uint> shifts = stackalloc uint[8];
            Span<uint> masks = stackalloc uint[8];
            // build per-selector single-block vectors
            for (uint sel = 0; sel < MODE_COUNT; sel++)
            {
                var widths = BitWidths[sel];
                uint running = 0;
                for (int i = 0; i < 8; i++)
                {
                    if (i < widths.Length)
                    {
                        int w = widths[i];
                        shifts[i] = TAG_BITS + running;
                        masks[i] = w >= 32 ? uint.MaxValue : ((1u << w) - 1u);
                        running += (uint)w;
                    }
                    else
                    {
                        shifts[i] = 0;
                        masks[i] = 0u;  // mask zero makes lane zero
                    }
                }
                table[(int)sel] = (shifts.ToVec256(), masks.ToVec256());
            }
            return table;
        }

        private static Vector256<uint> ToVec256(this Span<uint> span) => MemoryMarshal.Cast<uint, Vector256<uint>>(span)[0];

        const int TAG_BITS = 6;
        const int PAYLOAD_BITS = 64 - TAG_BITS;
        const int MODE_COUNT = 1 << TAG_BITS;
        const ulong SEL_MASK = (1u << TAG_BITS) - 1;

        /// <summary>
        /// The maximum number of elements encoded into a single 32bit block.
        /// Denominator of the maximum compression ratio.
        /// Gives a safe upper bound for buffer sizes.
        /// </summary>
        public static readonly int MaxCountPerBlock = BitWidths[0].Length;
    }

    public static class ZigZagEncoding
    {
        public const int SimdThreshold = 12;

        public static void Encode(Span<int> data)
        {
            int offset = 0;

            if (Vector.IsHardwareAccelerated && data.Length >= SimdThreshold)
            {
                Span<Vector<int>> vectors = MemoryMarshal.Cast<int, Vector<int>>(data);

                for (int i = 0; i < vectors.Length; i++)
                {
                    Vector<int> v = vectors[i];
                    vectors[i] = (v << 1) ^ (v >> 31);
                }

                offset = vectors.Length * Vector<int>.Count;
            }

            for (int i = offset; i < data.Length; i++)
            {
                int v = data[i];
                data[i] = (v << 1) ^ (v >> 31);
            }
        }

        public static void Decode(Span<int> data)
        {
            int offset = 0;

            if (Vector.IsHardwareAccelerated && data.Length >= SimdThreshold)
            {
                Span<Vector<int>> vectors = MemoryMarshal.Cast<int, Vector<int>>(data);
                Vector<int> one = Vector<int>.One;

                for (int i = 0; i < vectors.Length; i++)
                {
                    Vector<int> v = vectors[i];
                    vectors[i] = (v >> 1) ^ -(v & one);
                }

                offset = vectors.Length * Vector<int>.Count;
            }

            for (int i = offset; i < data.Length; i++)
            {
                int v = data[i];
                data[i] = (v >> 1) ^ -(v & 1);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector<int> EncodeVector(ref Vector<int> vector)
        {
            return (vector << 1) ^ (vector >> 31);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector256<int> EncodeVector(Vector256<int> vector)
        {
            return (vector << 1) ^ (vector >> 31);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector512<int> EncodeVector(Vector512<int> vector)
        {
            return (vector << 1) ^ (vector >> 31);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector<int> DecodeVector(ref Vector<int> vector)
        {
            Vector<int> one = Vector<int>.One;
            return (vector >> 1) ^ -(vector & one);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector256<int> DecodeVector(Vector256<int> vector)
        {
            Vector256<int> one = Vector256<int>.One;
            return (vector >> 1) ^ -(vector & one);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector512<int> DecodeVector(ref Vector512<int> vector)
        {
            Vector512<int> one = Vector512<int>.One;
            return (vector >> 1) ^ -(vector & one);
        }
    }
}
