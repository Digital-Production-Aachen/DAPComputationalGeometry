using System;
using System.Buffers;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace g3
{
    public ref struct RentedBuffer<T> where T : unmanaged
    {
        public readonly Span<T> Span;
        private byte[] _rentedBuffer;

        public RentedBuffer(int length)
        {
            //always use ArrayPool<byte> and cast to share buffers between all uses
            _rentedBuffer = ArrayPool<byte>.Shared.Rent(length * Unsafe.SizeOf<T>());
            Span = MemoryMarshal.Cast<byte, T>(_rentedBuffer).Slice(0, length);
        }

        public void Return()
        {
            if(_rentedBuffer != null) ArrayPool<byte>.Shared.Return(_rentedBuffer);
        }
    }
}
