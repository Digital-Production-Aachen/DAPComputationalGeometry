using System;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace g3
{
    /// <summary>
    /// Concurrent collection, all methods thread safe. Uses a lock on write access.
    /// Keeps top (largest) values with a fixed capacity of the top ranks, e.g. Top 10
    /// When larger values are added, values no longer in the Top Ranks are discarded.
    /// Can be read concurrently while multiple threads add values.
    /// Only guaranties the presence of values (not the order) is correct when read concurrently with write operations,
    /// except for if a snapshot is performed (snapshot uses a lock).
    /// </summary>
    /// <typeparam name="T"></typeparam>
    public class ConcurrentTopRankArray<T> : IReadOnlyList<T> where T : IComparable<T>
    {
        private T[] _values;
        private int _size;

        public int Count => _size;
        public int Capacity => _values.Length;
        public int FreeSlots => Capacity - Count;
        public bool IsFull => Count == Capacity;

        public T this[int index] => _values[index];

        public ConcurrentTopRankArray(int capacity, T defaultValue)
        {
            _values = new T[capacity];
            Array.Fill(_values, defaultValue);
            _size = 0;
        }

        public T TryAdd(T value)
        {
            T curMin = _values[^1];
            if (curMin.CompareTo(value) >= 0)
            {
                return curMin;
            }

            lock (_values)
            {
                int i;
                for (i = _size - 1; i >= 0; i--)
                {
                    T occupant = _values[i];
                    int next = i + 1;
                    if (occupant.CompareTo(value) >= 0)
                    {
                        if (next < Capacity)
                        { 
                            _values[next] = value;
                            if (_size < Capacity) _size++;
                        }
                        break;
                    }
                    else
                    {
                        if (next < Capacity) { _values[next] = occupant; }                        
                    }
                }

                if (i == -1)
                {
                    _values[0] = value;
                    if (_size < Capacity) _size++;
                }
            }

            return _values[_size - 1];
        }

        /// <summary>
        /// Locks the collection and takes a safe snapshot. Order is guarantied correct.
        /// </summary>
        /// <returns></returns>
        public T[] Snapshot()
        {
            //we don't know if size will have changed before we acquire the lock, deal with that later
            T[] buffer = new T[_values.Length];
            int size;
            lock (_values)
            {
                size = _size;
                Array.Copy(_values, buffer, _size);
            }
            if(size < buffer.Length)
            {
                T[] bufferRightSize = new T[size];
                Array.Copy(buffer, bufferRightSize, size);
                return bufferRightSize;
            }
            else
            {
                return buffer;
            }
        }

        public IEnumerator<T> GetEnumerator()
        {
            ArraySegment<T> segment = new ArraySegment<T>(_values, 0, Count);
            return segment.GetEnumerator();
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            ArraySegment<T> segment = new ArraySegment<T>(_values, 0, Count);
            return segment.GetEnumerator();
        }
    }
}
