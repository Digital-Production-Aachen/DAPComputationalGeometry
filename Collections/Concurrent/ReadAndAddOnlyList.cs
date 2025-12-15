using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Threading;

namespace g3
{
    /// <summary>
    /// A specialized lock free ReadOnlyList implementation that supports appending new items to the list by a single writer thread
    /// while multiple reader threads are allowed to read and enumerate the list concurrently.
    /// </summary>
    public class ReadAndAddOnlyList<T> : IReadOnlyList<T>
    {
        private const int DefaultCapacity = 4;

        private volatile T[] _items;
        private volatile int _size;

        public T this[int index]
        {
            get
            {
                // Following trick can reduce the range check by one [copied from List]
                if ((uint)index >= (uint)_size)
                {
                    throw new IndexOutOfRangeException("index must be greater or equal to zero and less than the size of the collection");
                };
                return _items[index];
            }
        }

        public int Count => _size;

        public ReadAndAddOnlyList()
        {
            _items = Array.Empty<T>();
        }

        public ReadAndAddOnlyList(int capacity)
        {
            _items = new T[capacity];
        }

        // Constructs a List, copying the contents of the given collection. The
        // size and capacity of the new list will both be equal to the size of the
        // given collection.
        public ReadAndAddOnlyList(IEnumerable<T> collection)
        {
            if (collection is ICollection<T> c)
            {
                int count = c.Count;
                if (count == 0)
                {
                    _items = Array.Empty<T>();
                }
                else
                {
                    _items = new T[count];
                    c.CopyTo(_items, 0);
                    _size = count;
                }
            }
            else
            {
                _items = Array.Empty<T>();
                using (IEnumerator<T> en = collection!.GetEnumerator())
                {
                    while (en.MoveNext())
                    {
                        Add(en.Current);
                    }
                }
            }
        }

        // Gets and sets the capacity of this list. The capacity is the size of
        // the internal array used to hold items. When set, the internal
        // array of the list is reallocated to the given capacity.
        // Reducing the capacity is not supported.
        public int Capacity
        {
            get => _items.Length;
            set
            {
                if (value > _items.Length)
                {
                    T[] newItems = new T[value];
                    if (_size > 0)
                    {
                        Array.Copy(_items, newItems, _size);
                    }
                    //ensure the integrity of the copy before making it visible by a memory barrier
                    //all old entries up to the old size are copies and have not changed in the new array, so they are valid
                    //regardless of which reference (old or new) other threads are seeing
                    Interlocked.MemoryBarrier();
                    _items = newItems;
                }
            }
        }

        /// <summary>
        /// Get a span of the current items.
        /// Caution: Span length is a snapshot, concurrent adds do not reflect in the span.
        /// Writes on the span might get lost if the backing array is resized concurrently!
        /// Reads are thread safe.
        /// </summary>
        /// <returns></returns>
        public Span<T> AsSpan()
        {
            //snapshot size into local (volatile => acquire memory barrier)
            int size = _size;
            Span<T> span = _items.AsSpan();
            return span.Slice(0, size);
        }

        /// <summary>
        /// Returns an editor of the ReadAndAddOnlyList. Actions by the editor are NOT thread safe
        /// and my only be performed while no concurrent readers use the ReadAndAddOnlyList.
        /// </summary>
        /// <returns></returns>
        public SingleThreadEditor GetSingleThreadEditor()
        {
            return new SingleThreadEditor(this);
        }

        public void Add(T item)
        {
            int newSize = _size + 1;
            if (newSize > _items.Length)
            {
                Grow(newSize);
            }
            //first set the item reference, then employ a Memory Barrier before publishing the increased size
            _items[_size] = item;
            Interlocked.MemoryBarrier();
            _size++;
        }

        private void Grow(int capacity)
        {
            //grow the array
            int newCapacity = _items.Length == 0 ? DefaultCapacity : 2 * _items.Length;
            if (newCapacity < capacity) newCapacity = capacity;
            Capacity = newCapacity;
        }

        public bool Contains(T item)
        {
            return _size != 0 && IndexOf(item) >= 0;
        }

        public int IndexOf(T item)
            => Array.IndexOf(_items, item, 0, _size);

        public int IndexOf(T item, int index)
        {
            return Array.IndexOf(_items, item, index, _size - index);
        }

        public int IndexOf(T item, int index, int count)
        {
            return Array.IndexOf(_items, item, index, count);
        }

        public T[] ToArray()
        {
            //snapshot size
            int size = _size;
            if (size == 0)
            {
                return Array.Empty<T>();
            }

            T[] array = new T[size];
            Array.Copy(_items, array, size);
            return array;
        }

        public IEnumerator<T> GetEnumerator()
        {
            return new ReadAndAddEnumerator<T>(this);
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return new ReadAndAddEnumerator<T>(this);
        }

        private struct ReadAndAddEnumerator<T2> : IEnumerator<T2>
        {
            private ReadAndAddOnlyList<T2> _list;
            private int _index = 0;
            private T2 _current;

            public ReadAndAddEnumerator(ReadAndAddOnlyList<T2> list)
            {
                _list = list;
            }

            public T2 Current => _current!;

            object IEnumerator.Current => _current!;

            public void Dispose()
            {                
            }

            public bool MoveNext()
            {
                int count = _list._size;
                if ((uint)_index < (uint)count)
                {
                    _current = _list[_index];
                    _index++;
                    return true;
                }
                return MoveNextRare(count);
            }

            private bool MoveNextRare(int count)
            {
                _index = count + 1;
                _current = default;
                return false;
            }

            public void Reset()
            {
                _index = 0;
                _current = default;
            }
        }

        public class SingleThreadEditor : IList<T>
        {
            private readonly ReadAndAddOnlyList<T> list;
            private int _size { get=> list._size; set=> list._size = value; }
            private T[] _items => list._items;

            public int Count => list.Count;

            public bool IsReadOnly => false;

            public T this[int index]
            {
                get => list[index];
                set
                {
                    if ((uint)index >= (uint)_size)
                    {
                        throw new IndexOutOfRangeException("index must be greater or equal to zero and less than the size of the collection");
                    }
                    _items[index] = value;
                }
            }

            internal SingleThreadEditor(ReadAndAddOnlyList<T> list)
            {
                this.list = list;
            }

            /// <summary>
            /// // This method removes all items which matches the predicate.
            /// The complexity is O(n).
            /// </summary>
            /// <param name="match"></param>
            /// <returns></returns>
            /// <exception cref="ArgumentNullException"></exception>
            public int RemoveAll(Predicate<T> match)
            {
                if (match == null) throw new ArgumentNullException("match");

                int freeIndex = 0;   // the first free slot in items array

                // Find the first item which needs to be removed.
                while (freeIndex < _size && !match(_items[freeIndex])) freeIndex++;
                if (freeIndex >= _size) return 0;

                int current = freeIndex + 1;
                while (current < _size)
                {
                    // Find the first item which needs to be kept.
                    while (current < _size && match(_items[current])) current++;

                    if (current < _size)
                    {
                        // copy item to the free slot.
                        _items[freeIndex++] = _items[current++];
                    }
                }

                if (RuntimeHelpers.IsReferenceOrContainsReferences<T>())
                {
                    Array.Clear(_items, freeIndex, _size - freeIndex); // Clear the elements so that the gc can reclaim the references.
                }

                int result = _size - freeIndex;
                _size = freeIndex;
                return result;
            }

            /// <summary>
            /// Removes the first occurrence of the given element, if found.
            /// The size of the list is decreased by one if successful.
            /// </summary>
            /// <param name="item"></param>
            /// <returns></returns>
            public bool Remove(T item)
            {
                int index = IndexOf(item);
                if (index >= 0)
                {
                    RemoveAt(index);
                    return true;
                }
                return false;
            }

            /// <summary>
            /// Removes the element at the given index. The size of the list is decreased by one.
            /// </summary>
            /// <param name="index"></param>
            public void RemoveAt(int index)
            {
                if ((uint)index >= (uint)_size)
                {
                    throw new IndexOutOfRangeException();
                }
                _size--;
                if (index < _size)
                {
                    Array.Copy(_items, index + 1, _items, index, _size - index);
                }
                if (RuntimeHelpers.IsReferenceOrContainsReferences<T>())
                {
                    _items[_size] = default;
                }
            }

            /// <summary>
            /// Removes a range of elements from this list.
            /// </summary>
            /// <param name="index"></param>
            /// <param name="count"></param>
            /// <exception cref="ArgumentOutOfRangeException"></exception>
            /// <exception cref="ArgumentException"></exception>
            public void RemoveRange(int index, int count)
            {
                if (index < 0 || count < 0) throw new ArgumentOutOfRangeException();
                if (_size - index < count) throw new ArgumentException();

                if (count > 0)
                {
                    _size -= count;
                    if (index < _size)
                    {
                        Array.Copy(_items, index + count, _items, index, _size - index);
                    }

                    if (RuntimeHelpers.IsReferenceOrContainsReferences<T>())
                    {
                        Array.Clear(_items, _size, count);
                    }
                }
            }

            public void AddRange(IEnumerable<T> collection)
            {
                ArgumentNullException.ThrowIfNull(collection, nameof(collection));

                if (collection is ICollection<T> c)
                {
                    int count = c.Count;
                    if (count > 0)
                    {
                        if (_items.Length - _size < count)
                        {
                            list.Capacity = checked(_size + count);
                        }

                        c.CopyTo(_items, _size);
                        _size += count;
                    }
                }
                else
                {
                    using (IEnumerator<T> en = collection.GetEnumerator())
                    {
                        while (en.MoveNext())
                        {
                            list.Add(en.Current);
                        }
                    }
                }
            }

            /// <summary>
            /// Inserts an element into this list at a given index. The size of the list
            /// is increased by one. If required, the capacity of the list is doubled
            /// before inserting the new element.
            /// </summary>
            /// <param name="index"></param>
            /// <param name="item"></param>
            public void Insert(int index, T item)
            {
                // Note that insertions at the end are legal.
                if ((uint)index > (uint)_size)
                {
                    throw new IndexOutOfRangeException();
                }
                if (_size == _items.Length) list.Grow(_size + 1);
                if (index < _size)
                {
                    Array.Copy(_items, index, _items, index + 1, _size - index);
                }
                _items[index] = item;
                _size++;
            }

            /// <summary>
            /// Inserts the elements of the given collection at a given index. If
            /// required, the capacity of the list is increased to twice the previous
            /// capacity or the new size, whichever is larger.  Ranges may be added
            /// to the end of the list by setting index to the List's size.
            /// </summary>
            /// <param name="index"></param>
            /// <param name="collection"></param>
            /// <exception cref="IndexOutOfRangeException"></exception>
            public void InsertRange(int index, IEnumerable<T> collection)
            {
                ArgumentNullException.ThrowIfNull(collection, nameof(collection));
                if ((uint)index > (uint)_size)
                {
                    throw new IndexOutOfRangeException();
                }

                if (collection is ICollection<T> c)
                {
                    int count = c.Count;
                    if (count > 0)
                    {
                        if (_items.Length - _size < count)
                        {
                            list.Grow(checked(_size + count));
                        }
                        if (index < _size)
                        {
                            Array.Copy(_items, index, _items, index + count, _size - index);
                        }

                        // If we're inserting a List into itself, we want to be able to deal with that.
                        if (this == c)
                        {
                            // Copy first part of _items to insert location
                            Array.Copy(_items, 0, _items, index, index);
                            // Copy last part of _items back to inserted location
                            Array.Copy(_items, index + count, _items, index * 2, _size - index);
                        }
                        else
                        {
                            c.CopyTo(_items, index);
                        }
                        _size += count;
                    }
                }
                else
                {
                    using (IEnumerator<T> en = collection.GetEnumerator())
                    {
                        while (en.MoveNext())
                        {
                            Insert(index++, en.Current);
                        }
                    }
                }
            }

            /// <summary>
            /// Reverses the elements in this list.
            /// </summary>
            public void Reverse() => Reverse(0, _size);

            /// <summary>
            /// Reverses the elements in a range of this list. Following a call to this
            /// method, an element in the range given by index and count
            /// which was previously located at index i will now be located at
            /// index index + (index + count - i - 1).
            /// </summary>
            /// <param name="index"></param>
            /// <param name="count"></param>
            /// <exception cref="ArgumentOutOfRangeException"></exception>
            /// <exception cref="ArgumentException"></exception>
            public void Reverse(int index, int count)
            {
                if (index < 0 || count < 0) throw new ArgumentOutOfRangeException();
                if (_size - index < count) throw new ArgumentException();
                if (count > 1)
                {
                    Array.Reverse(_items, index, count);
                }
            }

            /// <summary>
            /// Copies this List into array, which must be of a compatible array type.
            /// </summary>
            /// <param name="array"></param>
            public void CopyTo(T[] array)
                => CopyTo(array, 0);

            /// <summary>
            /// Copies a section of this list to the given array at the given index.
            /// The method uses the Array.Copy method to copy the elements.
            /// </summary>
            /// <param name="index"></param>
            /// <param name="array"></param>
            /// <param name="arrayIndex"></param>
            /// <param name="count"></param>
            /// <exception cref="ArgumentException"></exception>
            public void CopyTo(int index, T[] array, int arrayIndex, int count)
            {
                if (_size - index < count) throw new ArgumentException();

                // Delegate rest of error checking to Array.Copy.
                Array.Copy(_items, index, array, arrayIndex, count);
            }

            public void CopyTo(T[] array, int arrayIndex)
            {
                // Delegate rest of error checking to Array.Copy.
                Array.Copy(_items, 0, array, arrayIndex, _size);
            }

            /// <summary>
            /// Clears the contents of List.
            /// </summary>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Clear()
            {
                if (RuntimeHelpers.IsReferenceOrContainsReferences<T>())
                {
                    int size = _size;
                    _size = 0;
                    if (size > 0)
                    {
                        Array.Clear(_items, 0, size); // Clear the elements so that the gc can reclaim the references.
                    }
                }
                else
                {
                    _size = 0;
                }
            }

            /// <summary>
            /// Sorts the elements in this list. Uses the default comparer and Array.Sort.
            /// </summary>
            public void Sort()
                => Sort(0, Count, null);

            /// <summary>
            /// Sorts the elements in this list. Uses Array.Sort with the provided comparer.
            /// </summary>
            /// <param name="comparer"></param>
            public void Sort(IComparer<T>? comparer)
                => Sort(0, Count, comparer);

            /// <summary>
            /// Sorts the elements in a section of this list. The sort compares the
            /// elements to each other using the given IComparer interface. If
            /// comparer is null, the elements are compared to each other using
            /// the IComparable interface, which in that case must be implemented by all
            /// elements of the list.
            ///
            /// This method uses the Array.Sort method to sort the elements.
            /// </summary>
            /// <param name="index"></param>
            /// <param name="count"></param>
            /// <param name="comparer"></param>
            /// <exception cref="ArgumentOutOfRangeException"></exception>
            /// <exception cref="ArgumentException"></exception>
            public void Sort(int index, int count, IComparer<T>? comparer)
            {
                if (index < 0) throw new ArgumentOutOfRangeException(nameof(index));
                if (count < 0) throw new ArgumentOutOfRangeException(nameof(count));
                if (_size - index < count) throw new ArgumentException();

                if (count > 1)
                {
                    Array.Sort(_items, index, count, comparer);
                }
            }

            public void Sort(Comparison<T> comparison)
            {
                if (comparison == null)
                {
                    throw new ArgumentNullException(nameof(comparison));
                }

                if (_size > 1)
                {
                    _items.AsSpan().Slice(0, _size).Sort(comparison);
                }
            }

            public int IndexOf(T item) => list.IndexOf(item);
            public void Add(T item) => list.Add(item);
            public bool Contains(T item) => list.Contains(item);
            public IEnumerator<T> GetEnumerator() => list.GetEnumerator();
            IEnumerator IEnumerable.GetEnumerator() => list.GetEnumerator();
        }
    }
}
