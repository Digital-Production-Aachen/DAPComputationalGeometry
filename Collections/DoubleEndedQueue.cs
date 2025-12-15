using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace g3
{
    public class DoubleEndedQueue<T> : ICollection<T>, IReadOnlyList<T>
    {
        private T[] _buffer;
        private int _head = 0; // Index of the first element.
        private int _tail = 0; // Index after the last element.
        private int _count = 0;

        /// <summary>
        /// Gets the number of elements contained in the deque.
        /// </summary>
        public int Count { get => _count; private set => _count = value; }

        /// <summary>
        /// Gets a value indicating whether the deque is read-only.
        /// </summary>
        public bool IsReadOnly => false;

        public T this[int index]
        {
            get
            {
                if ((uint)index >= _count) throw new IndexOutOfRangeException();
                index += _head;
                if (index >= _buffer.Length) index -= _buffer.Length;
                return _buffer[index];
            }
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="DoubleEndedQueue{T}"/> class with the specified capacity.
        /// </summary>
        /// <param name="capacity">The initial capacity of the deque.</param>
        /// <exception cref="ArgumentException">Thrown when capacity is less than 1.</exception>
        public DoubleEndedQueue(int capacity = 16)
        {
            if (capacity < 1)
                throw new ArgumentException("Capacity must be positive.", nameof(capacity));

            _buffer = new T[capacity];
            _head = 0;
            _tail = 0;
            Count = 0;
        }

        #region Deque Specific Operations

        /// <summary>
        /// Adds an item to the front of the deque.
        /// </summary>
        /// <param name="item">The item to add.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddToFront(T item)
        {
            EnsureCapacity();
            // Decrement head index without using modulo:
            _head = _head == 0 ? _buffer.Length - 1 : _head - 1;
            _buffer[_head] = item;
            Count++;
        }

        /// <summary>
        /// Adds an item to the back of the deque.
        /// </summary>
        /// <param name="item">The item to add.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddToBack(T item)
        {
            EnsureCapacity();
            _buffer[_tail] = item;
            int newTail = _tail + 1;
            _tail = (newTail == _buffer.Length ? 0 : newTail);
            Count++;
        }

        /// <summary>
        /// Removes and returns the item at the front of the deque.
        /// </summary>
        /// <returns>The item removed from the front.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public T RemoveFromFront()
        {
            Debug.Assert(Count > 0);
            T item = _buffer[_head];
            if (RuntimeHelpers.IsReferenceOrContainsReferences<T>())
            {
                _buffer[_head] = default;
            }
            int newHead = _head + 1;
            _head = (newHead == _buffer.Length ? 0 : newHead);
            Count--;
            return item;
        }

        /// <summary>
        /// Removes and returns the item at the back of the deque.
        /// </summary>
        /// <returns>The item removed from the back.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public T RemoveFromBack()
        {
            Debug.Assert(Count > 0);
            // Decrement tail index without using modulo:
            _tail = _tail == 0 ? _buffer.Length - 1 : _tail - 1;
            T item = _buffer[_tail];
            if (RuntimeHelpers.IsReferenceOrContainsReferences<T>())
            {
                _buffer[_tail] = default;
            }
            Count--;
            return item;
        }

        /// <summary>
        /// Returns the item at the front of the deque without removing it.
        /// </summary>
        /// <returns>The item at the front.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public T PeekFront()
        {
            Debug.Assert(Count > 0);
            return _buffer[_head];
        }

        /// <summary>
        /// Returns the item at the back of the deque without removing it.
        /// </summary>
        /// <returns>The item at the back.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public T PeekBack()
        {
            Debug.Assert(Count > 0);
            // Compute index of last element without using modulo:
            int index = _tail == 0 ? _buffer.Length - 1 : _tail - 1;
            return _buffer[index];
        }

        /// <summary>
        /// Returns the second item from the front of the deque without removing it.
        /// </summary>
        /// <returns>The second item from the front.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public T PeekSecondFront()
        {
            Debug.Assert(Count > 1);
            int index = _head + 1;
            index = index == _buffer.Length ? 0 : index;
            return _buffer[index];
        }

        /// <summary>
        /// Returns the second item from the back of the deque without removing it.
        /// </summary>
        /// <returns>The second item from the back.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public T PeekSecondBack()
        {
            Debug.Assert(Count > 1);
            // Calculate the index two positions before tail.
            int index = _tail - 2;
            index = index < 0 ? index + _buffer.Length : index;
            return _buffer[index];
        }

        /// <summary>
        /// Ensures that the internal buffer has sufficient capacity to store additional elements.
        /// </summary>
        private void EnsureCapacity()
        {
            if (Count == _buffer.Length)
            {
                int newCapacity = _buffer.Length * 2;
                T[] newBuffer = new T[newCapacity];

                // If the items are contiguous in the array, copy them in one go.
                if (_head < _tail)
                {
                    Array.Copy(_buffer, _head, newBuffer, 0, Count);
                }
                else
                {
                    int countHead = _buffer.Length - _head;
                    Array.Copy(_buffer, _head, newBuffer, 0, countHead);
                    Array.Copy(_buffer, 0, newBuffer, countHead, _tail);
                }

                _buffer = newBuffer;
                _head = 0;
                _tail = Count;
            }
        }

        #endregion

        #region ICollection<T> Members

        /// <summary>
        /// Adds an item to the collection (to the back of the deque).
        /// </summary>
        /// <param name="item">The object to add to the collection.</param>
        public void Add(T item)
        {
            AddToBack(item);
        }

        /// <summary>
        /// Removes all items from the deque.
        /// </summary>
        public void Clear()
        {
            _head = 0;
            _tail = 0;
            Count = 0;

            // Clear the array to remove references.
            if (!RuntimeHelpers.IsReferenceOrContainsReferences<T>()) { return; }
            if (_head < _tail)
            {
                Array.Clear(_buffer, _head, Count);
            }
            else
            {
                Array.Clear(_buffer, _head, _buffer.Length - _head);
                Array.Clear(_buffer, 0, _tail);
            }
        }

        /// <summary>
        /// Determines whether the deque contains a specific value.
        /// </summary>
        /// <param name="item">The object to locate in the deque.</param>
        /// <returns>true if the item is found in the deque; otherwise, false.</returns>
        public bool Contains(T item)
        {
            var comparer = EqualityComparer<T>.Default;
            for (int i = 0; i < Count; i++)
            {
                int index = _head + i;
                if (index >= _buffer.Length)
                    index -= _buffer.Length;

                if (comparer.Equals(_buffer[index], item))
                    return true;
            }
            return false;
        }

        /// <summary>
        /// Copies the elements of the deque to an array, starting at a particular array index.
        /// </summary>
        /// <param name="array">The one-dimensional array that is the destination of the elements copied from the deque.</param>
        /// <param name="arrayIndex">The zero-based index in array at which copying begins.</param>
        /// <exception cref="ArgumentNullException">Thrown when the destination array is null.</exception>
        /// <exception cref="ArgumentOutOfRangeException">Thrown when the arrayIndex is less than 0 or greater than the length of the array.</exception>
        /// <exception cref="ArgumentException">
        /// Thrown when there is insufficient space from arrayIndex to the end of the destination array.
        /// </exception>
        public void CopyTo(T[] array, int arrayIndex)
        {
            if (array == null)
                throw new ArgumentNullException(nameof(array));

            if (arrayIndex < 0 || arrayIndex > array.Length)
                throw new ArgumentOutOfRangeException(nameof(arrayIndex));

            if (array.Length - arrayIndex < Count)
                throw new ArgumentException("The number of elements in the source deque is greater than the available space in the destination array.");

            for (int i = 0; i < Count; i++)
            {
                int index = _head + i;
                if (index >= _buffer.Length)
                    index -= _buffer.Length;
                array[arrayIndex + i] = _buffer[index];
            }
        }

        /// <summary>
        /// Removes the first occurrence of a specific object from the deque.
        /// </summary>
        /// <param name="item">The object to remove from the deque.</param>
        /// <returns>true if item was successfully removed from the deque; otherwise, false.</returns>
        public bool Remove(T item)
        {
            var comparer = EqualityComparer<T>.Default;
            for (int i = 0; i < Count; i++)
            {
                int index = _head + i;
                if (index >= _buffer.Length)
                    index -= _buffer.Length;

                if (comparer.Equals(_buffer[index], item))
                {
                    // Decide whether shifting elements toward the head or tail is more efficient.
                    if (i < Count - i - 1)
                    {
                        // Shift elements from head to index forward.
                        for (int j = i; j > 0; j--)
                        {
                            int fromIndex = _head + j - 1;
                            if (fromIndex >= _buffer.Length)
                                fromIndex -= _buffer.Length;
                            int toIndex = _head + j;
                            if (toIndex >= _buffer.Length)
                                toIndex -= _buffer.Length;
                            _buffer[toIndex] = _buffer[fromIndex];
                        }
                        _buffer[_head] = default;
                        // Increment head using conditional logic.
                        int newHead = _head + 1;
                        _head = (newHead == _buffer.Length ? 0 : newHead);
                    }
                    else
                    {
                        // Shift elements from index+1 to tail backward.
                        for (int j = i; j < Count - 1; j++)
                        {
                            int fromIndex = _head + j + 1;
                            if (fromIndex >= _buffer.Length)
                                fromIndex -= _buffer.Length;
                            int toIndex = _head + j;
                            if (toIndex >= _buffer.Length)
                                toIndex -= _buffer.Length;
                            _buffer[toIndex] = _buffer[fromIndex];
                        }
                        // Decrement tail using conditional logic.
                        _tail = _tail == 0 ? _buffer.Length - 1 : _tail - 1;
                        _buffer[_tail] = default;
                    }
                    Count--;
                    return true;
                }
            }
            return false;
        }

        #endregion

        #region IEnumerable<T> Members

        /// <summary>
        /// Returns an enumerator that iterates through the deque.
        /// </summary>
        /// <returns>An enumerator for the deque.</returns>
        public IEnumerator<T> GetEnumerator()
        {
            for (int i = 0; i < Count; i++)
            {
                yield return this[i];
            }
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }

        #endregion
    }
}
