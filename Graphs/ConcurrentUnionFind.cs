using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Threading;

namespace g3
{
    public class ConcurrentUnionFind
    {
        private readonly ReadAndAddOnlyList<int> _parents;
        private readonly ConcurrentDictionary<int, int> _sizes;
        public IEnumerable<int> Roots => _sizes.Keys;

        public int Capacity
        {
            get => _parents.Capacity;
            set
            {
                lock (_parents)
                {
                    _parents.Capacity = value;
                }
            }
        }

        public int Count => _parents.Count;
        public int RootCount => Roots.Count();

        public ConcurrentUnionFind(int capacity = 0)
        {
            _parents = new(capacity);
            _sizes = new();
        }

        /// <summary>
        /// Union the two sets a and b belong to
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns>the new root, or -1 if a and b already belong to the same connected component</returns>
        public int Union(int a, int b)
        {
            int ra = FindRoot(a);
            int rb = FindRoot(b);
            if (ra == rb) return -1;

            // Single‐writer only on parents
            lock (_parents)
            {
                //update roots while holding the lock
                ra = FindRoot(a);
                rb = FindRoot(b);
                if (ra == rb) return -1;
                Span<int> parents = _parents.AsSpan();
                // simple union‐by‐size
                if (_sizes[ra] <= _sizes[rb])
                {
                    Volatile.Write(ref parents[ra], rb);
                    _sizes[rb] += _sizes[ra];
                    _sizes.Remove(ra, out _);
                    return rb;
                }
                else
                {
                    Volatile.Write(ref parents[rb], ra);
                    _sizes[ra] += _sizes[rb];
                    _sizes.Remove(rb, out _);
                    return ra;
                }
            }
        }

        public int AppendNew()
        {
            int added;
            lock (_parents)
            {
                added = _parents.Count;
                _parents.Add(added);
                _sizes.TryAdd(added, 1);
            }
            return added;
        }

        /// <summary>
        /// Find the root of the connected component of node
        /// </summary>
        /// <param name="node"></param>
        /// <returns></returns>
        public int FindRoot(int node)
        {
            // 1) find root: chase parent pointers until a node is its own parent
            int orig = node;
            int root = node;
            int counter = 0;
            while (true)
            {
                int p = _parents[root];
                if (p == root)
                    break;
                root = p;
                if (counter++ == _parents.Count)
                    throw new InvalidOperationException("union find state corrupted, tree contains a cycle");
            }

            // 2) try to acquire the lock and compress path back to root
            // if the lock is contested, just return result without compression
            if (!Monitor.TryEnter(_parents))
                return root;

            try
            {
                // check if root is still up to date, if not recurse while holding the lock
                if (_parents[root] != root)
                    return FindRoot(root);
                Span<int> span = _parents.AsSpan();
                while (true)
                {
                    int p = span[orig];
                    if (p == root) break;
                    span[orig] = root;
                    orig = p;
                }
            }
            finally { Monitor.Exit(_parents); }
            return root;
        }

        /// <summary>
        /// Get the size of the connected component for a given root
        /// </summary>
        /// <param name="root"></param>
        /// <returns></returns>
        public int RootComponentSize(int root) => _sizes[root];

        /// <summary>
        /// Get the size of the connected component for any node
        /// </summary>
        /// <param name="node"></param>
        /// <returns></returns>
        public int ComponentSize(int node) => _sizes[FindRoot(node)];

        public bool TryGetComponentSize(int node, out int size)
        {
            if (_parents.Count <= (uint)node)
            {
                size = -1;
                return false;
            }
            int root = FindRoot(node);
            return _sizes.TryGetValue(root, out size);
        }
    }
}
