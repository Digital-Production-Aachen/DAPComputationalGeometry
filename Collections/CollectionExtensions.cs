// Ignore Spelling: Memoize

using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.InteropServices;

namespace g3
{
    public static class CollectionExtensions
    {
        public static SumType Sum<ItemType, SumType>(this IList<ItemType> source, Func<ItemType, SumType> selector) where SumType : INumber<SumType>
        {
            SumType sum = SumType.Zero;
            for (int i = 0; i < source.Count; i++)
            {
                sum += selector(source[i]);
            }
            return sum;
        }

        public static TResult[] SelectToArray<T, TResult>(this IReadOnlyCollection<T> source, Func<T, TResult> selector)
        {
            if(source.Count == 0) return Array.Empty<TResult>();
            TResult[] array = new TResult[source.Count];
            int i = 0;
            foreach (T item in source)
            {
                array[i++] = selector(item);
            }
            return array;
        }

        public static List<TResult> SelectToList<T, TResult>(this IReadOnlyCollection<T> source, Func<T, TResult> selector)
        {
            List<TResult> list = new List<TResult>(source.Count);
            foreach(T item in source)
            {
                list.Add(selector(item));
            }
            return list;
        }

        /// <summary>
        /// Implementation of LINQ select for lists, supporting lazy evaluated additional read only index based access as well as the ICollection interface.
        /// Count and CopyTo will automagically be used by framework methods like ToList(), and reduce allocations if they know the count ahead of time.
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <typeparam name="TResult"></typeparam>
        /// <param name="source"></param>
        /// <param name="selector"></param>
        /// <returns></returns>
        public static IReadOnlyList<TResult> Select<T, TResult>(this IReadOnlyList<T> source, Func<T, TResult> selector)
        {
            return new ReadOnlyListSelector<T, TResult>(source, selector);
        }

        public static IReadOnlyList<TResult> Cast<TFrom, TResult>(this IReadOnlyList<TFrom> source)
        {
            return new ReadOnlyListSelector<TFrom, TResult>(source, tIn => (TResult)(object)tIn);
        }

        private readonly struct ReadOnlyListSelector<T, TResult> : IReadOnlyList<TResult>, ICollection<TResult>
        {
            private readonly IReadOnlyList<T> source;
            private readonly Func<T, TResult> selector;

            public ReadOnlyListSelector(IReadOnlyList<T> source, Func<T, TResult> selector)
            {
                this.source = source;
                this.selector = selector;
            }

            public TResult this[int index] => selector(source[index]);

            public int Count => source.Count;

            public bool IsReadOnly => true;

            public void Add(TResult item)
            {
                throw new NotSupportedException("Select is read only");
            }

            public void Clear()
            {
                throw new NotSupportedException("Select is read only");
            }

            public bool Remove(TResult item)
            {
                throw new NotSupportedException("Select is read only");
            }

            public bool Contains(TResult item)
            {
                foreach(var sourceItem in source)
                {
                    if(selector(sourceItem).Equals(item)) return true;
                }
                return false;
            }

            public void CopyTo(TResult[] array, int arrayIndex)
            {
                int count = source.Count;
                for (int i = 0; i < count; i++)
                {
                    array[i + arrayIndex] = selector(source[i]);
                }
            }

            public IEnumerator<TResult> GetEnumerator()
            {
                return new IListSelectorIterator<T, TResult>(this);
            }

            IEnumerator IEnumerable.GetEnumerator()
            {
                return new IListSelectorIterator<T, TResult>(this);
            }
        }

        private struct IListSelectorIterator<T, T2> : IEnumerator<T2>
        {
            readonly ReadOnlyListSelector<T, T2> _list;
            private int _index = 0;
            private T2 _current;

            public IListSelectorIterator(ReadOnlyListSelector<T, T2> list)
            {
                _list = list;
            }

            public T2 Current => _current;

            object IEnumerator.Current => _current;

            public void Dispose()
            {
            }

            public bool MoveNext()
            {
                int count = _list.Count;
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

        /// <summary>
        /// Implementation of LINQ select for collection, supporting lazy evaluated additional read only as well as the ICollection interface.
        /// Count and CopyTo will automagically be used by framework methods like ToList(), and reduce allocations if they know the count ahead of time.
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <typeparam name="TResult"></typeparam>
        /// <param name="source"></param>
        /// <param name="selector"></param>
        /// <returns></returns>
        public static IReadOnlyCollection<TResult> Select<T, TResult>(this IReadOnlyCollection<T> source, Func<T, TResult> selector)
        {
            return new ReadOnlyCollectionSelector<T, TResult>(source, selector);
        }

        private readonly struct ReadOnlyCollectionSelector<T, TResult> : IReadOnlyCollection<TResult>, ICollection<TResult>
        {
            private readonly IReadOnlyCollection<T> source;
            private readonly Func<T, TResult> selector;

            public ReadOnlyCollectionSelector(IReadOnlyCollection<T> source, Func<T, TResult> selector)
            {
                this.source = source;
                this.selector = selector;
            }

            public int Count => source.Count;

            public bool IsReadOnly => true;

            public void Add(TResult item) => throw new NotSupportedException("Select is read only");

            public void Clear() => throw new NotSupportedException("Select is read only");

            public bool Remove(TResult item) => throw new NotSupportedException("Select is read only");

            public bool Contains(TResult item)
            {
                foreach (var sourceItem in source)
                {
                    if (selector(sourceItem).Equals(item)) return true;
                }
                return false;
            }

            public void CopyTo(TResult[] array, int arrayIndex)
            {
                int i = 0;
                foreach (var sourceItem in source)
                {
                    array[arrayIndex + i++] = selector(sourceItem);
                }
            }

            public IEnumerator<TResult> GetEnumerator() => new ReadOnlyCollectionSelectorIterator<T, TResult>(source, selector);
            IEnumerator IEnumerable.GetEnumerator() => new ReadOnlyCollectionSelectorIterator<T, TResult>(source, selector);
        }

        private struct ReadOnlyCollectionSelectorIterator<T, T2> : IEnumerator<T2>
        {
            readonly private IEnumerator<T> _sourceEnum;
            private readonly Func<T, T2> selector;

            public ReadOnlyCollectionSelectorIterator(IReadOnlyCollection<T> source, Func<T, T2> selector)
            {
                _sourceEnum = source.GetEnumerator();
                this.selector = selector;
            }
            public T2 Current => selector(_sourceEnum.Current);
            object IEnumerator.Current => Current;
            public void Dispose() => _sourceEnum.Dispose();
            public bool MoveNext() => _sourceEnum.MoveNext();
            public void Reset() => _sourceEnum.Reset();
        }

        /// <summary>
        /// Lazily caches the elements of the source sequence as they are
        /// iterated, so that subsequent enumerations do not re-enumerate
        /// the original sequence.
        /// </summary>
        /// <typeparam name="T">The type of the elements of the source sequence.</typeparam>
        /// <param name="source">The sequence to cache lazily.</param>
        /// <returns>
        /// An <see cref="IEnumerable{T}"/> that caches elements as they are
        /// iterated, and replays the cached elements on subsequent enumerations.
        /// </returns>
        /// <exception cref="ArgumentNullException">
        /// Thrown when <paramref name="source"/> is <c>null</c>.
        /// </exception>
        public static IEnumerable<T> Memoize<T>(this IEnumerable<T> source)
        {
            if (source is null) throw new ArgumentNullException(nameof(source));
            return MemoizeIterator(source);
        }

        private static IEnumerable<T> MemoizeIterator<T>(IEnumerable<T> source)
        {
            List<T> cache = new();
            using var enumerator = source.GetEnumerator();
            bool hasMore = true;

            for (int i = 0; ; i++)
            {
                if (i < cache.Count)
                {
                    yield return cache[i];
                }
                else if (hasMore)
                {
                    if (enumerator.MoveNext())
                    {
                        cache.Add(enumerator.Current);
                        yield return enumerator.Current;
                    }
                    else
                    {
                        hasMore = false;
                        yield break;
                    }
                }
                else
                {
                    yield break;
                }
            }
        }

        public static List<T> TakeToList<T>(this List<T> source, int count)
        {
            count = int.Clamp(count, 0, source.Count);
            return CollectionsMarshal.AsSpan(source).Slice(0, count).ToList();
        }

        public static List<T> TakeToList<T>(this T[] source, int count)
        {
            count = int.Clamp(count, 0, source.Length);
            return source.AsSpan().Slice(0, count).ToList<T>();
        }

        public static List<T> ToArray<T>(this Span<T> sourceSpan) => ToList((ReadOnlySpan<T>)sourceSpan);
        public static List<T> ToArray<T>(this ReadOnlySpan<T> sourceSpan)
        {
            List<T> list = new List<T>(sourceSpan.Length);
            var destSpan = CollectionsMarshal.AsSpan(list);
            sourceSpan.CopyTo(destSpan);
            return list;
        }

        public static List<T> ToList<T>(this Span<T> sourceSpan) => [.. sourceSpan];
        public static List<T> ToList<T>(this ReadOnlySpan<T> sourceSpan) => [.. sourceSpan];

        public static T[] TakeToArray<T>(this List<T> source, int count)
        {
            count = int.Clamp(count, 0, source.Count);
            return CollectionsMarshal.AsSpan(source).Slice(0, count).ToArray();
        }

        public static List<T> TrimTo<T>(this List<T> list, int count)
        {
            int toRemove = list.Count - count; 
            if(toRemove > 0) list.RemoveRange(count, toRemove);
            return list;
        }

        public static IEnumerable<T> Generate<T>(int repeats, Func<T> factory)
        {
            for (int i = 0; i < repeats; i++) yield return factory();
        }

        public static IEnumerable<T> Range<T>(this IReadOnlyList<T> list, int start, int end = int.MaxValue)
        {
            if (end > list.Count) end = list.Count;
            for (int i = start; i < end; i++) yield return list[i];
        }

        public static List<T> GetRange<T>(this IReadOnlyList<T> inputList, int index, int count)
        {
            List<T> list = new List<T>(count);
            var rangeEnd = index + count;
            for(int i = index; i < rangeEnd; i++)
            {
                list.Add(inputList[i]);
            }
            return list;
        }

        public static IEnumerable<(T left, T right)> UniquePairs<T>(this IReadOnlyList<T> list)
        {
            for (int i = 0; i < list.Count; i++)
            {
                for (int j = i + 1; j < list.Count; j++)
                {
                    yield return (list[i], list[j]);
                }
            }
        }

        public static IEnumerable<T> TakeFirstAndLast<T>(this IEnumerable<T> source)
        {
            var count = source.Count();
            if (count > 0)
                yield return source.First();
            if (count > 1)
                yield return source.Last();
        }

        public static int IndexOfMinBy<ElementType, TypeToMin>(this ReadOnlySpan<ElementType> span, Func<ElementType, TypeToMin> minSelector) where TypeToMin : IComparable
        {
            if (span.Length == 0) return -1;

            int index = 0;
            TypeToMin min = minSelector(span[0]);
            for (int i = 1; i < span.Length; i++)
            {
                var val = minSelector(span[i]);
                if (val.CompareTo(min) < 0)
                {
                    min = val;
                    index = i;
                }
            }
            return index;
        }

        public static int IndexOfMaxBy<ElementType, TypeToMax>(this ReadOnlySpan<ElementType> list, Func<ElementType, TypeToMax> maxSelector) where TypeToMax : IComparable
        {
            if (list.Length == 0) return -1;

            int index = 0;
            TypeToMax max = maxSelector(list[0]);
            for (int i = 1; i < list.Length; i++)
            {
                var val = maxSelector(list[i]);
                if (val.CompareTo(max) > 0)
                {
                    max = val;
                    index = i;
                }
            }
            return index;
        }

        public static (int iMin, int iMax) IndexOfMinMax<ElementType, TypeToMinMax>(this IReadOnlyList<ElementType> list, Func<ElementType, TypeToMinMax> selector) where TypeToMinMax : IComparable
        {
            if (list.Count == 0) return (-1, -1);

            int iMin = 0;
            int iMax = 0;
            TypeToMinMax min = selector(list[0]);
            TypeToMinMax max = min;
            for (int i = 1; i < list.Count; i++)
            {
                var val = selector(list[i]);
                if (val.CompareTo(min) < 0)
                {
                    min = val;
                    iMin = i;
                }
                else if (val.CompareTo(max) > 0)
                {
                    max = val;
                    iMax = i;
                }
            }
            return (iMin, iMax);
        }

        public static int IndexOfFirst<ElementType>(this IReadOnlyList<ElementType> list, Predicate<ElementType> predicate)
        {
            for (int i = 0; i < list.Count; i++)
            {
                if (predicate(list[i])) return i;
            }
            return -1;
        }

        public static int IndexOfLast<ElementType>(this IReadOnlyList<ElementType> list, Predicate<ElementType> predicate)
        {
            for (int i = list.Count - 1; i >= 0; i--)
            {
                if (predicate(list[i])) return i;
            }
            return -1;
        }

        public static List<T> SortBy<T, T2>(this List<T> list, Func<T, T2> keySelector) where T2 : IComparable
        {
            if(list.Count > 1)
                list.Sort((e1, e2) => keySelector(e1).CompareTo(keySelector(e2)));
            return list;
        }

        public static T[] SortByThenBy<T, T2>(this T[] array, params Func<T, T2>[] keySelectors) where T2 : IComparable
        {
            if (array.Length > 1)
                Array.Sort(array, new ParamsComparer<T, T2>(keySelectors));
            return array;
        }

        public static List<T> SortByThenBy<T, T2>(this List<T> list, params Func<T, T2>[] keySelectors) where T2 : IComparable
        {
            if (list.Count > 1)
                list.Sort(new ParamsComparer<T, T2>(keySelectors));
            return list;
        }

        private class ParamsComparer<T, T2> : IComparer<T> where T2 : IComparable
        {
            private readonly Func<T, T2>[] keySelectors;
            public ParamsComparer(Func<T, T2>[] keySelectors)
            {
                this.keySelectors = keySelectors;
            }

            public int Compare(T x, T y)
            {
                for(int i = 0; i < keySelectors.Length; i++)
                {
                    var selector = keySelectors[i];
                    int result = selector(x).CompareTo(selector(y));
                    if (result != 0) return result;
                }
                return 0;
            }
        }

        public static List<T> SortByDescending<T, T2>(this List<T> list, Func<T, T2> keySelector) where T2 : IComparable
        {
            list.Sort((e1, e2) => keySelector(e2).CompareTo(keySelector(e1)));
            return list;
        }

        public static T[] SortBy<T, T2>(this T[] array, Func<T, T2> keySelector) where T2 : IComparable
        {
            Array.Sort(array, (e1, e2) => keySelector(e1).CompareTo(keySelector(e2)));
            return array;
        }

        public static Span<T> SortBy<T, T2>(this Span<T> span, Func<T, T2> keySelector) where T2 : IComparable
        {
            span.Sort((e1, e2) => keySelector(e1).CompareTo(keySelector(e2)));
            return span;
        }

        public static T[] SortByDescending<T, T2>(this T[] array, Func<T, T2> keySelector) where T2 : IComparable
        {
            Array.Sort(array, (e1, e2) => keySelector(e2).CompareTo(keySelector(e1)));
            return array;
        }
        public static Span<T> SortByDescending<T, T2>(this Span<T> span, Func<T, T2> keySelector) where T2 : IComparable
        {
            span.Sort((e1, e2) => keySelector(e2).CompareTo(keySelector(e1)));
            return span;
        }

        public static T[] Flatten<T>(this IEnumerable<T[]> arrays)
        {
            return ((IEnumerable<ICollection<T>>)arrays).Flatten();
        }

        public static IEnumerable<T> RepeatSequence<T>(this IEnumerable<T> input, int repeats)
        {
            for (int i = 0; i < repeats; i++)
            {
                foreach (T item in input) yield return item;
            }
        }

        public static T[] Flatten<T>(this IEnumerable<ICollection<T>> collections)
        {
            int length = 0;
            foreach (var array in collections)
            {
                length += array.Count;
            }

            int i = 0;
            var result = new T[length];
            foreach (var array in collections)
            {
                array.CopyTo(result, i);
                i += array.Count;
            }
            return result;
        }

        public static void AddSorted<T>(this List<T> list, T item) where T : IComparable<T>
            => AddSorted(list, item, Comparer<T>.Default);

        public static void AddSorted<T, T2>(this List<T> list, T item, Func<T, T2> selector) where T2 : IComparable<T2>
        {
            if (list.Count == 0)
            {
                list.Add(item);
                return;
            }
            T2 itemKey = selector(item);
            if (selector(list[^1]).CompareTo(itemKey) <= 0)
            {
                list.Add(item);
                return;
            }
            if (selector(list[0]).CompareTo(itemKey) >= 0)
            {
                list.Insert(0, item);
                return;
            }
            int index;
            if (list.Count < 32)
            {
                //linear search
                index = list.IndexOfFirst(x => selector(x).CompareTo(itemKey) > 0);
            }
            else
            {
                //binary search
                index = list.BinarySearch(item, new LambdaComparer<T, T2>(selector));
                if (index < 0)
                    index = ~index;
            }            
            list.Insert(index, item);
        }

        private class LambdaComparer<T, T2>(Func<T, T2> selector) : IComparer<T> where T2 : IComparable<T2>
        {
            public int Compare(T x, T y) => selector(x).CompareTo(selector(y));
        }

        public static void AddSorted<T>(this List<T> list, T item, IComparer<T> comparer)
        {
            if (list.Count == 0)
            {
                list.Add(item);
                return;
            }
            if (comparer.Compare(list[^1], item) <= 0)
            {
                list.Add(item);
                return;
            }
            if (comparer.Compare(list[0], item) >= 0)
            {
                list.Insert(0, item);
                return;
            }
            int index = list.BinarySearch(item, comparer);
            if (index < 0)
                index = ~index;
            list.Insert(index, item);
        }

        public static T MinByOrDefault<T, T2>(this IEnumerable<T> sequence, Func<T, T2> selector) where T2 : IComparable<T2>
        {
            using var enumerator = sequence.GetEnumerator();
            bool any = enumerator.MoveNext();
            if (!any)
                return default(T);

            T minT = enumerator.Current;
            while (enumerator.MoveNext())
            {
                var item = enumerator.Current;
                if (minT == null || selector(item).CompareTo(selector(minT)) < 0)
                {
                    minT = item;
                }
            }

            return minT;
        }

        public static T MaxByOrDefault<T, T2>(this IEnumerable<T> sequence, Func<T, T2> selector) where T2 : IComparable<T2>
        {
            using var enumerator = sequence.GetEnumerator();
            bool any = enumerator.MoveNext();
            if (!any)
                return default(T);

            T maxT = enumerator.Current;
            while (enumerator.MoveNext())
            {
                var item = enumerator.Current;
                if (maxT == null || selector(item).CompareTo(selector(maxT)) >= 0)
                {
                    maxT = item;
                }
            }

            return maxT;
        }

        public static T MaxBy<T, T2>(this Span<T> span, Func<T, T2> selector) where T2 : IComparable<T2>
        {
            if (span.IsEmpty)
                return default(T);

            T maxT = span[0];
            for(int i = 1; i < span.Length; i++)
            {
                var item = span[i];
                if (maxT == null || selector(item).CompareTo(selector(maxT)) >= 0)
                {
                    maxT = item;
                }
            }

            return maxT;
        }

        public static T Max<T>(this Span<T> span) where T : IComparable<T>
        {
            if (span.IsEmpty)
                return default;

            T maxT = span[0];
            for (int i = 1; i < span.Length; i++)
            {
                var item = span[i];
                if (maxT == null || item.CompareTo(maxT) >= 0)
                {
                    maxT = item;
                }
            }

            return maxT;
        }

        public static T Min<T>(this Span<T> span) where T : IComparable<T>
        {
            if (span.IsEmpty)
                return default;

            T maxT = span[0];
            for (int i = 1; i < span.Length; i++)
            {
                var item = span[i];
                if (maxT == null || item.CompareTo(maxT) < 0)
                {
                    maxT = item;
                }
            }

            return maxT;
        }

        public static T MinBy<T, T2>(this Span<T> span, Func<T, T2> selector) where T2 : IComparable<T2>
        {
            if (span.IsEmpty)
                return default(T);

            T minT = span[0];
            for (int i = 1; i < span.Length; i++)
            {
                var item = span[i];
                if (minT == null || selector(item).CompareTo(selector(minT)) < 0)
                {
                    minT = item;
                }
            }

            return minT;
        }

        public static bool IsSortedBy<T, TKey>(this IList<T> list, Func<T, TKey> selector) where TKey : IComparable<TKey>
        {
            int count = list.Count;
            if (count <= 1) return true;

            TKey previous = selector(list[0]);
            for (int i = 1; i < count; i++)
            {
                TKey current = selector(list[i]);
                if (current.CompareTo(previous) < 0)
                {
                    return false;
                }
                previous = current;
            }

            return true;
        }

        public static bool IsSortedDescendingBy<T, TKey>(this IList<T> list, Func<T, TKey> selector) where TKey : IComparable<TKey>
        {
            int count = list.Count;

            for (int i = 1; i < count; i++)
            {
                TKey current = selector(list[i]);
                TKey previous = selector(list[i - 1]);

                if (current.CompareTo(previous) > 0)
                {
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// Shuffle the list in place using Fisher-Yates shuffle algorithm.
        /// </summary>
        /// https://en.wikipedia.org/wiki/Fisher%E2%80%93Yates_shuffle
        /// <typeparam name="T">List element type.</typeparam>
        /// <param name="list">List to shuffle.</param>
        /// <param name="rng">optional: random number generator to use</param>
        public static void Shuffle<T>(this List<T> list, Random rng = null) => Shuffle(CollectionsMarshal.AsSpan(list), rng);

        /// <summary>
        /// Shuffle the array in place using Fisher-Yates shuffle algorithm.
        /// </summary>
        /// https://en.wikipedia.org/wiki/Fisher%E2%80%93Yates_shuffle
        /// <typeparam name="T">array element type.</typeparam>
        /// <param name="array">array to shuffle.</param>
        /// <param name="rng">optional: random number generator to use</param>
        public static void Shuffle<T>(this T[] array, Random rng = null) => Shuffle(array.AsSpan(), rng);

        /// <summary>
        /// Shuffle the span in place using Fisher-Yates shuffle algorithm.
        /// </summary>
        /// https://en.wikipedia.org/wiki/Fisher%E2%80%93Yates_shuffle
        /// <typeparam name="T">Span element type.</typeparam>
        /// <param name="span">Span to shuffle.</param>
        /// <param name="rng">optional: random number generator to use</param>
        public static void Shuffle<T>(this Span<T> span, Random rng = null)
        {
            rng ??= new Random();
            int n = span.Length;
            for (int i = 0; i < (n - 1); i++)
            {
                //swap all elements of the list with random other elements
                int r = i + rng.Next(n - i);
                T temp = span[r];
                span[r] = span[i];
                span[i] = temp;
            }
        }
    }
}
