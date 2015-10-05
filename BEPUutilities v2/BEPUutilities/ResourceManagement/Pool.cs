using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace BEPUutilities2.ResourceManagement
{
    /// <summary>
    /// Manages a cache of a type of resource.
    /// </summary>
    /// <typeparam name="T">Type of object to pool.</typeparam>
    public abstract class Pool<T> where T : class
    {
        Stack<T> stack = new Stack<T>();
        SpinLock spinLock = new SpinLock();
        /// <summary>
        /// Gets the locker used by LockingTake and LockingReturn.
        /// </summary>
        public SpinLock Locker //WATCH OUT: If you ever change to the System.Threading.SpinLock, it will return by copy.
        {
            get
            {
                return spinLock;
            }
        }

        /// <summary>
        /// Gets or sets the function used to create new objects when the pool has no existing objects available.
        /// </summary>
        public Func<T> Creator
        {
            get; set;
        }

        /// <summary>
        /// Gets or sets the action applied to an element when it is returned to the pool.
        /// </summary>
        public Action<T> Cleaner
        {
            get; set;
        }

        protected Pool(Func<T> creator, Action<T> cleaner = null)
        {
            if (creator == null)
                throw new ArgumentException("Creator must not be null.");
            Creator = creator;
            Cleaner = cleaner;
        }

#if DEBUG
        public HashSet<T> OutstandingElements = new HashSet<T>();
#endif

        /// <summary>
        /// Clears all elements from the pool.
        /// </summary>
        public void Clear()
        {
            stack.Clear();
        }

        [Conditional("DEBUG")]
        public void CheckIfOutstandingElementsExist()
        {
#if DEBUG
            Debug.Assert(OutstandingElements.Count == 0);
#endif
        }

        /// <summary>
        /// Takes an element from the pool. If the pool is empty, a new resource is created and returned.
        /// </summary>
        /// <returns>Element from the pool.</returns>
        public T Take()
        {
            T item;
            if (stack.Count > 0)
            {
                item = stack.Pop();
            }
            else
            {
                item = Creator();
            }
#if DEBUG
            OutstandingElements.Add(item);
#endif
            return item;
        }


        /// <summary>
        /// Returns the specified item to the pool. If a cleaner delegate is set, the item is cleaned.
        /// </summary>
        /// <param name="item">Item to give back to the pool.</param>
        public void Return(T item)
        {
            if (Cleaner != null)
                Cleaner(item);
#if DEBUG
            if (!OutstandingElements.Remove(item))
                throw new InvalidOperationException("Cannot return an item that did not originate from this pool.");
#endif
            stack.Push(item);
        }

        /// <summary>
        /// Takes an element from the pool. If the pool is empty, a new resource is created and returned.
        /// No other LockingTakes or LockingReturns will interfere during the execution of this function.
        /// </summary>
        /// <returns>Element from the pool.</returns>
        public T LockingTake()
        {
            spinLock.Enter();
            var item = Take();
            spinLock.Exit();
            return item;
        }

        /// <summary>
        /// Returns the specified item to the pool. If a cleaner delegate is set, the item is cleaned.
        /// No other LockingTakes or LockingReturns will interfere during the execution of this function.
        /// </summary>
        /// <param name="item">Item to give back to the pool.</param>
        public void LockingReturn(T item)
        {
            spinLock.Enter();
            Return(item);
            spinLock.Exit();
        }

    }
}