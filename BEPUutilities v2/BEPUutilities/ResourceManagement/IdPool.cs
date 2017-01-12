using System.Collections.Generic;

namespace BEPUutilities2.ResourceManagement
{
    /// <summary>
    /// Manages a pool of identifier values. Grabbing an id from the pool picks a number that has been picked and returned before, 
    /// or if none of those are available, the minimum value greater than any existing id.
    /// </summary>
    public class IdPool
    {
        private int nextIndex;

        /// <summary>
        /// Gets the highest value which any index claimed thus far could possibly have.
        /// This is not necessarily the current highest claimed index; this value may represent an earlier claim that has already been released.
        /// -1 if nothing has ever been claimed.
        /// </summary>
        public int HighestPossiblyClaimedId
        {
            get { return nextIndex - 1; }
        }
        
        public IdPool(int initialCapacity = 128)
        {
            AvailableIds = new Queue<int>(initialCapacity);
        }

        //Note that all availableIds are guaranteed to be less than nextIndex.
        //[0, nextIndex) contains all currently used ids and ids contained within availableIds.
        public Queue<int> AvailableIds
        {
            get; private set;
        }

        public int Take()
        {
            if (AvailableIds.Count > 0)
                return AvailableIds.Dequeue();
            return nextIndex++;
        }

        public void Return(int id)
        {
            AvailableIds.Enqueue(id);
        }
        
        /// <summary>
        /// Resets the pool. Does not handle any outstanding ids.
        /// </summary>
        public void Reset()
        {
            nextIndex = 0;
            AvailableIds.Clear();
        }
    }
}
