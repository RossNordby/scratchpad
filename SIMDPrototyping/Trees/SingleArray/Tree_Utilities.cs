using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees.SingleArray
{
    partial class Tree
    {
        /// <summary>
        /// Directly sets a leaf bounding box without performing a refit.
        /// </summary>
        /// <param name="leafIndex">Index of the leaf to update the bounding box of.</param>
        /// <param name="boundingBox">New bounding box associated with the leaf.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void SetLeafBoundingBox(int leafIndex, ref BoundingBox boundingBox)
        {
            var leaf = leaves + leafIndex;
            (&nodes[leaf->NodeIndex].A)[leaf->ChildIndex] = boundingBox;
        }
        /// <summary>
        /// Gets a leaf's bounding box in the tree.
        /// </summary>
        /// <param name="leafIndex">Index of the leaf to find the bounding box of.</param>
        /// <param name="boundingBox">Bounding box associated with the leaf.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void GetLeafBoundingBox(int leafIndex, out BoundingBox boundingBox)
        {
            var leaf = leaves + leafIndex;
            boundingBox = (&nodes[leaf->NodeIndex].A)[leaf->ChildIndex];
        }

    }
}
