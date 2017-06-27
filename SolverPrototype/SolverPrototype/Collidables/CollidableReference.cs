using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace SolverPrototype.Collidables
{
    public struct CollidableReference
    {
        internal uint packed;

        /// <summary>
        /// Gets or sets whether this reference points to a body collidable. If false, the reference points to a static.
        /// </summary>
        public bool IsBody
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (packed & 31) > 0; }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set { packed = (packed & 0x7FFFFFFF) | (value ? 1u << 31 : 0); }
        }

        /// <summary>
        /// Gets or sets the shape type index of the collidable referred to by this instance.
        /// </summary>
        public int ShapeTypeIndex
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (int)((packed >> 24) & 0x0000007F); }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set { ValidateTypeIndex(value); packed = (packed & 0x80FFFFFF) | ((uint)value << 24); }
        }

        /// <summary>
        /// Gets or sets the index of the collidable referred to by this instance.
        /// </summary>
        public int CollidableIndex
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (int)(packed & 0x00FFFFFF); }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set { ValidateCollidableIndex(value); packed = (packed & 0xFF000000) | (uint)value; }
        }

        [Conditional("DEBUG")]
        void ValidateTypeIndex(int shapeTypeIndex)
        {
            Debug.Assert(shapeTypeIndex >= 0 && shapeTypeIndex < 128,
                "If the number of shape type indices somehow exceeds the packing limit, this needs a redesign. Are you sure the type index is right?");
        }

        [Conditional("DEBUG")]
        void ValidateCollidableIndex(int collidableIndex)
        { 
            Debug.Assert(collidableIndex >= 0 && collidableIndex < (1 << 24),
                "If you actually have 16777216+ collidables of a single type, this needs a redesign. Are you sure this is a correct index?");
        }

        public CollidableReference(bool isBody, int shapeTypeIndex, int collidableIndex)
        {
            ValidateTypeIndex(shapeTypeIndex);
            ValidateCollidableIndex(collidableIndex);
            packed = ((isBody ? 1u << 31 : 0u) | ((uint)shapeTypeIndex << 24) | (uint)collidableIndex);
        }

        //We need two know three things:
        //1) Is this associated with a body, or a static? (Activity state is implicit- if it's in the active broadphase, then it's associated with an active body.)
        //2) What is the type of the shape underlying the collidable?
        //3) Where is the collidable in that batch?
        //Bit 31: 0 if body, 1 if static.
        //Bits 24-30: shape type index.
        //Bits 0-23: collidable index within the type batch (within the body or static set).
    }
}
