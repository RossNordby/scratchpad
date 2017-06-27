using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace SolverPrototype.Collidables
{
    public struct TypedIndex
    {
        uint packed;

        /// <summary>
        /// Gets or sets the type index of the object.
        /// </summary>
        public int Type
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (int)(packed & 0xFF000000) >> 24; }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set { ValidateType(value); packed = (packed & 0x00FFFFFF) | (uint)(value << 24); }
        }

        /// <summary>
        /// Gets or sets the index of the object.
        /// </summary>
        public int Index
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (int)(packed & 0x00FFFFFF); }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set { ValidateIndex(value); packed = (packed & 0xFF000000) | (uint)value; }
        }

        [Conditional("DEBUG")]
        public void ValidateType(int type)
        {
            Debug.Assert(type >= 0 && type < 256, "Do you really have that many type indices, or is the index corrupt?");
        }
        [Conditional("DEBUG")]
        public void ValidateIndex(int index)
        {
            Debug.Assert(index >= 0 && index < 256, "Do you really have that many instances, or is the index corrupt?");
        }

        public TypedIndex(int type, int index)
        {
            ValidateType(type);
            ValidateIndex(index);
            packed = (uint)((type << 24) | index);
        }
    }
}
