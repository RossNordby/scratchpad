using System.Numerics;
using System.Runtime.CompilerServices;
namespace BepuScatter.Tracing
{
    public struct Vector3Bundle
    {
        public Wide<float> X;
        public Wide<float> Y;
        public Wide<float> Z;


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(ref Vector3Bundle a, ref Vector3Bundle b, out Vector3Bundle result)
        {
            Wide.Add(ref a.X, ref b.X, out result.X);
            Wide.Add(ref a.Y, ref b.Y, out result.Y);
            Wide.Add(ref a.Z, ref b.Z, out result.Z);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Subtract(ref Vector3Bundle a, ref Vector3Bundle b, out Vector3Bundle result)
        {
            Wide.Subtract(ref a.X, ref b.X, out result.X);
            Wide.Subtract(ref a.Y, ref b.Y, out result.Y);
            Wide.Subtract(ref a.Z, ref b.Z, out result.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Dot(ref Vector3Bundle a, ref Vector3Bundle b, out Wide<float> result)
        {
            Wide.Multiply(ref a.X, ref b.X, out var x);
            Wide.Multiply(ref a.Y, ref b.Y, out var y);
            Wide.Multiply(ref a.Z, ref b.Z, out var z);
            Wide.Add(ref x, ref y, out var xy);
            Wide.Add(ref xy, ref z, out result);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Min(ref Vector3Bundle a, ref Vector3Bundle b, out Vector3Bundle result)
        {
            Wide.Min(ref a.X, ref b.X, out result.X);
            Wide.Min(ref a.Y, ref b.Y, out result.Y);
            Wide.Min(ref a.Z, ref b.Z, out result.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Max(ref Vector3Bundle a, ref Vector3Bundle b, out Vector3Bundle result)
        {
            Wide.Max(ref a.X, ref b.X, out result.X);
            Wide.Max(ref a.Y, ref b.Y, out result.Y);
            Wide.Max(ref a.Z, ref b.Z, out result.Z);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void LengthSquared(ref Vector3Bundle v, out Wide<float> lengthSquared)
        {
            Dot(ref v, ref v, out lengthSquared);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Length(ref Vector3Bundle v, out Wide<float> length)
        {
            LengthSquared(ref v, out var lengthSquared);
            Wide.Sqrt(ref lengthSquared, out length);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ConditionalSelect(ref Wide<int> condition, ref Vector3Bundle left, ref Vector3Bundle right, out Vector3Bundle result)
        {
            Wide.ConditionalSelect(ref condition, ref left.X, ref right.X, out result.X);
            Wide.ConditionalSelect(ref condition, ref left.Y, ref right.Y, out result.Y);
            Wide.ConditionalSelect(ref condition, ref left.Z, ref right.Z, out result.Z);
        }

        /// <summary>
        /// Multiplies the components of one vector with another.
        /// </summary>
        /// <param name="a">First vector to multiply.</param>
        /// <param name="b">Second vector to multiply.</param>
        /// <param name="result">Result of the multiplication.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Multiply(ref Vector3Bundle a, ref Vector3Bundle b, out Vector3Bundle result)
        {
            Wide.Multiply(ref a.X, ref b.X, out result.X);
            Wide.Multiply(ref a.Y, ref b.Y, out result.Y);
            Wide.Multiply(ref a.Z, ref b.Z, out result.Z);
        }

        ///// <summary>
        ///// Pulls one lane out of the wide representation.
        ///// </summary>
        ///// <param name="wide">Source of the lane.</param>
        ///// <param name="slotIndex">Index of the lane within the wide representation to read.</param>
        ///// <param name="narrow">Non-SIMD type to store the lane in.</param>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public static void ReadSlot(ref Vector3Bundle wide, int slotIndex, out Vector3 narrow)
        //{
        //    ref var offset = ref GatherScatter.GetOffsetInstance(ref wide, slotIndex);
        //    ReadFirst(offset, out narrow);
        //}


        /// <summary>
        /// Pulls one lane out of the wide representation.
        /// </summary>
        /// <param name="source">Source of the lane.</param>
        /// <param name="target">Non-SIMD type to store the lane in.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ReadFirst(in Vector3Bundle source, out Vector3 target)
        {
            target.X = source.X.Value;
            target.Y = source.Y.Value;
            target.Z = source.Z.Value;
        }

        /// <summary>
        /// Gathers values from a vector and places them into the first indices of the target vector.
        /// </summary>
        /// <param name="source">Vector to copy values from.</param>
        /// <param name="targetSlot">Wide vectorto place values into.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WriteFirst(in Vector3 source, ref Vector3Bundle targetSlot)
        {
            targetSlot.X.Value = source.X;
            targetSlot.Y.Value = source.Y;
            targetSlot.Z.Value = source.Z;
        }

        ///// <summary>
        ///// Writes a value into a slot of the target bundle.
        ///// </summary>
        ///// <param name="source">Source of the value to write.</param>
        ///// <param name="slotIndex">Index of the slot to write into.</param>
        ///// <param name="target">Bundle to write the value into.</param>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public static void WriteSlot(in Vector3 source, int slotIndex, ref Vector3Bundle target)
        //{
        //    WriteFirst(source, ref GatherScatter.GetOffsetInstance(ref target, slotIndex));
        //}

        /// <summary>
        /// Expands each scalar value to every slot of the bundle.
        /// </summary>
        /// <param name="source">Source value to write to every bundle slot.</param>
        /// <param name="broadcasted">Bundle containing the source's components in every slot.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Broadcast(in Vector3 source, out Vector3Bundle broadcasted)
        {
            Wide.Broadcast(source.X, out broadcasted.X);
            Wide.Broadcast(source.Y, out broadcasted.Y);
            Wide.Broadcast(source.Z, out broadcasted.Z);
        }

        ///// <summary>
        ///// Takes a slot from the source vector and places it into a slot of the target.
        ///// </summary>
        ///// <param name="source">Vector to pull values from.</param>
        ///// <param name="sourceSlotIndex">Slot in the source vectors to pull values from.</param>
        ///// <param name="target">Target vector whose slot will be filled with the selected data.</param>
        ///// <param name="targetSlotIndex">Slot in the target vectors to write values into.</param>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public static void CopySlot(ref Vector3Bundle source, int sourceSlotIndex, ref Vector3Bundle target, int targetSlotIndex)
        //{
        //    ref var sourceSlot = ref GatherScatter.GetOffsetInstance(ref source, sourceSlotIndex);
        //    ref var targetSlot = ref GatherScatter.GetOffsetInstance(ref target, targetSlotIndex);
        //    targetSlot.X.Value = sourceSlot.X.Value;
        //    targetSlot.Y.Value = sourceSlot.Y.Value;
        //    targetSlot.Z.Value = sourceSlot.Z.Value;
        //}

        public override string ToString()
        {
            return $"<{X}, {Y}, {Z}>";
        }

    }
}
