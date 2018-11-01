//using System.Runtime.CompilerServices;
//using System.Runtime.Intrinsics;
//using System.Runtime.Intrinsics.X86;

//namespace GoingWide
//{

//    public unsafe struct Vector3Shared
//    {
//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static void CrossAVX(
//               in Vector256<float> ax, in Vector256<float> ay, in Vector256<float> az,
//               in Vector256<float> bx, in Vector256<float> by, in Vector256<float> bz,
//               out Vector256<float> rx, out Vector256<float> ry, out Vector256<float> rz)
//        {
//            rx = Avx.Subtract(Avx.Multiply(ay, bz), Avx.Multiply(az, by));
//            ry = Avx.Subtract(Avx.Multiply(az, bx), Avx.Multiply(ax, bz));
//            rz = Avx.Subtract(Avx.Multiply(ax, by), Avx.Multiply(ay, bx));
//        }
//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static void CrossSSE(
//             in Vector128<float> ax, in Vector128<float> ay, in Vector128<float> az,
//             in Vector128<float> bx, in Vector128<float> by, in Vector128<float> bz,
//             out Vector128<float> rx, out Vector128<float> ry, out Vector128<float> rz)
//        {
//            rx = Sse.Subtract(Sse.Multiply(ay, bz), Sse.Multiply(az, by));
//            ry = Sse.Subtract(Sse.Multiply(az, bx), Sse.Multiply(ax, bz));
//            rz = Sse.Subtract(Sse.Multiply(ax, by), Sse.Multiply(ay, bx));
//        }
//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static void CrossScalar(
//             ref float ax, ref float ay, ref float az,
//             ref float bx, ref float by, ref float bz,
//             int offset,
//             out float rx, out float ry, out float rz)
//        {
//            rx = Unsafe.Add(ref ay, offset) * Unsafe.Add(ref bz, offset) - Unsafe.Add(ref az, offset) * Unsafe.Add(ref by, offset);
//            ry = Unsafe.Add(ref az, offset) * Unsafe.Add(ref bx, offset) - Unsafe.Add(ref ax, offset) * Unsafe.Add(ref bz, offset);
//            rz = Unsafe.Add(ref ax, offset) * Unsafe.Add(ref by, offset) - Unsafe.Add(ref ay, offset) * Unsafe.Add(ref bx, offset);
//        }

//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static void Cross<TBundle>(
//            TBundle* axBundle, TBundle* ayBundle, TBundle* azBundle,
//            TBundle* bxBundle, TBundle* byBundle, TBundle* bzBundle,
//            TBundle* rxBundle, TBundle* ryBundle, TBundle* rzBundle) where TBundle : unmanaged, IBundle
//        {
//            if (Avx.IsSupported)
//            {
//                ref var ax = ref Unsafe.AsRef<Vector256<float>>(axBundle);
//                ref var ay = ref Unsafe.AsRef<Vector256<float>>(ayBundle);
//                ref var az = ref Unsafe.AsRef<Vector256<float>>(azBundle);
//                ref var bx = ref Unsafe.AsRef<Vector256<float>>(bxBundle);
//                ref var by = ref Unsafe.AsRef<Vector256<float>>(byBundle);
//                ref var bz = ref Unsafe.AsRef<Vector256<float>>(bzBundle);
//                ref var rx = ref Unsafe.AsRef<Vector256<float>>(rxBundle);
//                ref var ry = ref Unsafe.AsRef<Vector256<float>>(ryBundle);
//                ref var rz = ref Unsafe.AsRef<Vector256<float>>(rzBundle);
//                for (int i = 0; i < default(TBundle).Count; i += 8)
//                {
//                    CrossAVX(
//                        Unsafe.Add(ref ax, i), Unsafe.Add(ref ay, i), Unsafe.Add(ref az, i),
//                        Unsafe.Add(ref bx, i), Unsafe.Add(ref by, i), Unsafe.Add(ref bz, i),
//                        out Unsafe.Add(ref rx, i), out Unsafe.Add(ref ry, i), out Unsafe.Add(ref rz, i));
//                }
//            }
//            else if (Sse.IsSupported)
//            {
//                ref var ax = ref Unsafe.AsRef<Vector128<float>>(axBundle);
//                ref var ay = ref Unsafe.AsRef<Vector128<float>>(ayBundle);
//                ref var az = ref Unsafe.AsRef<Vector128<float>>(azBundle);
//                ref var bx = ref Unsafe.AsRef<Vector128<float>>(bxBundle);
//                ref var by = ref Unsafe.AsRef<Vector128<float>>(byBundle);
//                ref var bz = ref Unsafe.AsRef<Vector128<float>>(bzBundle);
//                ref var rx = ref Unsafe.AsRef<Vector128<float>>(rxBundle);
//                ref var ry = ref Unsafe.AsRef<Vector128<float>>(ryBundle);
//                ref var rz = ref Unsafe.AsRef<Vector128<float>>(rzBundle);
//                for (int i = 0; i < default(TBundle).Count; i += 4)
//                {
//                    CrossSSE(
//                        Unsafe.Add(ref ax, i), Unsafe.Add(ref ay, i), Unsafe.Add(ref az, i),
//                        Unsafe.Add(ref bx, i), Unsafe.Add(ref by, i), Unsafe.Add(ref bz, i),
//                        out Unsafe.Add(ref rx, i), out Unsafe.Add(ref ry, i), out Unsafe.Add(ref rz, i));
//                }
//            }
//            else
//            {
//                ref var ax = ref Unsafe.AsRef<float>(axBundle);
//                ref var ay = ref Unsafe.AsRef<float>(ayBundle);
//                ref var az = ref Unsafe.AsRef<float>(azBundle);
//                ref var bx = ref Unsafe.AsRef<float>(bxBundle);
//                ref var by = ref Unsafe.AsRef<float>(byBundle);
//                ref var bz = ref Unsafe.AsRef<float>(bzBundle);
//                ref var rx = ref Unsafe.AsRef<float>(rxBundle);
//                ref var ry = ref Unsafe.AsRef<float>(ryBundle);
//                ref var rz = ref Unsafe.AsRef<float>(rzBundle);
//                for (int i = 0; i < default(TBundle).Count; i += 4)
//                {
//                    CrossScalar(ref ax, ref ay, ref az, ref bx, ref by, ref bz, i + 0, out rx, out ry, out rz);
//                    CrossScalar(ref ax, ref ay, ref az, ref bx, ref by, ref bz, i + 1, out rx, out ry, out rz);
//                    CrossScalar(ref ax, ref ay, ref az, ref bx, ref by, ref bz, i + 2, out rx, out ry, out rz);
//                    CrossScalar(ref ax, ref ay, ref az, ref bx, ref by, ref bz, i + 3, out rx, out ry, out rz);
//                }
//            }
//        }
//    }
//}
