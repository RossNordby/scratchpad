using SolverPrototype;
using SolverPrototype.Constraints;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace SolverPrototypeTests
{
    public static class LocalsinitCodegen
    {

        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void Matrix2x3WideTest(ref Matrix2x3Wide m, ref Vector<float> s, out Matrix2x3Wide result)
        {
            //rep stos, ecx 18h (96 bytes)
            Matrix2x3Wide.Scale(ref m, ref s, out var temp);
            Matrix2x3Wide.Scale(ref temp, ref s, out result);
        }


        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void Matrix2x2WideTest(ref Matrix2x2Wide m, ref Vector<float> s, out Matrix2x2Wide result)
        {
            //no locals initialization
            Matrix2x2Wide.Scale(ref m, ref s, out var temp);
            Matrix2x2Wide.Scale(ref temp, ref s, out result);
        }
        [MethodImpl(MethodImplOptions.NoInlining)]
        static void Vector3WideScaleTest(ref Vector3Wide v, ref Vector<float> s, out Vector3Wide result)
        {
            //no locals initialization
            Vector3Wide.Scale(ref v, ref s, out var temp0);
            Vector3Wide.Add(ref temp0, ref v, out result);
        }

        public struct InnerStruct
        {
            public Vector<float> X;
            public Vector<float> Y;

            [MethodImpl(MethodImplOptions.NoInlining)]
            public static void Scale(ref InnerStruct v, ref Vector<float> s, out InnerStruct result)
            {
                result.X = v.X * s;
                result.Y = v.Y * s;
            }
        }
        public struct OuterStruct
        {
            public InnerStruct X;
            public InnerStruct Y;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Scale(ref OuterStruct m, ref Vector<float> s, out OuterStruct result)
            {
                result.X.X = m.X.X * s;
                result.X.Y = m.X.Y * s;
                result.Y.X = m.Y.X * s;
                result.Y.Y = m.Y.Y * s;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void ScaleSubcalls(ref OuterStruct m, ref Vector<float> s, out OuterStruct result)
            {
                InnerStruct.Scale(ref m.X, ref s, out result.X);
                InnerStruct.Scale(ref m.Y, ref s, out result.Y);
            }
        }

        public struct FlattenedStruct
        {
            public Vector<float> XX;
            public Vector<float> XY;
            public Vector<float> YX;
            public Vector<float> YY;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static ref InnerStruct GetX(ref FlattenedStruct m)
            {
                return ref Unsafe.As<Vector<float>, InnerStruct>(ref m.XX);
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static ref InnerStruct GetY(ref FlattenedStruct m)
            {
                return ref Unsafe.As<Vector<float>, InnerStruct>(ref m.YX);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Scale(ref FlattenedStruct m, ref Vector<float> s, out FlattenedStruct result)
            {
                result.XX = m.XX * s;
                result.XY = m.XY * s;
                result.YX = m.YX * s;
                result.YY = m.YY * s;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void ScaleRenested(ref FlattenedStruct m, ref Vector<float> s, out FlattenedStruct result)
            {
                ref var x = ref GetX(ref m);
                result.XX = x.X * s;
                result.XY = x.Y * s;
                ref var y = ref GetY(ref m);
                result.YX = y.X * s;
                result.YY = y.Y * s;
            }
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void NestedStructTest(ref OuterStruct m, ref Vector<float> s, out OuterStruct result)
        {
            //rep stos, ecx 10h (64 bytes)
            OuterStruct.Scale(ref m, ref s, out var intermediate);
            OuterStruct.Scale(ref intermediate, ref s, out result);
        }
        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void SubcallsNestedStructTest(ref OuterStruct m, ref Vector<float> s, out OuterStruct result)
        {
            //rep stos, ecx 10h (64 bytes), with or without inlining on the interior subcall
            OuterStruct.ScaleSubcalls(ref m, ref s, out var intermediate);
            OuterStruct.ScaleSubcalls(ref intermediate, ref s, out result);
        }
        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void FlattenedStructTest(ref FlattenedStruct m, ref Vector<float> s, out FlattenedStruct result)
        {
            //no locals initialization
            FlattenedStruct.Scale(ref m, ref s, out var intermediate);
            FlattenedStruct.Scale(ref intermediate, ref s, out result);
        }
        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void RenestedStructTest(ref FlattenedStruct m, ref Vector<float> s, out FlattenedStruct result)
        {
            //rep stos, ecx 8 (32 bytes)
            FlattenedStruct.ScaleRenested(ref m, ref s, out var intermediate);
            FlattenedStruct.ScaleRenested(ref intermediate, ref s, out result);
        }


        public struct InnerStructForDummy
        {
            public Vector<float> XX;
            public Vector<float> XY;
            public Vector<float> YX;
            public Vector<float> YY;
        }
        public struct DummyOuterStruct
        {
            public InnerStructForDummy Inner;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Scale(ref DummyOuterStruct m, ref Vector<float> s, out DummyOuterStruct result)
            {
                result.Inner.XX = m.Inner.XX * s;
                result.Inner.XY = m.Inner.XY * s;
                result.Inner.YX = m.Inner.YX * s;
                result.Inner.YY = m.Inner.YY * s;
            }
        }
        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void DummyNestedStructTest(ref DummyOuterStruct m, ref Vector<float> s, out DummyOuterStruct result)
        {
            //rep stos, ecx 10h (64 bytes)
            DummyOuterStruct.Scale(ref m, ref s, out var intermediate);
            DummyOuterStruct.Scale(ref intermediate, ref s, out result);
        }


        public struct ScalarOuterStruct8
        {
            public struct ScalarInnerStruct
            {
                public float X;
            }
            public ScalarInnerStruct X;
            public ScalarInnerStruct Y;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void Scale(ref ScalarOuterStruct8 m, float s, out ScalarOuterStruct8 result)
            {
                result.X.X = m.X.X * s;
                result.Y.X = m.Y.X * s;
            }

            [MethodImpl(MethodImplOptions.NoInlining)]
            public static void Test(ref ScalarOuterStruct8 m, float s, out ScalarOuterStruct8 result)
            {
                //8 byte mov clear
                Scale(ref m, s, out var intermediate);
                Scale(ref intermediate, s, out result);
            }
        }

        public struct ScalarOuterStruct16
        {
            public struct ScalarInnerStruct
            {
                public float X;
                public float Y;
            }
            public ScalarInnerStruct X;
            public ScalarInnerStruct Y;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void Scale(ref ScalarOuterStruct16 m, float s, out ScalarOuterStruct16 result)
            {
                result.X.X = m.X.X * s;
                result.X.Y = m.X.Y * s;
                result.Y.X = m.Y.X * s;
                result.Y.Y = m.Y.Y * s;
            }

            [MethodImpl(MethodImplOptions.NoInlining)]
            public static void Test(ref ScalarOuterStruct16 m, float s, out ScalarOuterStruct16 result)
            {
                //rep stos, ecx 4 (16 bytes)
                Scale(ref m, s, out var intermediate);
                Scale(ref intermediate, s, out result);
            }
        }

        public struct ScalarFlattenedStruct
        {
            public float XX;
            public float XY;
            public float YX;
            public float YY;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Scale(ref ScalarFlattenedStruct m, float s, out ScalarFlattenedStruct result)
            {
                result.XX = m.XX * s;
                result.XY = m.XY * s;
                result.YX = m.YX * s;
                result.YY = m.YY * s;
            }
            [MethodImpl(MethodImplOptions.NoInlining)]
            public static void Test(ref ScalarFlattenedStruct m, float s, out ScalarFlattenedStruct result)
            {
                //no locals initialization
                Scale(ref m, s, out var intermediate);
                Scale(ref intermediate, s, out result);
            }
        }


        public struct Scale48
        {
            public Vector<float> A;
            public Vector<float> B;
            public Vector<float> C;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Scale(ref Scale48 m, ref Vector<float> s, out Scale48 result)
            {
                result.A = m.A * s;
                result.B = m.B * s;
                result.C = m.C * s;

            }

            [MethodImpl(MethodImplOptions.NoInlining)]
            public static void Test(ref Scale48 m, ref Vector<float> s, out Scale48 result)
            {
                //no locals initialized
                Scale(ref m, ref s, out var temp);
                Scale(ref temp, ref s, out result);
            }
        }
        public struct Scale64
        {
            public Vector<float> A;
            public Vector<float> B;
            public Vector<float> C;
            public Vector<float> D;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Scale(ref Scale64 m, ref Vector<float> s, out Scale64 result)
            {
                result.A = m.A * s;
                result.B = m.B * s;
                result.C = m.C * s;
                result.D = m.D * s;

            }

            [MethodImpl(MethodImplOptions.NoInlining)]
            public static void Test(ref Scale64 m, ref Vector<float> s, out Scale64 result)
            {
                //no locals initialized
                Scale(ref m, ref s, out var temp);
                Scale(ref temp, ref s, out result);
            }
        }

        public struct Scale80
        {
            public Vector<float> A;
            public Vector<float> B;
            public Vector<float> C;
            public Vector<float> D;
            public Vector<float> E;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Scale(ref Scale80 m, ref Vector<float> s, out Scale80 result)
            {
                result.A = m.A * s;
                result.B = m.B * s;
                result.C = m.C * s;
                result.D = m.D * s;
                result.E = m.E * s;

            }

            [MethodImpl(MethodImplOptions.NoInlining)]
            public static void Test(ref Scale80 m, ref Vector<float> s, out Scale80 result)
            {
                //rep stos, ecx 14h (80 bytes)
                Scale(ref m, ref s, out var temp);
                Scale(ref temp, ref s, out result);
            }
        }
        public struct Scale96
        {
            public Vector<float> A;
            public Vector<float> B;
            public Vector<float> C;
            public Vector<float> D;
            public Vector<float> E;
            public Vector<float> F;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Scale(ref Scale96 m, ref Vector<float> s, out Scale96 result)
            {
                result.A = m.A * s;
                result.B = m.B * s;
                result.C = m.C * s;
                result.D = m.D * s;
                result.E = m.C * s;
                result.F = m.D * s;

            }

            [MethodImpl(MethodImplOptions.NoInlining)]
            public static void Test(ref Scale96 m, ref Vector<float> s, out Scale96 result)
            {
                //rep stos, ecx 18h (96 bytes)
                Scale(ref m, ref s, out var temp);
                Scale(ref temp, ref s, out result);
            }
        }

        public struct Scale128
        {
            public Vector<float> A;
            public Vector<float> B;
            public Vector<float> C;
            public Vector<float> D;
            public Vector<float> E;
            public Vector<float> F;
            public Vector<float> G;
            public Vector<float> H;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Scale(ref Scale128 m, ref Vector<float> s, out Scale128 result)
            {
                result.A = m.A * s;
                result.B = m.B * s;
                result.C = m.C * s;
                result.D = m.D * s;
                result.E = m.E * s;
                result.F = m.F * s;
                result.G = m.G * s;
                result.H = m.H * s;

            }

            [MethodImpl(MethodImplOptions.NoInlining)]
            public static void Test(ref Scale128 m, ref Vector<float> s, out Scale128 result)
            {
                //rep stos, ecx 20h (128 bytes)
                Scale(ref m, ref s, out var temp);
                Scale(ref temp, ref s, out result);
            }
        }


        [StructLayout(LayoutKind.Sequential, Size = 16)]
        struct DummyStruct16
        {
        }
        [StructLayout(LayoutKind.Sequential, Size = 32)]
        struct DummyStruct32
        {
        }
        [StructLayout(LayoutKind.Sequential, Size = 48)]
        struct DummyStruct48
        {
        }
        [StructLayout(LayoutKind.Sequential, Size = 64)]
        struct DummyStruct64
        {
        }
        [StructLayout(LayoutKind.Sequential, Size = 80)]
        struct DummyStruct80
        {
        }
        [StructLayout(LayoutKind.Sequential, Size = 96)]
        struct DummyStruct96
        {
        }
        [StructLayout(LayoutKind.Sequential, Size = 112)]
        struct DummyStruct112
        {
        }
        [StructLayout(LayoutKind.Sequential, Size = 128)]
        struct DummyStruct128
        {
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void InnerDoDummies<T>(ref T a, out T result)
        {
            result = a;
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        static void DoDummies<T>(ref T a, out T result)
        {
            //init across size equal to T size. (for 16 byte case, uses two movs)
            InnerDoDummies(ref a, out var temp);
            InnerDoDummies(ref temp, out result);
        }


        public static void Test()
        {
            {
                var m = new OuterStruct();
                var s = new Vector<float>();
                NestedStructTest(ref m, ref s, out var result);
                SubcallsNestedStructTest(ref m, ref s, out result);

                var m2 = new FlattenedStruct();
                FlattenedStructTest(ref m2, ref s, out var result2);
                RenestedStructTest(ref m2, ref s, out result2);

                var m3 = new DummyOuterStruct();
                DummyNestedStructTest(ref m3, ref s, out var result3);
            }
            {
                var s8 = new ScalarOuterStruct8();
                var s16 = new ScalarOuterStruct16();
                var s = 0f;
                ScalarOuterStruct8.Test(ref s8, s, out var result8);
                ScalarOuterStruct16.Test(ref s16, s, out var result16);

                var m2 = new ScalarFlattenedStruct();
                ScalarFlattenedStruct.Test(ref m2, s, out var result2);

            }
            {
                var v = new Vector3Wide();
                var m2x3 = new Matrix2x3Wide();
                var m2x2 = new Matrix2x2Wide();
                var s = new Vector<float>();
                Vector3WideScaleTest(ref v, ref s, out var vResult);
                Matrix2x3WideTest(ref m2x3, ref s, out var result2x3);
                Matrix2x2WideTest(ref m2x2, ref s, out var result2x2);
            }
            {
                var s16 = new DummyStruct16();
                var s32 = new DummyStruct32();
                var s48 = new DummyStruct48();
                var s64 = new DummyStruct64();
                var s80 = new DummyStruct80();
                var s96 = new DummyStruct96();
                var s112 = new DummyStruct112();
                var s128 = new DummyStruct128();
                DoDummies(ref s16, out var result16);
                DoDummies(ref s32, out var result32);
                DoDummies(ref s48, out var result48);
                DoDummies(ref s64, out var result64);
                DoDummies(ref s80, out var result80);
                DoDummies(ref s96, out var result96);
                DoDummies(ref s112, out var result112);
                DoDummies(ref s128, out var result128);
            }
            {
                var s48 = new Scale48();
                var s64 = new Scale64();
                var s80 = new Scale80();
                var s96 = new Scale96();
                var s128 = new Scale128();
                var s = new Vector<float>();
                Scale48.Test(ref s48, ref s, out var s48Result);
                Scale64.Test(ref s64, ref s, out var s64Result);
                Scale80.Test(ref s80, ref s, out var s80Result);
                Scale96.Test(ref s96, ref s, out var s96Result);
                Scale128.Test(ref s128, ref s, out var s128Result);
            }
        }

    }
}
