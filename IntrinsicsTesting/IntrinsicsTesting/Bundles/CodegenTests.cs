using BepuScatter.Tracing;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuScatterDemos
{
    public static class CodegenTests
    {
        //[MethodImpl(MethodImplOptions.NoInlining)]
        //public static void Numerics(out Vector3Wide min, out Vector3Wide max, out Vector3Wide add, out Vector3Wide mul, out Vector3Wide conditional)
        //{
        //    Vector3Wide.Broadcast(new Vector3(4), out var a);
        //    Vector3Wide.Broadcast(new Vector3(5), out var b);
        //    Vector3Wide.Max(a, b, out max);
        //    Vector3Wide.Min(a, b, out min);
        //    Vector3Wide.Add(a, b, out add);
        //    Vector3Wide.Multiply(a, b, out mul);
        //    var condition = new Vector<int>(-1);
        //    Vector3Wide.ConditionalSelect(condition, a, b, out conditional);
        //}
        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void Bundles(out Vector3Bundle min, out Vector3Bundle max, out Vector3Bundle add, out Vector3Bundle mul, out Vector3Bundle conditional)
        {
            Vector3Bundle.Broadcast(new Vector3(4), out var a);
            Vector3Bundle.Broadcast(new Vector3(5), out var b);
            Vector3Bundle.Max(ref a, ref b, out max);
            Vector3Bundle.Min(ref a, ref b, out min);
            Vector3Bundle.Add(ref a, ref b, out add);
            Vector3Bundle.Multiply(ref a, ref b, out mul);
            Wide.Broadcast(-1, out var condition);
            Vector3Bundle.ConditionalSelect(ref condition, ref a, ref b, out conditional);
        }
        public static void Test()
        {
            //Numerics(out var minn, out var maxn, out var addn, out var muln, out var condn);
            Bundles(out var minb, out var maxb, out var addb, out var mulb, out var condb);
        }
    }
}
