using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace GoingWide
{
    public unsafe class SOABundled : Benchmark
    {
        //This isn't the cleanest way to do SOA, but it does demonstrate the core idea!
        Buffer<float> ax, ay, az, bx, by, bz, cx, cy, cz, dx, dy, dz, result;

        public SOABundled()
        {
            ax = pool.Allocate<float>(LaneCount);
            ay = pool.Allocate<float>(LaneCount);
            az = pool.Allocate<float>(LaneCount);
            bx = pool.Allocate<float>(LaneCount);
            by = pool.Allocate<float>(LaneCount);
            bz = pool.Allocate<float>(LaneCount);
            cx = pool.Allocate<float>(LaneCount);
            cy = pool.Allocate<float>(LaneCount);
            cz = pool.Allocate<float>(LaneCount);
            dx = pool.Allocate<float>(LaneCount);
            dy = pool.Allocate<float>(LaneCount);
            dz = pool.Allocate<float>(LaneCount);
            result = pool.Allocate<float>(LaneCount);
        }

        //This is deliberately avoiding AOSOA-ish intermediate layouts for demonstration purposes. Nothing stopping a real SOA implementation from being more reasonable.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Cross(
            in Vector<float> ax, in Vector<float> ay, in Vector<float> az,
            in Vector<float> bx, in Vector<float> by, in Vector<float> bz,
            out Vector<float> rx, out Vector<float> ry, out Vector<float> rz)
        {
            rx = ay * bz - az * by;
            ry = az * bx - ax * bz;
            rz = ax * by - ay * bx;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Dot(
            in Vector<float> ax, in Vector<float> ay, in Vector<float> az,
            in Vector<float> bx, in Vector<float> by, in Vector<float> bz,
            out Vector<float> result)
        {
            result = ax * bx + ay * by + az * bz;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(
            in Vector<float> ax, in Vector<float> ay, in Vector<float> az, in Vector<float> scale,
            out Vector<float> rx, out Vector<float> ry, out Vector<float> rz)
        {
            rx = ax * scale;
            ry = ay * scale;
            rz = az * scale;
        }

        public override void Execute()
        {
            for (int i = 0; i < LaneCount; i += Vector<float>.Count)
            {
                ref var ax = ref Unsafe.As<float, Vector<float>>(ref this.ax[i]);
                ref var ay = ref Unsafe.As<float, Vector<float>>(ref this.ay[i]);
                ref var az = ref Unsafe.As<float, Vector<float>>(ref this.az[i]);
                ref var bx = ref Unsafe.As<float, Vector<float>>(ref this.bx[i]);
                ref var by = ref Unsafe.As<float, Vector<float>>(ref this.by[i]);
                ref var bz = ref Unsafe.As<float, Vector<float>>(ref this.bz[i]);
                ref var cx = ref Unsafe.As<float, Vector<float>>(ref this.cx[i]);
                ref var cy = ref Unsafe.As<float, Vector<float>>(ref this.cy[i]);
                ref var cz = ref Unsafe.As<float, Vector<float>>(ref this.cz[i]);
                ref var dx = ref Unsafe.As<float, Vector<float>>(ref this.dx[i]);
                ref var dy = ref Unsafe.As<float, Vector<float>>(ref this.dy[i]);
                ref var dz = ref Unsafe.As<float, Vector<float>>(ref this.dz[i]);

                Cross(ax, ay, az, bx, by, bz, out var axbx, out var axby, out var axbz);
                Cross(cx, cy, cz, dx, dy, dz, out var cxdx, out var cxdy, out var cxdz);
                Dot(axbx, axby, axbz, ax, ay, az, out var axbDotA);
                Dot(cxdx, cxdy, cxdz, cx, cy, cz, out var cxdDotC);
                Scale(bx, by, bz, axbDotA, out var leftx, out var lefty, out var leftz);
                Scale(bx, by, bz, cxdDotC, out var rightx, out var righty, out var rightz);
                Dot(leftx, lefty, leftz, rightx, righty, rightz, out Unsafe.As<float, Vector<float>>(ref result[i]));
            }
        }
    }
}
