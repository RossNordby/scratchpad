using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees
{
#if NODE32
    /// <summary>
    /// 32-wide tree node.
    /// </summary>
    //[StructLayout(LayoutKind.Sequential, Size = 912 /*1168*/)]
    public unsafe struct Node
    {
        public BoundingBox A;
        public BoundingBox B;
        public BoundingBox C;
        public BoundingBox D;
        public BoundingBox E;
        public BoundingBox F;
        public BoundingBox G;
        public BoundingBox H;
        public BoundingBox I;
        public BoundingBox J;
        public BoundingBox K;
        public BoundingBox L;
        public BoundingBox M;
        public BoundingBox N;
        public BoundingBox O;
        public BoundingBox P;
        public BoundingBox A2;
        public BoundingBox B2;
        public BoundingBox C2;
        public BoundingBox D2;
        public BoundingBox E2;
        public BoundingBox F2;
        public BoundingBox G2;
        public BoundingBox H2;
        public BoundingBox I2;
        public BoundingBox J2;
        public BoundingBox K2;
        public BoundingBox L2;
        public BoundingBox M2;
        public BoundingBox N2;
        public BoundingBox O2;
        public BoundingBox P2;
        public int ChildA;
        public int ChildB;
        public int ChildC;
        public int ChildD;
        public int ChildE;
        public int ChildF;
        public int ChildG;
        public int ChildH;
        public int ChildI;
        public int ChildJ;
        public int ChildK;
        public int ChildL;
        public int ChildM;
        public int ChildN;
        public int ChildO;
        public int ChildP;
        public int ChildA2;
        public int ChildB2;
        public int ChildC2;
        public int ChildD2;
        public int ChildE2;
        public int ChildF2;
        public int ChildG2;
        public int ChildH2;
        public int ChildI2;
        public int ChildJ2;
        public int ChildK2;
        public int ChildL2;
        public int ChildM2;
        public int ChildN2;
        public int ChildO2;
        public int ChildP2;
        public int LeafCountA;
        public int LeafCountB;
        public int LeafCountC;
        public int LeafCountD;
        public int LeafCountE;
        public int LeafCountF;
        public int LeafCountG;
        public int LeafCountH;
        public int LeafCountI;
        public int LeafCountJ;
        public int LeafCountK;
        public int LeafCountL;
        public int LeafCountM;
        public int LeafCountN;
        public int LeafCountO;
        public int LeafCountP;
        public int LeafCountA2;
        public int LeafCountB2;
        public int LeafCountC2;
        public int LeafCountD2;
        public int LeafCountE2;
        public int LeafCountF2;
        public int LeafCountG2;
        public int LeafCountH2;
        public int LeafCountI2;
        public int LeafCountJ2;
        public int LeafCountK2;
        public int LeafCountL2;
        public int LeafCountM2;
        public int LeafCountN2;
        public int LeafCountO2;
        public int LeafCountP2;
        public int ChildCount;
        public int Parent;
        public int IndexInParent;
    }
#elif NODE16
    /// <summary>
    /// 16-wide tree node.
    /// </summary>
    //[StructLayout(LayoutKind.Sequential, Size = 464 /*592*/)]
    public unsafe struct Node
    {
        public BoundingBox A;
        public BoundingBox B;
        public BoundingBox C;
        public BoundingBox D;
        public BoundingBox E;
        public BoundingBox F;
        public BoundingBox G;
        public BoundingBox H;
        public BoundingBox I;
        public BoundingBox J;
        public BoundingBox K;
        public BoundingBox L;
        public BoundingBox M;
        public BoundingBox N;
        public BoundingBox O;
        public BoundingBox P;
        public int ChildA;
        public int ChildB;
        public int ChildC;
        public int ChildD;
        public int ChildE;
        public int ChildF;
        public int ChildG;
        public int ChildH;
        public int ChildI;
        public int ChildJ;
        public int ChildK;
        public int ChildL;
        public int ChildM;
        public int ChildN;
        public int ChildO;
        public int ChildP;
        public int LeafCountA;
        public int LeafCountB;
        public int LeafCountC;
        public int LeafCountD;
        public int LeafCountE;
        public int LeafCountF;
        public int LeafCountG;
        public int LeafCountH;
        public int LeafCountI;
        public int LeafCountJ;
        public int LeafCountK;
        public int LeafCountL;
        public int LeafCountM;
        public int LeafCountN;
        public int LeafCountO;
        public int LeafCountP;
        public int ChildCount;
        public int Parent;
        public int IndexInParent;
    }
#elif NODE8
    /// <summary>
    /// 8-wide tree node.
    /// </summary>
    //[StructLayout(LayoutKind.Sequential, Size = 240 /*304*/)]
    public unsafe struct Node
    {
        public BoundingBox A;
        public BoundingBox B;
        public BoundingBox C;
        public BoundingBox D;
        public BoundingBox E;
        public BoundingBox F;
        public BoundingBox G;
        public BoundingBox H;
        public int ChildA;
        public int ChildB;
        public int ChildC;
        public int ChildD;
        public int ChildE;
        public int ChildF;
        public int ChildG;
        public int ChildH;
        public int LeafCountA;
        public int LeafCountB;
        public int LeafCountC;
        public int LeafCountD;
        public int LeafCountE;
        public int LeafCountF;
        public int LeafCountG;
        public int LeafCountH;
        public int ChildCount;
        public int Parent;
        public int IndexInParent;
    }
#elif NODE4
    /// <summary>
    /// 4-wide tree node.
    /// </summary>
    //[StructLayout(LayoutKind.Sequential, Size = 128 /*160*/)]
    public unsafe struct Node
    {
        public BoundingBox A;
        public BoundingBox B;
        public BoundingBox C;
        public BoundingBox D;
        public int ChildA;
        public int ChildB;
        public int ChildC;
        public int ChildD;
        public int LeafCountA;
        public int LeafCountB;
        public int LeafCountC;
        public int LeafCountD;
        public int ChildCount;
        public int Parent;
        public int IndexInParent;
    }
#elif NODE2
    /// <summary>
    /// 2-wide tree node.
    /// </summary>
    //[StructLayout(LayoutKind.Sequential, Size = 64 /*80*/)]
    public unsafe struct Node
    {
        public BoundingBox A;
        public BoundingBox B;
        public int ChildA;
        public int ChildB;
        public int LeafCountA;
        public int LeafCountB;
        public int ChildCount;
        public int Parent;
        public int IndexInParent;
        public int RefineFlag;
    }
#endif
}
