public interface IRayLeafTester
{
    unsafe void TestLeaf(int leafIndex, RayData* rayData, float* maximumT);
}
