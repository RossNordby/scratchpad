static class Program
{
    struct SmallCat<T> { }

    interface ITechnique
    {
        void CatSlaps<T>();
    }

    struct ExtremelySmallCat : ITechnique
    {
        public void CatSlaps<T>() { }
    }

    static void Consider<TSomeCat, TAnotherFormOfCat>() where TSomeCat : struct, ITechnique
    {
        default(TSomeCat).CatSlaps<TAnotherFormOfCat>();
    }

    static void CatConcepts<T>()
    {
        Consider<ExtremelySmallCat, SmallCat<T>>();
    }

    static void Main()
    {
        CatConcepts<int>();
    }
}

