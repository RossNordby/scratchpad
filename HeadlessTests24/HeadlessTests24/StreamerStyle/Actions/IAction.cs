namespace HeadlessTests24.StreamerStyle.Actions;

public interface IAction
{
    void Initialize(Random random, Scene scene);
    bool Update(Scene scene, Random random,float accumulatedTime, float accumulatedRealTime);
    void Dispose() { }
}

