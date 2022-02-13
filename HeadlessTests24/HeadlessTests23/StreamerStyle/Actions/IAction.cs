namespace HeadlessTests23.StreamerStyle.Actions;

public interface IAction
{
    void Initialize(Random random, Scene scene);
    bool Update(Scene scene, Random random, float accumulatedTime);
    void Dispose() { }
}

