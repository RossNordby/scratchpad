
using HeadlessTests24.StreamerStyle.Actions;
using HeadlessTests24.StreamerStyle.Scenes;

namespace HeadlessTests24.StreamerStyle;
/// <summary>
/// Constructs a scene from the set of available scenebuilders on demand.
/// </summary>
public class Builders
{
    struct SceneOption
    {
        public string Name;
        public float Weight;
        public Func<Random, Scene> Builder;
    }
    struct ActionOption
    {
        public string Name;
        public float Weight;
        public Func<Random, Scene, IAction> Builder;
    }

    List<SceneOption> sceneOptions = new List<SceneOption>();
    List<ActionOption> actionOptions = new List<ActionOption>();
    void AddSceneOption<T>(float weight) where T : Scene, new()
    {
        sceneOptions.Add(new SceneOption
        {
            Builder = (random) =>
            {
                var scene = new T();
                scene.Initialize(random);
                return scene;
            },
            Weight = weight,
            Name = typeof(T).Name
        });
    }
    void AddActionOption<T>(float weight) where T : IAction, new()
    {
        actionOptions.Add(new ActionOption
        {
            Builder = (random, scene) =>
            {
                var action = new T();
                action.Initialize(random, scene);
                return action;
            },
            Weight = weight,
            Name = typeof(T).Name
        });
    }

    public Builders()
    {
        AddSceneOption<Colosseum>(1f);
        AddSceneOption<InterlockedBars>(1f);
        AddSceneOption<City>(1f);

        AddActionOption<Clonesmash>(0.35f);
        AddActionOption<Shootiepatootie>(1f);
        AddActionOption<Hail>(1f);
        AddActionOption<Ragdolls>(1f);
        AddActionOption<NewtZoom>(1f);
        AddActionOption<NewtHop>(1f);
        AddActionOption<Cars>(1f);
        AddActionOption<Tanks>(1f);
        AddActionOption<TornadoWarning>(1f);
    }

    public (Scene, IAction) Build(Random random)
    {
        var totalWeight = 0f;
        for (int i = 0; i < sceneOptions.Count; ++i)
        {
            totalWeight += sceneOptions[i].Weight;
        }
        var testValue = random.NextDouble() * totalWeight;
        var accumulatedWeight = 0f;
        int nextIndex = 0;
        for (int i = 0; i < sceneOptions.Count; ++i)
        {
            accumulatedWeight += sceneOptions[i].Weight;
            if (testValue < accumulatedWeight)
            {
                nextIndex = i;
                break;
            }
        }
        var scene = sceneOptions[nextIndex].Builder(random);

        totalWeight = 0f;
        for (int i = 0; i < actionOptions.Count; ++i)
        {
            totalWeight += actionOptions[i].Weight;
        }
        testValue = random.NextDouble() * totalWeight;
        accumulatedWeight = 0f;
        nextIndex = 0;
        for (int i = 0; i < actionOptions.Count; ++i)
        {
            accumulatedWeight += actionOptions[i].Weight;
            if (testValue < accumulatedWeight)
            {
                nextIndex = i;
                break;
            }
        }
        return (scene, actionOptions[nextIndex].Builder(random, scene));
        //random = new Random(5);
        //var scene = sceneOptions[1].Builder(content, random);
        //var action = actionOptions[2].Builder(content, random, scene);
        //return (scene, action);
    }
}
