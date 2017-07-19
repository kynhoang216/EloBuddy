using EloBuddy;
using EloBuddy.SDK;
using SharpDX;

namespace Evade.Benchmarking
{
    public static class Benchmark
    {
        private static Vector2 startPoint;
        private static Vector2 endPoint;

        public static void Initialize()
        {
            Game.OnWndProc += Game_OnWndProc;
        }


        static void SpawnLineSkillShot(Vector2 start, Vector2 end)
        {
            SkillshotDetector.TriggerOnDetectSkillshot(
                   DetectionType.ProcessSpell, SpellDatabase.GetByName("TestLineSkillShot"), Utils.TickCount,
                   start, end, end, ObjectManager.Player);

            Core.DelayAction(() => SpawnLineSkillShot(start, end), 5000);
        }

        static void SpawnCircleSkillShot(Vector2 start, Vector2 end)
        {
            SkillshotDetector.TriggerOnDetectSkillshot(
                   DetectionType.ProcessSpell, SpellDatabase.GetByName("TestCircleSkillShot"), Utils.TickCount,
                   start, end, end, ObjectManager.Player);

            Core.DelayAction(() => SpawnCircleSkillShot(start, end), 5000);
        }


        static void Game_OnWndProc(WndEventArgs args)
        {
            if (args.Msg == (uint)WindowMessages.LeftButtonDown)
            {
                startPoint = Game.CursorPos.To2D();
            }

            if (args.Msg == (uint)WindowMessages.LeftButtonUp)
            {
                endPoint = Game.CursorPos.To2D();
            }

            if (args.Msg == (uint)WindowMessages.KeyUp && args.WParam == 'L') //line missile skillshot
            {
                SpawnLineSkillShot(startPoint, endPoint);
            }

            if (args.Msg == (uint)WindowMessages.KeyUp && args.WParam == 'I') //circular skillshoot
            {
                SpawnCircleSkillShot(startPoint, endPoint);
            }


        }
    }
}