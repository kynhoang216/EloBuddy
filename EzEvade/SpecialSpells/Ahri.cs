using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using EloBuddy;
using EloBuddy.SDK;
using SharpDX;
using EloBuddy.SDK.Menu.Values;

namespace ezEvade.SpecialSpells
{
    class Ahri : ChampionPlugin
    {
        public void LoadSpecialSpell(SpellData spellData)
        {
            if (spellData.spellName == "AhriOrbofDeception2")
            {
                var hero = EntityManager.Heroes.AllHeroes.FirstOrDefault(x => x.ChampionName == "Ahri");
                if (hero != null && hero.CheckTeam())
                {
                    Game.OnUpdate += (args) => Game_OnUpdate(args, hero);
                }
            }
        }

        private void Game_OnUpdate(EventArgs args, AIHeroClient hero)
        {
            foreach (
                var spell in
                    SpellDetector.detectedSpells.Where(
                        s =>
                            s.Value.heroID == hero.NetworkId &&
                            s.Value.info.spellName.ToLower() == "ahriorbofdeception2"))
            {
                spell.Value.endPos = hero.ServerPosition.To2D();
            }
        }
    }
}
