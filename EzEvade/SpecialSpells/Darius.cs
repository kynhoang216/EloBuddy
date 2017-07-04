using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using EloBuddy;
using EloBuddy.SDK;

namespace ezEvade.SpecialSpells
{
    class Darius : ChampionPlugin
    {
        static Darius()
        {
            // todo: fix for multiple darius' on same team (one for all)
        }

        public void LoadSpecialSpell(SpellData spellData)
        {
            if (spellData.spellName == "DariusCleave")
            {
                var hero = EntityManager.Heroes.AllHeroes.FirstOrDefault(x => x.ChampionName == "Darius");
                if (hero != null && hero.CheckTeam())
                {
                    Game.OnUpdate += (args) => Game_OnUpdate(args, hero);
                }
            }
        }

        private void Game_OnUpdate(EventArgs args, AIHeroClient hero)
        {
            foreach (var spell in SpellDetector.detectedSpells.Where(x => x.Value.heroID == hero.NetworkId))
            {
                if (spell.Value.info.spellName == "DariusCleave")
                {
                    spell.Value.startPos = hero.ServerPosition.To2D();
                    spell.Value.endPos = hero.ServerPosition.To2D() + spell.Value.direction * spell.Value.info.range;
                }
            }
        }
    }
}
