﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using EloBuddy;
using EloBuddy.SDK;
using SharpDX;

namespace ezEvade.SpecialSpells
{
    class Yasuo : ChampionPlugin
    {
        static Yasuo()
        {

        }

        public void LoadSpecialSpell(SpellData spellData)
        {
            if (spellData.spellName == "YasuoQW" || spellData.spellName == "YasuoQ3W")
            {
                var hero = EntityManager.Heroes.AllHeroes.FirstOrDefault(h => h.ChampionName == "Yasuo");
                if (hero != null && hero.CheckTeam())
                {
                    AIHeroClient.OnProcessSpellCast += (sender, args) => ProcessSpell_YasuoQW(sender, args, spellData);
                }
            }
        }

        private static void ProcessSpell_YasuoQW(Obj_AI_Base hero, GameObjectProcessSpellCastEventArgs args, SpellData spellData)
        {
            if (Player.Instance.IsEnemy && args.SData.Name == "YasuoQ")
            {
                var castTime = (Player.Instance.Spellbook.CastTime - Game.Time) * 1000;

                if (castTime > 0)
                {
                    spellData.spellDelay = castTime;
                }
            }
        }
    }
}