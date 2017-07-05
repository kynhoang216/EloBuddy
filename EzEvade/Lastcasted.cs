using EloBuddy;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using EzEvade;

namespace ezEvade
{
    static class LastCastedSpell
    {
        /// <summary>
        /// The casted spells
        /// </summary>
        internal static readonly Dictionary<int, LastCastedSpellEntry> CastedSpells =
            new Dictionary<int, LastCastedSpellEntry>();

        /// <summary>
        /// The last cast packet sent
        /// </summary>
        public static LastCastPacketSentEntry LastCastPacketSent;

        /// <summary>
        /// Initializes static members of the <see cref="LastCastedSpell"/> class. 
        /// </summary>
        static LastCastedSpell()
        {
            Obj_AI_Base.OnProcessSpellCast += Obj_AI_Hero_OnProcessSpellCast;
            Spellbook.OnCastSpell += SpellbookOnCastSpell;
        }

        /// <summary>
        /// Fired then a spell is casted.
        /// </summary>
        /// <param name="spellbook">The spellbook.</param>
        /// <param name="args">The <see cref="SpellbookCastSpellEventArgs"/> instance containing the event data.</param>
        static void SpellbookOnCastSpell(Spellbook spellbook, SpellbookCastSpellEventArgs args)
        {
            if (spellbook.Owner.IsMe)
            {
                LastCastPacketSent = new LastCastPacketSentEntry(
                        args.Slot, Utils.TickCount, (args.Target is Obj_AI_Base) ? args.Target.NetworkId : 0);
            }
        }

        /// <summary>
        /// Fired when the game processes the spell cast.
        /// </summary>
        /// <param name="sender">The sender.</param>
        /// <param name="args">The <see cref="GameObjectProcessSpellCastEventArgs"/> instance containing the event data.</param>
        private static void Obj_AI_Hero_OnProcessSpellCast(Obj_AI_Base sender, GameObjectProcessSpellCastEventArgs args)
        {
            if (sender is AIHeroClient)
            {
                var entry = new LastCastedSpellEntry(args.SData.Name, Utils.TickCount, ObjectManager.Player);
                if (CastedSpells.ContainsKey(sender.NetworkId))
                {
                    CastedSpells[sender.NetworkId] = entry;
                }
                else
                {
                    CastedSpells.Add(sender.NetworkId, entry);
                }
            }
        }

        /// <summary>
        /// Gets the last casted spell tick.
        /// </summary>
        /// <param name="unit">The unit.</param>
        /// <returns></returns>
        public static int LastCastedSpellT(this AIHeroClient unit)
        {
            return CastedSpells.ContainsKey(unit.NetworkId) ? CastedSpells[unit.NetworkId].Tick : (Utils.TickCount > 0 ? 0 : int.MinValue);
        }

        /// <summary>
        /// Gets the last casted spell name.
        /// </summary>
        /// <param name="unit">The unit.</param>
        /// <returns></returns>
        public static string LastCastedSpellName(this AIHeroClient unit)
        {
            return CastedSpells.ContainsKey(unit.NetworkId) ? CastedSpells[unit.NetworkId].Name : string.Empty;
        }

        /// <summary>
        /// Gets the last casted spell's target.
        /// </summary>
        /// <param name="unit">The unit.</param>
        /// <returns></returns>
        public static Obj_AI_Base LastCastedSpellTarget(this AIHeroClient unit)
        {
            return CastedSpells.ContainsKey(unit.NetworkId) ? CastedSpells[unit.NetworkId].Target : null;
        }

        /// <summary>
        /// Gets the last casted spell.
        /// </summary>
        /// <param name="unit">The unit.</param>
        /// <returns></returns>
        public static LastCastedSpellEntry LastCastedspell(this AIHeroClient unit)
        {
            return CastedSpells.ContainsKey(unit.NetworkId) ? CastedSpells[unit.NetworkId] : null;
        }
    }

    public class LastCastedSpellEntry
    {
        /// <summary>
        /// The name
        /// </summary>
        public string Name;

        /// <summary>
        /// The target
        /// </summary>
        public Obj_AI_Base Target;

        /// <summary>
        /// The tick
        /// </summary>
        public int Tick;

        /// <summary>
        /// Initializes a new instance of the <see cref="LastCastedSpellEntry"/> class.
        /// </summary>
        /// <param name="name">The name.</param>
        /// <param name="tick">The tick.</param>
        /// <param name="target">The target.</param>
        public LastCastedSpellEntry(string name, int tick, Obj_AI_Base target)
        {
            Name = name;
            Tick = tick;
            Target = target;
        }
    }

    public class LastCastPacketSentEntry
    {
        /// <summary>
        /// The slot
        /// </summary>
        public SpellSlot Slot;

        /// <summary>
        /// The target network identifier
        /// </summary>
        public int TargetNetworkId;

        /// <summary>
        /// The tick
        /// </summary>
        public int Tick;

        /// <summary>
        /// Initializes a new instance of the <see cref="LastCastPacketSentEntry"/> class.
        /// </summary>
        /// <param name="slot">The slot.</param>
        /// <param name="tick">The tick.</param>
        /// <param name="targetNetworkId">The target network identifier.</param>
        public LastCastPacketSentEntry(SpellSlot slot, int tick, int targetNetworkId)
        {
            Slot = slot;
            Tick = tick;
            TargetNetworkId = targetNetworkId;
        }
    }
}
