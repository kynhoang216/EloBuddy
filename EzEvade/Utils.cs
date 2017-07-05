using EloBuddy;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ezEvade
{
    public static class Utils
    {
        #region Public Properties

        /// <summary>
        ///     Gets the game time tick count.
        /// </summary>
        public static int GameTimeTickCount
        {
            get { return (int)(Game.Time * 1000); }
        }

        /// <summary>
        ///     Gets the tick count.
        /// </summary>
        public static int TickCount
        {
            get { return Environment.TickCount & int.MaxValue; }
        }

        #endregion
    }
}
