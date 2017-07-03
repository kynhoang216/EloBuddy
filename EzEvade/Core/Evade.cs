using System;
using System.Reflection;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using EloBuddy;
using EloBuddy.SDK;
using EloBuddy.SDK.Menu;
using EloBuddy.SDK.Menu.Values;
using EzEvade;
using SharpDX;

namespace ezEvade
{
    internal class Evade
    {
        public static AIHeroClient myHero { get { return ObjectManager.Player; } }

        public static SpellDetector spellDetector;
        private static SpellDrawer spellDrawer;
        private static EvadeTester evadeTester;
        private static PingTester pingTester;
        private static EvadeSpell evadeSpell;
        private static SpellTester spellTester;
        private static AutoSetPing autoSetPing;

        public static SpellSlot lastSpellCast;
        public static float lastSpellCastTime = 0;

        public static float lastWindupTime = 0;

        public static float lastTickCount = 0;
        public static float lastStopEvadeTime = 0;

        public static Vector3 lastMovementBlockPos = Vector3.Zero;
        public static float lastMovementBlockTime = 0;

        public static float lastEvadeOrderTime = 0;
        public static float lastIssueOrderGameTime = 0;
        public static float lastIssueOrderTime = 0;
        public static PlayerIssueOrderEventArgs lastIssueOrderArgs = null;

        public static Vector2 lastMoveToPosition = Vector2.Zero;
        public static Vector2 lastMoveToServerPos = Vector2.Zero;
        public static Vector2 lastStopPosition = Vector2.Zero;

        public static DateTime assemblyLoadTime = DateTime.Now;

        public static bool isDodging = false;
        public static bool dodgeOnlyDangerous = false;

        public static bool devModeOn = false;
        public static bool hasGameEnded = false;
        public static bool isChanneling = false;
        public static Vector2 channelPosition = Vector2.Zero;

        public static PositionInfo lastPosInfo;

        public static EvadeCommand lastEvadeCommand = new EvadeCommand { isProcessed = true, timestamp = EvadeUtils.TickCount };

        public static EvadeCommand lastBlockedUserMoveTo = new EvadeCommand { isProcessed = true, timestamp = EvadeUtils.TickCount };
        public static float lastDodgingEndTime = 0;

        public static Menu menu;

        public static float sumCalculationTime = 0;
        public static float numCalculationTime = 0;
        public static float avgCalculationTime = 0;

        public Evade()
        {
            LoadAssembly();
        }

        private void LoadAssembly()
        {
            {
                //Game_OnGameLoad(null);
                EloBuddy.SDK.Events.Loading.OnLoadingComplete += Game_OnGameLoad;
            }
        }


        private void Game_OnGameLoad(EventArgs args)
        {
            try
            {
                //devModeOn = true;

                Player.OnIssueOrder += Game_OnIssueOrder;
                Spellbook.OnCastSpell += Game_OnCastSpell;
                Game.OnUpdate += Game_OnGameUpdate;

                AIHeroClient.OnProcessSpellCast += Game_OnProcessSpell;

                Game.OnEnd += Game_OnGameEnd;
                SpellDetector.OnProcessDetectedSpells += SpellDetector_OnProcessDetectedSpells;
                Orbwalker.OnPreAttack += Orbwalking_BeforeAttack;

                //Chat.Print("<font color=\"#66CCFF\" >Yomie's </font><font color=\"#CCFFFF\" >ezEvade </font><font color=\"#66CCFF\" >EzEvade loaded</font>");

                menu = MainMenu.AddMenu("ezEvade", "ezEvade");
                ObjectCache.menuCache.AddMenuToCache(menu);

                Menu mainMenu = menu.AddSubMenuEx("Main", "Main");
                mainMenu.Add("DodgeSkillShots",new KeyBind("Dodge SkillShots", true, KeyBind.BindTypes.PressToggle));
                mainMenu.Add("ActivateEvadeSpells",new KeyBind("Use Evade Spells", true, KeyBind.BindTypes.PressToggle));
                mainMenu.AddSeparator();
                mainMenu.Add("DodgeDangerous", new CheckBox("Dodge Only Dangerous", false));
                mainMenu.Add("DodgeIgnoreHP", new CheckBox("Check Ignored HP %(ChaseMode)"));
                mainMenu.Add("DodgeFOWSpells", new CheckBox("Dodge FOW SkillShots", true));
                mainMenu.Add("DodgeCircularSpells", new CheckBox("Dodge Circular SkillShots", true));
                mainMenu.AddSeparator();
                mainMenu.Add("DodgeDangerousKeyEnabled", new CheckBox("Enable Dodge Only Dangerous Keys", false));
                mainMenu.Add("DodgeDangerousKey",new KeyBind("Dodge Only Dangerous Key", false, KeyBind.BindTypes.HoldActive));
                mainMenu.Add("DodgeDangerousKey2",new KeyBind("Dodge Only Dangerous Key 2", false, KeyBind.BindTypes.HoldActive));
                mainMenu.AddSeparator();
                //   mainMenu.Add("ChaseMode.MinHP", new Slider("Chase Mode enable if my health >= (&)", 20, 0, 100));
                mainMenu.AddGroupLabel("Evade Mode");
                var sliderEvadeMode = mainMenu.Add("EvadeMode", new Slider("Smooth", 1, 0, 2));
                var modeArray = new[] { "Smooth", "Fastest", "Very Smooth" };
                sliderEvadeMode.DisplayName = modeArray[sliderEvadeMode.CurrentValue];
                sliderEvadeMode.OnValueChange +=
                    delegate (ValueBase<int> sender, ValueBase<int>.ValueChangeArgs changeArgs)
                    {
                        sender.DisplayName = modeArray[changeArgs.NewValue];
                        OnEvadeModeChange(sender, changeArgs);
                    };
                //var keyBind = mainMenu.Item("DodgeSkillShots").GetValue<KeyBind>();
                //mainMenu.Item("DodgeSkillShots").SetValue(new KeyBind(keyBind.Key, KeyBindType.Toggle, true));

                spellDetector = new SpellDetector(menu);
                evadeSpell = new EvadeSpell(menu);

                Menu keyMenu = menu.AddSubMenuEx("Key Settings", "KeySettings");
                keyMenu.Add("DodgeDangerousKeyEnabled", new CheckBox("Enable Dodge Only Dangerous Keys", false));
                keyMenu.Add("DodgeDangerousKey", new KeyBind("Dodge Only Dangerous Key", false, KeyBind.BindTypes.PressToggle));
                keyMenu.Add("DodgeDangerousKey2",new KeyBind("Dodge Only Dangerous Key 2", false, KeyBind.BindTypes.HoldActive));
                keyMenu.Add("DodgeOnlyOnComboKeyEnabled", new CheckBox("Enable Dodge Only On Combo Key", false));
                keyMenu.Add("DodgeComboKey", new KeyBind("Dodge Only Combo Key", false, KeyBind.BindTypes.PressToggle));
                keyMenu.Add("DontDodgeKeyEnabled", new CheckBox("Enable Don't Dodge Key", false));
                keyMenu.Add("DontDodgeKey", new KeyBind("Don't Dodge Key", false, KeyBind.BindTypes.PressToggle));
                //menu.AddSubMenu(keyMenu);

                Menu miscMenu = menu.AddSubMenuEx("Misc Settings", "MiscSettings");
                miscMenu.Add("HigherPrecision", new CheckBox("Enhanced Dodge Precision", false));
                miscMenu.Add("RecalculatePosition", new CheckBox("Recalculate Path", true));
                miscMenu.Add("ContinueMovement", new CheckBox("Continue Last Movement", false));
                miscMenu.Add("CalculateWindupDelay", new CheckBox("Calculate Windup Delay", true));
                miscMenu.Add("CheckSpellCollision", new CheckBox("Check Spell Collision", false));
                miscMenu.Add("PreventDodgingUnderTower", new CheckBox("Prevent Dodging Under Tower", false));
                miscMenu.Add("PreventDodgingNearEnemy", new CheckBox("Prevent Dodging Near Enemies", false));
                miscMenu.Add("AdvancedSpellDetection", new CheckBox("Advanced Spell Detection", false));
                //miscMenu.AddItem(new MenuItem("AllowCrossing", "Allow Crossing").SetValue(false));
                //miscMenu.AddItem(new MenuItem("CalculateHeroPos", "Calculate Hero Position").SetValue(false));



                Menu limiterMenu = menu.AddSubMenuEx("Humanizer", "Limiter");
                limiterMenu.Add("ClickOnlyOnce", new CheckBox("Click Only Once", false));
                limiterMenu.Add("EnableEvadeDistance", new CheckBox("Extended Evade", false));
                limiterMenu.Add("TickLimiter", new Slider("Tick Limiter", 0, 0, 500));
                limiterMenu.Add("SpellDetectionTime", new Slider("Spell Detection Time", 0, 0, 1000));
                limiterMenu.Add("ReactionTime", new Slider("Reaction Time", 0, 0, 500));
                limiterMenu.Add("DodgeInterval", new Slider("Dodge Interval", 0, 0, 2000));

                Menu fastEvadeMenu = menu.AddSubMenuEx("Fast Evade", "FastEvade");
                fastEvadeMenu.Add("FastMovementBlock", new CheckBox("Fast Movement Block", false));
                fastEvadeMenu.Add("FastEvadeActivationTime", new Slider("FastEvade Activation Time", 65, 0, 500));
                fastEvadeMenu.Add("SpellActivationTime", new Slider("Spell Activation Time", 400, 0, 1000));
                fastEvadeMenu.Add("RejectMinDistance", new Slider("Collision Distance Buffer", 10, 0, 100));

                /*Menu evadeSpellSettingsMenu = new Menu("Evade Spell", "EvadeSpellMisc");
                evadeSpellSettingsMenu.AddItem(new MenuItem("EvadeSpellActivationTime", "Evade Spell Activation Time").SetValue(new Slider(150, 0, 500)));

                miscMenu.AddSubMenuEx(evadeSpellSettingsMenu);*/

                Menu bufferMenu = menu.AddSubMenuEx("Extra Buffers", "ExtraBuffers");
                bufferMenu.Add("ExtraPingBuffer", new Slider("Extra Ping Buffer", 65, 0, 200));
                bufferMenu.Add("ExtraCPADistance", new Slider("Extra Collision Distance", 10, 0, 150));
                bufferMenu.Add("ExtraSpellRadius", new Slider("Extra Spell Radius", 0, 0, 100));
                bufferMenu.Add("ExtraEvadeDistance", new Slider("Extra Evade Distance", 100, 0, 300));
                bufferMenu.Add("ExtraAvoidDistance", new Slider("Extra Avoid Distance", 50, 0, 300));
                bufferMenu.Add("MinComfortZone", new Slider("Min Distance to Champion", 550, 0, 1000));


           //     Menu resetMenu = menu.AddSubMenuEx("Reset Config", "ResetConfig");
            //    resetMenu.Add("ResetConfig", new CheckBox("Reset Config", false));
           //     resetMenu.Add("ResetConfig200", new CheckBox("Set Patch Config", true));


                //Menu loadTestMenu = menu.AddSubMenuEx("Tests", "LoadTests");
                //loadTestMenu.Add("LoadPingTester", new CheckBox("Load Ping Tester", false)).OnValueChange += OnLoadPingTesterChange;
                //loadTestMenu.Add("LoadSpellTester", new CheckBox("Load Spell Tester", false)).OnValueChange += OnLoadSpellTesterChange;

                spellDrawer = new SpellDrawer(menu);

                //autoSetPing = new AutoSetPing(menu);

                var initCache = ObjectCache.myHeroCache;

                //evadeTester = new EvadeTester(menu);

                //Console.WriteLine("ezEvade Loaded");
            }
            catch (Exception e)
            {
                Console.WriteLine(e);
            }
        }


        public static void ResetConfig(bool kappa = true)
        {
            //TODO
          //  menu["DodgeSkillShots"].Cast<CheckBox>().CurrentValue = false;
            
            //menu.Item("DodgeSkillShots").SetValue(new KeyBind('K', KeyBindType.Toggle, true));
            //menu.Item("ActivateEvadeSpells").SetValue(new KeyBind('K', KeyBindType.Toggle, true));
            //menu.Item("DodgeDangerous").SetValue(false);
            //menu.Item("DodgeFOWSpells").SetValue(true);
            //menu.Item("DodgeCircularSpells").SetValue(true);

            //menu.Item("DodgeDangerousKeyEnabled").SetValue(false);
            //menu.Item("DodgeDangerousKey").SetValue(new KeyBind(32, KeyBindType.Press));
            //menu.Item("DodgeDangerousKey2").SetValue(new KeyBind('V', KeyBindType.Press));

            //menu.Item("HigherPrecision").SetValue(false);
            //menu.Item("RecalculatePosition").SetValue(true);
            //menu.Item("ContinueMovement").SetValue(true);
            //menu.Item("CalculateWindupDelay").SetValue(true);
            //menu.Item("CheckSpellCollision").SetValue(false);
            //menu.Item("PreventDodgingUnderTower").SetValue(false);
            //menu.Item("PreventDodgingNearEnemy").SetValue(true);
            //menu.Item("AdvancedSpellDetection").SetValue(false);
            //menu.Item("LoadPingTester").SetValue(true);

            //menu.Item("EvadeMode").SetValue(new StringList(new[] {"Smooth", "Fastest", "Very Smooth"}, 0));

            //menu.Item("TickLimiter").SetValue(new Slider(100, 0, 500));
            //menu.Item("SpellDetectionTime").SetValue(new Slider(0, 0, 1000));
            //menu.Item("ReactionTime").SetValue(new Slider(0, 0, 500));
            //menu.Item("DodgeInterval").SetValue(new Slider(0, 0, 2000));

            //menu.Item("FastEvadeActivationTime").SetValue(new Slider(65, 0, 500));
            //menu.Item("SpellActivationTime").SetValue(new Slider(200, 0, 1000));
            //menu.Item("RejectMinDistance").SetValue(new Slider(10, 0, 100));

            //menu.Item("ExtraPingBuffer").SetValue(new Slider(65, 0, 200));
            //menu.Item("ExtraCPADistance").SetValue(new Slider(10, 0, 150));
            //menu.Item("ExtraSpellRadius").SetValue(new Slider(0, 0, 100));
            //menu.Item("ExtraEvadeDistance").SetValue(new Slider(100, 0, 300));
            //menu.Item("ExtraAvoidDistance").SetValue(new Slider(50, 0, 300));
            //menu.Item("MinComfortZone").SetValue(new Slider(550, 0, 1000));


        }

        public static void SetPatchConfig()
        {
            menu["FastEvadeActivationTime"].Cast<Slider>().CurrentValue = 0;
            //menu.Item("ExtraAvoidDistance").SetValue(new Slider(0, 0, 300));
            //menu.Item("TickLimiter").SetValue(new Slider(100, 0, 500));
        }

        private void OnEvadeModeChange(ValueBase<int> sender, ValueBase<int>.ValueChangeArgs changeArgs )
        {
            var mode = sender.DisplayName;

            if (mode == "Fastest")
            {
                ResetConfig(false);
                menu["FastMovementBlock"].Cast<CheckBox>().CurrentValue = true;
                menu["FastEvadeActivationTime"].Cast<Slider>().CurrentValue = 120;
                menu["RejectMinDistance"].Cast<Slider>().CurrentValue = 25;
                menu["ExtraCPADistance"].Cast<Slider>().CurrentValue = 25;
                menu["ExtraPingBuffer"].Cast<Slider>().CurrentValue = 80;
            }

            else if (mode == "Very Smooth")
            {
                ResetConfig(false);
                menu["FastEvadeActivationTime"].Cast<Slider>().CurrentValue = 0;
                menu["RejectMinDistance"].Cast<Slider>().CurrentValue = 0;
                menu["ExtraCPADistance"].Cast<Slider>().CurrentValue = 0;
                menu["ExtraPingBuffer"].Cast<Slider>().CurrentValue = 40;
            }
            else if (mode == "Smooth")
            {
                ResetConfig(false);
                menu["FastEvadeActivationTime"].Cast<Slider>().CurrentValue = 65;
                menu["RejectMinDistance"].Cast<Slider>().CurrentValue = 10;
                menu["ExtraCPADistance"].Cast<Slider>().CurrentValue = 10;
                menu["ExtraPingBuffer"].Cast<Slider>().CurrentValue = 65;
            }
        }

        private void OnLoadPingTesterChange(object sender, PlayerIssueOrderEventArgs e)
        {
            e.Process = false;

            if (pingTester == null)
            {
                pingTester = new PingTester();
            }
        }

        private void OnLoadSpellTesterChange(object sender, PlayerIssueOrderEventArgs e)
        {
            e.Process = false;

            if (spellTester == null)
            {
                spellTester = new SpellTester();
            }
        }

        private void Game_OnGameEnd(GameEndEventArgs args)
        {
            hasGameEnded = true;
        }

        private void Game_OnCastSpell(Spellbook spellbook, SpellbookCastSpellEventArgs args)
        {
            if (!spellbook.Owner.IsMe)
                return;

            var sData = spellbook.GetSpell(args.Slot);
            string name;

            if (SpellDetector.channeledSpells.TryGetValue(sData.Name, out name))
            {
                //Evade.isChanneling = true;
                //Evade.channelPosition = ObjectCache.myHeroCache.serverPos2D;
                lastStopEvadeTime = EvadeUtils.TickCount + ObjectCache.gamePing + 100;
            }

            //block spell commmands if evade spell just used
            if (EvadeSpell.lastSpellEvadeCommand != null &&
                EvadeSpell.lastSpellEvadeCommand.timestamp + ObjectCache.gamePing + 150 > EvadeUtils.TickCount)
            {
                args.Process = false;
            }

            lastSpellCast = args.Slot;
            lastSpellCastTime = EvadeUtils.TickCount;

            //moved from processPacket

            /*if (args.Slot == SpellSlot.Recall)
            {
                lastStopPosition = myHero.ServerPosition.To2D();
            }*/

            if (Situation.ShouldDodge())
            {
                if (isDodging && SpellDetector.spells.Any())
                {
                    foreach (KeyValuePair<String, SpellData> entry in SpellDetector.windupSpells)
                    {
                        SpellData spellData = entry.Value;

                        if (spellData.spellKey == args.Slot) //check if it's a spell that we should block
                        {
                            args.Process = false;
                            return;
                        }
                    }
                }
            }


            foreach (var evadeSpell in EvadeSpell.evadeSpells)
            {
                if (evadeSpell.isItem == false && evadeSpell.spellKey == args.Slot &&
                    evadeSpell.untargetable == false)
                {
                    if (evadeSpell.evadeType == EvadeType.Blink)
                    {
                        var blinkPos = args.StartPosition.To2D();

                        var posInfo = EvadeHelper.CanHeroWalkToPos(blinkPos, evadeSpell.speed, ObjectCache.gamePing, 0);
                        if (posInfo != null && posInfo.posDangerLevel == 0)
                        {
                            EvadeCommand.MoveTo(posInfo.position);
                            lastStopEvadeTime = EvadeUtils.TickCount + ObjectCache.gamePing + evadeSpell.spellDelay;
                        }
                    }

                    if (evadeSpell.evadeType == EvadeType.Dash)
                    {
                        var dashPos = args.StartPosition.To2D();

                        if (args.Target != null)
                        {
                            dashPos = args.Target.Position.To2D();
                        }

                        if (evadeSpell.fixedRange || dashPos.Distance(myHero.ServerPosition.To2D()) > evadeSpell.range)
                        {
                            var dir = (dashPos - myHero.ServerPosition.To2D()).Normalized();
                            dashPos = myHero.ServerPosition.To2D() + dir * evadeSpell.range;
                        }

                        var posInfo = EvadeHelper.CanHeroWalkToPos(dashPos, evadeSpell.speed, ObjectCache.gamePing, 0);
                        if (posInfo != null && posInfo.posDangerLevel > 0)
                        {
                            args.Process = false;
                            return;
                        }

                        if (isDodging || EvadeUtils.TickCount < lastDodgingEndTime + 500)
                        {
                            EvadeCommand.MoveTo(Game.CursorPos.To2D());
                            lastStopEvadeTime = EvadeUtils.TickCount + ObjectCache.gamePing + 100;
                        }
                    }
                    return;
                }
            }
        }

        private void Game_OnIssueOrder(Obj_AI_Base hero, PlayerIssueOrderEventArgs args)
        {
            if (!hero.IsMe)
                return;

            if (!Situation.ShouldDodge())
                return;

            if (args.Order == GameObjectOrder.MoveTo)
            {
                if (isDodging && SpellDetector.spells.Any())
                {
                    CheckHeroInDanger();

                    lastBlockedUserMoveTo = new EvadeCommand
                    {
                        order = EvadeOrderCommand.MoveTo,
                        targetPosition = args.TargetPosition.To2D(),
                        timestamp = EvadeUtils.TickCount,
                        isProcessed = false,
                    };

                    args.Process = false;
                }
                else
                {
                    var movePos = args.TargetPosition.To2D();
                    var extraDelay = ObjectCache.menuCache.cache["ExtraPingBuffer"].Cast<Slider>().CurrentValue;

                    if (EvadeHelper.CheckMovePath(movePos, ObjectCache.gamePing + extraDelay))
                    {
                        /*if (ObjectCache.menuCache.cache["AllowCrossing"].GetValue<bool>())
                        {
                            var extraDelayBuffer = ObjectCache.menuCache.cache["ExtraPingBuffer"]
                                .GetValue<Slider>().Value + 30;
                            var extraDist = ObjectCache.menuCache.cache["ExtraCPADistance"]
                                .GetValue<Slider>().Value + 10;
                            var tPosInfo = EvadeHelper.CanHeroWalkToPos(movePos, ObjectCache.myHeroCache.moveSpeed, extraDelayBuffer + ObjectCache.gamePing, extraDist);
                            if (tPosInfo.posDangerLevel == 0)
                            {
                                lastPosInfo = tPosInfo;
                                return;
                            }
                        }*/

                        lastBlockedUserMoveTo = new EvadeCommand
                        {
                            order = EvadeOrderCommand.MoveTo,
                            targetPosition = args.TargetPosition.To2D(),
                            timestamp = EvadeUtils.TickCount,
                            isProcessed = false,
                        };

                        args.Process = false; //Block the command

                        if (EvadeUtils.TickCount - lastMovementBlockTime < 500 && lastMovementBlockPos.Distance(args.TargetPosition) < 100)
                        {
                            return;
                        }

                        lastMovementBlockPos = args.TargetPosition;
                        lastMovementBlockTime = EvadeUtils.TickCount;

                        var posInfo = EvadeHelper.GetBestPositionMovementBlock(movePos);
                        if (posInfo != null)
                        {
                            EvadeCommand.MoveTo(posInfo.position);
                        }
                        return;
                    }
                    else
                    {
                        lastBlockedUserMoveTo.isProcessed = true;
                    }
                }
            }
            else //need more logic
            {
                if (isDodging)
                {
                    args.Process = false; //Block the command
                }
                else
                {
                    if (args.Order == GameObjectOrder.AttackUnit)
                    {
                        var target = args.Target;
                        if (target != null && target.GetType() == typeof(Obj_AI_Base) && ((Obj_AI_Base) target).IsValid())
                        {
                            var baseTarget = target as Obj_AI_Base;
                            if (ObjectCache.myHeroCache.serverPos2D.Distance(baseTarget.ServerPosition.To2D()) >
                                myHero.AttackRange + ObjectCache.myHeroCache.boundingRadius + baseTarget.BoundingRadius)
                            {
                                var movePos = args.TargetPosition.To2D();
                                var extraDelay = ObjectCache.menuCache.cache["ExtraPingBuffer"].Cast<Slider>().CurrentValue;
                                if (EvadeHelper.CheckMovePath(movePos, ObjectCache.gamePing + extraDelay))
                                {
                                    args.Process = false; //Block the command
                                    return;
                                }
                            }
                        }
                    }
                }
            }

            if (args.Process == true)
            {
                lastIssueOrderGameTime = Game.Time * 1000;
                lastIssueOrderTime = EvadeUtils.TickCount;
                lastIssueOrderArgs = args;

                if (args.Order == GameObjectOrder.MoveTo)
                {
                    lastMoveToPosition = args.TargetPosition.To2D();
                    lastMoveToServerPos = myHero.ServerPosition.To2D();
                }

                if (args.Order == GameObjectOrder.Stop)
                {
                    lastStopPosition = myHero.ServerPosition.To2D();
                }
            }
        }

        private void Orbwalking_BeforeAttack(AttackableUnit target, Orbwalker.PreAttackArgs args)
        {
            if (isDodging)
            {
                args.Process = false; //Block orbwalking
            }
        }

        private void Game_OnProcessSpell(Obj_AI_Base hero, GameObjectProcessSpellCastEventArgs args)
        {
            if (!hero.IsMe)
            {
                return;
            }

            string name;
            if (SpellDetector.channeledSpells.TryGetValue(args.SData.Name.ToLower(), out name))
            {
                isChanneling = true;
                channelPosition = myHero.ServerPosition.To2D();
            }

            if (ObjectCache.menuCache.cache["CalculateWindupDelay"].Cast<CheckBox>().CurrentValue)
            {
                var castTime = (hero.Spellbook.CastTime - Game.Time) * 1000;

                if (castTime > 0 && !EloBuddy.SDK.Constants.AutoAttacks.IsAutoAttack(args.SData.Name)
                    && Math.Abs(castTime - myHero.AttackCastDelay * 1000) > 1)
                {
                    lastWindupTime = EvadeUtils.TickCount + castTime - Game.Ping / 2;

                    if (isDodging)
                    {
                        SpellDetector_OnProcessDetectedSpells(); //reprocess
                    }
                }
            }


        }

        private void Game_OnGameUpdate(EventArgs args)
        {
            try
            {
                ObjectCache.myHeroCache.UpdateInfo();
                CheckHeroInDanger();

                if (isChanneling && channelPosition.Distance(ObjectCache.myHeroCache.serverPos2D) > 50
                    && !myHero.Spellbook.IsChanneling)
                {
                    isChanneling = false;
                }

                if (ObjectCache.menuCache.cache["ResetConfig"].Cast<CheckBox>().CurrentValue)
                {
                    ResetConfig();
                    menu["ResetConfig"].Cast<CheckBox>().CurrentValue = false;
                }

                var limitDelay = ObjectCache.menuCache.cache["TickLimiter"].Cast<Slider>().CurrentValue; //Tick limiter                
                if (EvadeHelper.fastEvadeMode || EvadeUtils.TickCount - lastTickCount > limitDelay)
                {
                    if (EvadeUtils.TickCount > lastStopEvadeTime)
                    {
                        DodgeSkillShots(); //walking           
                        ContinueLastBlockedCommand();
                    }

                    lastTickCount = EvadeUtils.TickCount;
                }

                EvadeSpell.UseEvadeSpell(); //using spells
                CheckDodgeOnlyDangerous();
                RecalculatePath();
            }
            catch (Exception e)
            {
                Console.WriteLine(e);
            }
        }

        private void RecalculatePath()
        {
            if (ObjectCache.menuCache.cache["RecalculatePosition"].Cast<CheckBox>().CurrentValue && isDodging)//recheck path
            {
                if (lastPosInfo != null && !lastPosInfo.recalculatedPath)
                {
                    var path = myHero.Path;
                    if (path.Length > 0)
                    {
                        var movePos = path.Last().To2D();

                        if (movePos.Distance(lastPosInfo.position) < 5) //more strict checking
                        {
                            var posInfo = EvadeHelper.CanHeroWalkToPos(movePos, ObjectCache.myHeroCache.moveSpeed, 0, 0, false);
                            if (posInfo.posDangerCount > lastPosInfo.posDangerCount)
                            {
                                lastPosInfo.recalculatedPath = true;

                                if (EvadeSpell.PreferEvadeSpell())
                                {
                                    lastPosInfo = PositionInfo.SetAllUndodgeable();
                                }
                                else
                                {
                                    var newPosInfo = EvadeHelper.GetBestPosition();
                                    if (newPosInfo.posDangerCount < posInfo.posDangerCount)
                                    {
                                        lastPosInfo = newPosInfo;
                                        CheckHeroInDanger();
                                        DodgeSkillShots();
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }



        private void ContinueLastBlockedCommand()
        {
            if (ObjectCache.menuCache.cache["ContinueMovement"].Cast<CheckBox>().CurrentValue
                && Situation.ShouldDodge())
            {
                var movePos = lastBlockedUserMoveTo.targetPosition;
                var extraDelay = ObjectCache.menuCache.cache["ExtraPingBuffer"].Cast<Slider>().CurrentValue;

                if (isDodging == false && lastBlockedUserMoveTo.isProcessed == false
                    && EvadeUtils.TickCount - lastEvadeCommand.timestamp > ObjectCache.gamePing + extraDelay
                    && EvadeUtils.TickCount - lastBlockedUserMoveTo.timestamp < 1500)
                {
                    movePos = movePos + (movePos - ObjectCache.myHeroCache.serverPos2D).Normalized()
                        * EvadeUtils.random.NextFloat(1, 65);

                    if (!EvadeHelper.CheckMovePath(movePos, ObjectCache.gamePing + extraDelay))
                    {
                        //Console.WriteLine("Continue Movement");
                        //myHero.IssueOrder(GameObjectOrder.MoveTo, movePos.To3D());
                        EvadeCommand.MoveTo(movePos);
                        lastBlockedUserMoveTo.isProcessed = true;
                    }
                }
            }
        }

        private void CheckHeroInDanger()
        {
            bool playerInDanger = false;

            foreach (KeyValuePair<int, Spell> entry in SpellDetector.spells)
            {
                Spell spell = entry.Value;

                if (lastPosInfo != null && lastPosInfo.dodgeableSpells.Contains(spell.spellID))
                {
                    if (myHero.ServerPosition.To2D().InSkillShot(spell, ObjectCache.myHeroCache.boundingRadius))
                    {
                        playerInDanger = true;
                        break;
                    }

                    if (ObjectCache.menuCache.cache["EnableEvadeDistance"].Cast<CheckBox>().CurrentValue &&
                        EvadeUtils.TickCount < lastPosInfo.endTime)
                    {
                        playerInDanger = true;
                        break;
                    }
                }
            }

            if (isDodging && !playerInDanger)
            {
                lastDodgingEndTime = EvadeUtils.TickCount;
            }

            if (isDodging == false && !Situation.ShouldDodge())
                return;

            isDodging = playerInDanger;
        }

        private void DodgeSkillShots()
        {
            if (!Situation.ShouldDodge())
            {
                isDodging = false;
                return;
            }

            /*
            if (isDodging && playerInDanger == false) //serverpos test
            {
                myHero.IssueOrder(GameObjectOrder.HoldPosition, myHero, false);
            }*/

            if (isDodging)
            {
                if (lastPosInfo != null)
                {
                    /*foreach (KeyValuePair<int, Spell> entry in SpellDetector.spells)
                    {
                        Spell spell = entry.Value;
                        Console.WriteLine("" + (int)(TickCount-spell.startTime));
                    }*/


                    Vector2 lastBestPosition = lastPosInfo.position;

                    if (ObjectCache.menuCache.cache["ClickOnlyOnce"].Cast<CheckBox>().CurrentValue == false
                        || !(myHero.Path.Count() > 0 && lastPosInfo.position.Distance(myHero.Path.Last().To2D()) < 5))
                    //|| lastPosInfo.timestamp > lastEvadeOrderTime)
                    {
                        EvadeCommand.MoveTo(lastBestPosition);
                        lastEvadeOrderTime = EvadeUtils.TickCount;
                    }
                }
            }
            else //if not dodging
            {
                //Check if hero will walk into a skillshot
                var path = myHero.Path;
                if (path.Length > 0)
                {
                    var movePos = path[path.Length - 1].To2D();

                    if (EvadeHelper.CheckMovePath(movePos))
                    {
                        /*if (ObjectCache.menuCache.cache["AllowCrossing"].GetValue<bool>())
                        {
                            var extraDelayBuffer = ObjectCache.menuCache.cache["ExtraPingBuffer"]
                                .GetValue<Slider>().Value + 30;
                            var extraDist = ObjectCache.menuCache.cache["ExtraCPADistance"]
                                .GetValue<Slider>().Value + 10;
                            var tPosInfo = EvadeHelper.CanHeroWalkToPos(movePos, ObjectCache.myHeroCache.moveSpeed, extraDelayBuffer + ObjectCache.gamePing, extraDist);
                            if (tPosInfo.posDangerLevel == 0)
                            {
                                lastPosInfo = tPosInfo;
                                return;
                            }
                        }*/

                        var posInfo = EvadeHelper.GetBestPositionMovementBlock(movePos);
                        if (posInfo != null)
                        {
                            EvadeCommand.MoveTo(posInfo.position);
                        }
                        return;
                    }
                }
            }
        }


        public void CheckLastMoveTo()
        {
            if (EvadeHelper.fastEvadeMode || ObjectCache.menuCache.cache["FastMovementBlock"].Cast<CheckBox>().CurrentValue)
            {
                if (isDodging == false)
                {
                    if (lastIssueOrderArgs != null && lastIssueOrderArgs.Order == GameObjectOrder.MoveTo)
                    {
                        if (Game.Time * 1000 - lastIssueOrderGameTime < 500)
                        {
                            Game_OnIssueOrder(myHero, lastIssueOrderArgs);
                            lastIssueOrderArgs = null;
                        }
                    }
                }
            }
        }

        public static bool isDodgeDangerousEnabled()
        {
            if (ObjectCache.menuCache.cache["DodgeDangerous"].Cast<CheckBox>().CurrentValue == true)
            {
                return true;
            }

            if (ObjectCache.menuCache.cache["DodgeDangerousKeyEnabled"].Cast<CheckBox>().CurrentValue == true)
            {
                if (ObjectCache.menuCache.cache["DodgeDangerousKey"].Cast<KeyBind>().CurrentValue == true
                    || ObjectCache.menuCache.cache["DodgeDangerousKey2"].Cast<KeyBind>().CurrentValue == true)
                    return true;
            }

            return false;
        }

        public static void CheckDodgeOnlyDangerous() //Dodge only dangerous event
        {
            bool bDodgeOnlyDangerous = isDodgeDangerousEnabled();

            if (dodgeOnlyDangerous == false && bDodgeOnlyDangerous)
            {
                spellDetector.RemoveNonDangerousSpells();
                dodgeOnlyDangerous = true;
            }
            else
            {
                dodgeOnlyDangerous = bDodgeOnlyDangerous;
            }
        }

        public static void SetAllUndodgeable()
        {
            lastPosInfo = PositionInfo.SetAllUndodgeable();
        }

        private void SpellDetector_OnProcessDetectedSpells()
        {
            ObjectCache.myHeroCache.UpdateInfo();

            if (ObjectCache.menuCache.cache["DodgeSkillShots"].Cast<KeyBind>().CurrentValue == false)
            {
                lastPosInfo = PositionInfo.SetAllUndodgeable();
                EvadeSpell.UseEvadeSpell();
                return;
            }

            if (ObjectCache.myHeroCache.serverPos2D.CheckDangerousPos(0)
                || ObjectCache.myHeroCache.serverPos2DExtra.CheckDangerousPos(0))
            {
                if (EvadeSpell.PreferEvadeSpell())
                {
                    lastPosInfo = PositionInfo.SetAllUndodgeable();
                }
                else
                {

                    var posInfo = EvadeHelper.GetBestPosition();
                    var calculationTimer = EvadeUtils.TickCount;
                    var caculationTime = EvadeUtils.TickCount - calculationTimer;

                    //computing time
                    /*if (numCalculationTime > 0)
                    {
                        sumCalculationTime += caculationTime;
                        avgCalculationTime = sumCalculationTime / numCalculationTime;
                    }
                    numCalculationTime += 1;*/

                    //Console.WriteLine("CalculationTime: " + caculationTime);

                    /*if (EvadeHelper.GetHighestDetectedSpellID() > EvadeHelper.GetHighestSpellID(posInfo))
                    {
                        return;
                    }*/
                    if (posInfo != null)
                    {
                        lastPosInfo = posInfo.CompareLastMovePos();

                        var travelTime = ObjectCache.myHeroCache.serverPos2DPing.Distance(lastPosInfo.position) / myHero.MoveSpeed;

                        lastPosInfo.endTime = EvadeUtils.TickCount + travelTime * 1000 - 100;
                    }

                    CheckHeroInDanger();

                    if (EvadeUtils.TickCount > lastStopEvadeTime)
                    {
                        DodgeSkillShots(); //walking           
                    }

                    CheckLastMoveTo();
                    EvadeSpell.UseEvadeSpell(); //using spells
                }
            }
            else
            {
                lastPosInfo = PositionInfo.SetAllDodgeable();
                CheckLastMoveTo();
            }


            //Console.WriteLine("SkillsDodged: " + lastPosInfo.dodgeableSpells.Count + " DangerLevel: " + lastPosInfo.undodgeableSpells.Count);            
        }
    }
}