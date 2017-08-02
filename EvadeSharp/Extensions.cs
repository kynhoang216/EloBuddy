using System;
using System.Collections.Generic;
using System.Linq;
using System.Security.AccessControl;
using System.Text;
using System.Threading.Tasks;
using EloBuddy;
using EloBuddy.SDK;
using EloBuddy.SDK.Rendering;
using EloBuddy.SDK.Menu;
using EloBuddy.SDK.Menu.Values;
using SharpDX;
using SharpDX.Direct3D9;
using System.Text.RegularExpressions;
using System.Drawing.Design;
using Color = System.Drawing.Color;
using Font = SharpDX.Direct3D9.Font;
using Rectangle = SharpDX.Rectangle;
using System.Threading;
using System.IO;
using System.Drawing;
using System.Drawing.Imaging;
using Evade;
using EloBuddy.SDK.Enumerations;
using static EloBuddy.SDK.Prediction.Manager;
using EloBuddy.SDK.Events;

namespace Evade
{
    static class Extensions
    {
        public static List<Vector2> GetWaypoints(this Obj_AI_Base unit)
        {
            var result = new List<Vector2>();

            if (unit.IsVisible)
            {
                result.Add(unit.ServerPosition.To2D());
                var path = unit.Path;
                if (path.Length > 0)
                {
                    var first = path[0].To2D();
                    if (first.Distance(result[0], true) > 40)
                    {
                        result.Add(first);
                    }

                    for (int i = 1; i < path.Length; i++)
                    {
                        result.Add(path[i].To2D());
                    }
                }
            }
            else if (WaypointTracker.StoredPaths.ContainsKey(unit.NetworkId))
            {
                var path = WaypointTracker.StoredPaths[unit.NetworkId];
                var timePassed = (Utils.TickCount - WaypointTracker.StoredTick[unit.NetworkId]) / 1000f;
                if (path.PathLength() >= unit.MoveSpeed * timePassed)
                {
                    result = CutPath(path, (int)(unit.MoveSpeed * timePassed));
                }
            }

            return result;
        }

        public static List<Vector2> CutPath(this List<Vector2> path, float distance)
        {
            var result = new List<Vector2>();
            var Distance = distance;
            if (distance < 0)
            {
                path[0] = path[0] + distance * (path[1] - path[0]).Normalized();
                return path;
            }

            for (var i = 0; i < path.Count - 1; i++)
            {
                var dist = path[i].Distance(path[i + 1]);
                if (dist > Distance)
                {
                    result.Add(path[i] + Distance * (path[i + 1] - path[i]).Normalized());
                    for (var j = i + 1; j < path.Count; j++)
                    {
                        result.Add(path[j]);
                    }

                    break;
                }
                Distance -= dist;
            }
            return result.Count > 0 ? result : new List<Vector2> { path.Last() };
        }
    }
    internal static class WaypointTracker
    {
        public static readonly Dictionary<int, List<Vector2>> StoredPaths = new Dictionary<int, List<Vector2>>();
        public static readonly Dictionary<int, int> StoredTick = new Dictionary<int, int>();
    }

    internal static class SGeometry
    {
        public static float PathLength(this List<Vector2> path)
        {
            var distance = 0f;
            for (var i = 0; i < path.Count - 1; i++)
            {
                distance += path[i].Distance(path[i + 1]);
            }
            return distance;
        }
    }

    /// <summary>
    ///     Provides methods regarding geometry math.
    /// </summary>
    public static class SharpGeometry
    {
        //Obj_AI_Base class extended methods:
        /// <summary>
        ///     Calculates the 2D distance to the unit.
        /// </summary>
        /// <param name="anotherUnit">Another unit.</param>
        /// <param name="squared">if set to <c>true</c> [squared].</param>
        /// <returns></returns>
        public static float LSDistance(Obj_AI_Base anotherUnit, bool squared = false)
        {
            return ObjectManager.Player.LSDistance(anotherUnit, squared);
        }

        /// <summary>
        ///     Calculates the 2D distance to the unit.
        /// </summary>
        /// <param name="unit">The unit.</param>
        /// <param name="anotherUnit">Another unit.</param>
        /// <param name="squared">if set to <c>true</c> [squared].</param>
        /// <returns></returns>
        public static float LSDistance(this Obj_AI_Base unit, Obj_AI_Base anotherUnit, bool squared = false)
        {
            return unit.ServerPosition.To2D().LSDistance(anotherUnit.ServerPosition.To2D(), squared);
        }

        /// <summary>
        ///     Calculates the 2D distance to the unit.
        /// </summary>
        /// <param name="unit">The unit.</param>
        /// <param name="anotherUnit">Another unit.</param>
        /// <param name="squared">if set to <c>true</c> [squared].</param>
        /// <returns></returns>
        public static float LSDistance(this Obj_AI_Base unit, AttackableUnit anotherUnit, bool squared = false)
        {
            return unit.ServerPosition.To2D().LSDistance(anotherUnit.Position.To2D(), squared);
        }

        /// <summary>
        ///     Calculates the 2D distance to the point.
        /// </summary>
        /// <param name="unit">The unit.</param>
        /// <param name="point">The point.</param>
        /// <param name="squared">if set to <c>true</c> [squared].</param>
        /// <returns></returns>
        public static float LSDistance(this Obj_AI_Base unit, Vector3 point, bool squared = false)
        {
            return unit.ServerPosition.To2D().LSDistance(point.To2D(), squared);
        }

        /// <summary>
        ///     Calculates the 2D distance to the point.
        /// </summary>
        /// <param name="unit">The unit.</param>
        /// <param name="point">The point.</param>
        /// <param name="squared">if set to <c>true</c> [squared].</param>
        /// <returns></returns>
        public static float LSDistance(this Obj_AI_Base unit, Vector2 point, bool squared = false)
        {
            return unit.ServerPosition.To2D().LSDistance(point, squared);
        }

        /// <summary>
        ///     Calculates the 3D distance to the unit.
        /// </summary>
        /// <param name="unit">The unit.</param>
        /// <param name="anotherUnit">Another unit.</param>
        /// <param name="squared">if set to <c>true</c> [squared].</param>
        /// <returns></returns>
        public static float Distance3D(this Obj_AI_Base unit, Obj_AI_Base anotherUnit, bool squared = false)
        {
            return squared
                ? Vector3.DistanceSquared(unit.Position, anotherUnit.Position)
                : Vector3.Distance(unit.Position, anotherUnit.Position);
        }

        //Vector3 class extended methods:

        /// <summary>
        ///     Converts a Vector3 to Vector2
        /// </summary>
        /// <param name="v">The v.</param>
        /// <returns></returns>
        public static Vector2 To2D(this Vector3 v)
        {
            return new Vector2(v.X, v.Y);
        }

        /// <summary>
        ///     Returns the 2D distance (XY plane) between two vector.
        /// </summary>
        /// <param name="v">The v.</param>
        /// <param name="other">The other.</param>
        /// <param name="squared">if set to <c>true</c> [squared].</param>
        /// <returns></returns>
        public static float LSDistance(this Vector3 v, Vector3 other, bool squared = false)
        {
            return v.To2D().LSDistance(other, squared);
        }

        //Vector2 class extended methods:

        /// <summary>
        ///     Returns true if the vector is valid.
        /// </summary>
        /// <param name="v">The vector.</param>
        /// <returns></returns>
        public static bool IsValid(this Vector2 v)
        {
            return v != Vector2.Zero;
        }

        /// <summary>
        ///     Determines whether this instance is valid.
        /// </summary>
        /// <param name="v">The vector.</param>
        /// <returns></returns>
        public static bool IsValid(this Vector3 v)
        {
            return v != Vector3.Zero;
        }

        /// <summary>
        ///     Converts the Vector2 to Vector3. (Z = Player.ServerPosition.Z)
        /// </summary>
        /// <param name="v">The vector.</param>
        /// <returns></returns>
        public static Vector3 To3D(this Vector2 v)
        {
            return new Vector3(v.X, v.Y, ObjectManager.Player.ServerPosition.Z);
        }

        /// <summary>
        ///     Converts the Vector2 to Vector3. (Z = NavMesh.GetHeightForPosition)
        /// </summary>
        /// <param name="v">The vector.</param>
        /// <returns></returns>
        public static Vector3 To3D2(this Vector2 v)
        {
            return new Vector3(v.X, v.Y, NavMesh.GetHeightForPosition(v.X, v.Y));
        }

        /// <summary>
        ///     Sets the z.
        /// </summary>
        /// <param name="v">The v.</param>
        /// <param name="value">The value.</param>
        /// <returns></returns>
        public static Vector3 LSSetZ(this Vector3 v, float? value = null)
        {
            if (value == null)
            {
                v.Z = Game.CursorPos.Z;
            }
            else
            {
                v.Z = (float)value;
            }
            return v;
        }

        /// <summary>
        ///     Calculates the distance to the Vector2.
        /// </summary>
        /// <param name="v">The v.</param>
        /// <param name="to">To.</param>
        /// <param name="squared">if set to <c>true</c> gets the distance squared.</param>
        /// <returns></returns>
        public static float LSDistance(this Vector2 v, Vector2 to, bool squared = false)
        {
            return squared ? Vector2.DistanceSquared(v, to) : Vector2.Distance(v, to);
        }

        /// <summary>
        ///     Calculates the distance to the Vector3.
        /// </summary>
        /// <param name="v">The v.</param>
        /// <param name="to">To.</param>
        /// <param name="squared">if set to <c>true</c> gets the distance squared.</param>
        /// <returns></returns>
        public static float LSDistance(this Vector2 v, Vector3 to, bool squared = false)
        {
            return v.LSDistance(to.To2D(), squared);
        }

        /// <summary>
        ///     Calculates the distance to the unit.
        /// </summary>
        /// <param name="v">The v.</param>
        /// <param name="to">To.</param>
        /// <param name="squared">if set to <c>true</c> gets the distance squared.</param>
        /// <returns></returns>
        public static float LSDistance(this Vector2 v, Obj_AI_Base to, bool squared = false)
        {
            return v.LSDistance(to.ServerPosition.To2D(), squared);
        }

        /// <summary>
        ///     Returns the distance to the line segment.
        /// </summary>
        /// <param name="point">The point.</param>
        /// <param name="segmentStart">The segment start.</param>
        /// <param name="segmentEnd">The segment end.</param>
        /// <param name="onlyIfOnSegment">if set to <c>true</c> [only if on segment].</param>
        /// <param name="squared">if set to <c>true</c> [squared].</param>
        /// <returns></returns>
        public static float LSDistance(this Vector2 point,
            Vector2 segmentStart,
            Vector2 segmentEnd,
            bool onlyIfOnSegment = false,
            bool squared = false)
        {
            var objects = point.LSProjectOn(segmentStart, segmentEnd);

            if (objects.IsOnSegment || onlyIfOnSegment == false)
            {
                return squared
                    ? Vector2.DistanceSquared(objects.SegmentPoint, point)
                    : Vector2.Distance(objects.SegmentPoint, point);
            }
            return float.MaxValue;
        }

        /// <summary>
        ///     Returns the vector normalized.
        /// </summary>
        /// <param name="v">The vector.</param>
        /// <returns></returns>
        public static Vector2 Normalized(this Vector2 v)
        {
            v.Normalize();
            return v;
        }

        /// <summary>
        ///     Normalizes the specified vector.
        /// </summary>
        /// <param name="v">The vector.</param>
        /// <returns></returns>
        public static Vector3 LSNormalized(this Vector3 v)
        {
            v.Normalize();
            return v;
        }

        /// <summary>
        ///     Extends the vector.
        /// </summary>
        /// <param name="v">The vector.</param>
        /// <param name="to">The vector to extend to</param>
        /// <param name="distance">The distance to extend.</param>
        /// <returns></returns>
        public static Vector2 LSExtend(this Vector2 v, Vector2 to, float distance)
        {
            return v + distance * (to - v).Normalized();
        }

        /// <summary>
        ///     Extends the specified vector.
        /// </summary>
        /// <param name="v">The vector.</param>
        /// <param name="to">The vector to extend to.</param>
        /// <param name="distance">The distance.</param>
        /// <returns></returns>
        public static Vector3 LSExtend(this Vector3 v, Vector3 to, float distance)
        {
            return v + distance * (to - v).LSNormalized();
        }

        /// <summary>
        ///     Shortens the specified vector.
        /// </summary>
        /// <param name="v">The vector.</param>
        /// <param name="to">The vector to shorten from.</param>
        /// <param name="distance">The distance.</param>
        /// <returns></returns>
        public static Vector2 Shorten(this Vector2 v, Vector2 to, float distance)
        {
            return v - distance * (to - v).Normalized();
        }

        /// <summary>
        ///     Shortens the specified vector.
        /// </summary>
        /// <param name="v">The vector.</param>
        /// <param name="to">The vector to shorten from.</param>
        /// <param name="distance">The distance.</param>
        /// <returns></returns>
        public static Vector3 LSShorten(this Vector3 v, Vector3 to, float distance)
        {
            return v - distance * (to - v).LSNormalized();
        }

        /// <summary>
        ///     Switches the Y and Z.
        /// </summary>
        /// <param name="v">The vector.</param>
        /// <returns></returns>
        public static Vector3 LSSwitchYZ(this Vector3 v)
        {
            return new Vector3(v.X, v.Z, v.Y);
        }

        /// <summary>
        ///     Returns the perpendicular vector.
        /// </summary>
        /// <param name="v">The v.</param>
        /// <returns></returns>
        public static Vector2 LSPerpendicular(this Vector2 v)
        {
            return new Vector2(-v.Y, v.X);
        }

        /// <summary>
        ///     Returns the second perpendicular vector.
        /// </summary>
        /// <param name="v">The vector.</param>
        /// <returns></returns>
        public static Vector2 Perpendicular2(this Vector2 v)
        {
            return new Vector2(v.Y, -v.X);
        }

        /// <summary>
        ///     Rotates the vector a set angle (angle in radians).
        /// </summary>
        /// <param name="v">The vector.</param>
        /// <param name="angle">The angle.</param>
        /// <returns></returns>
        public static Vector2 LSRotated(this Vector2 v, float angle)
        {
            var c = Math.Cos(angle);
            var s = Math.Sin(angle);

            return new Vector2((float)(v.X * c - v.Y * s), (float)(v.Y * c + v.X * s));
        }

        /// <summary>
        ///     Returns the cross product Z value.
        /// </summary>
        /// <param name="self">The self.</param>
        /// <param name="other">The other.</param>
        /// <returns></returns>
        public static float LSCrossProduct(this Vector2 self, Vector2 other)
        {
            return other.Y * self.X - other.X * self.Y;
        }

        /// <summary>
        ///     Converts radians to degrees.
        /// </summary>
        /// <param name="angle">The angle.</param>
        /// <returns></returns>
        public static float RadianToDegree(double angle)
        {
            return (float)(angle * (180.0 / Math.PI));
        }

        /// <summary>
        ///     Converts degrees to radians.
        /// </summary>
        /// <param name="angle">The angle.</param>
        /// <returns></returns>
        public static float DegreeToRadian(double angle)
        {
            return (float)(Math.PI * angle / 180.0);
        }

        /// <summary>
        ///     Returns the polar for vector angle (in Degrees).
        /// </summary>
        /// <param name="v1">The vector.</param>
        /// <returns></returns>
        public static float Polar(this Vector2 v1)
        {
            if (Close(v1.X, 0, 0))
            {
                if (v1.Y > 0)
                {
                    return 90;
                }
                return v1.Y < 0 ? 270 : 0;
            }

            var theta = RadianToDegree(Math.Atan(v1.Y / v1.X));
            if (v1.X < 0)
            {
                theta = theta + 180;
            }
            if (theta < 0)
            {
                theta = theta + 360;
            }
            return theta;
        }

        /// <summary>
        ///     Returns the angle with the vector p2 in degrees;
        /// </summary>
        /// <param name="p1">The first point.</param>
        /// <param name="p2">The second point.</param>
        /// <returns></returns>
        public static float LSAngleBetween(this Vector2 p1, Vector2 p2)
        {
            var theta = p1.Polar() - p2.Polar();
            if (theta < 0)
            {
                theta = theta + 360;
            }
            if (theta > 180)
            {
                theta = 360 - theta;
            }
            return theta;
        }

        /// <summary>
        ///     Returns the closest vector from a list.
        /// </summary>
        /// <param name="v">The v.</param>
        /// <param name="vList">The v list.</param>
        /// <returns></returns>
        public static Vector2 Closest(this Vector2 v, List<Vector2> vList)
        {
            var result = new Vector2();
            var dist = float.MaxValue;

            foreach (var vector in vList)
            {
                var distance = Vector2.DistanceSquared(v, vector);
                if (distance < dist)
                {
                    dist = distance;
                    result = vector;
                }
            }

            return result;
        }

        /// <summary>
        ///     Returns the projection of the Vector2 on the segment.
        /// </summary>
        /// <param name="point">The point.</param>
        /// <param name="segmentStart">The segment start.</param>
        /// <param name="segmentEnd">The segment end.</param>
        /// <returns></returns>
        public static ProjectionInfo LSProjectOn(this Vector2 point, Vector2 segmentStart, Vector2 segmentEnd)
        {
            var cx = point.X;
            var cy = point.Y;
            var ax = segmentStart.X;
            var ay = segmentStart.Y;
            var bx = segmentEnd.X;
            var by = segmentEnd.Y;
            var rL = ((cx - ax) * (bx - ax) + (cy - ay) * (by - ay)) /
                     ((float)Math.Pow(bx - ax, 2) + (float)Math.Pow(by - ay, 2));
            var pointLine = new Vector2(ax + rL * (bx - ax), ay + rL * (by - ay));
            float rS;
            if (rL < 0)
            {
                rS = 0;
            }
            else if (rL > 1)
            {
                rS = 1;
            }
            else
            {
                rS = rL;
            }

            var isOnSegment = rS.CompareTo(rL) == 0;
            var pointSegment = isOnSegment ? pointLine : new Vector2(ax + rS * (bx - ax), ay + rS * (@by - ay));
            return new ProjectionInfo(isOnSegment, pointSegment, pointLine);
        }

        //From: http://social.msdn.microsoft.com/Forums/vstudio/en-US/e5993847-c7a9-46ec-8edc-bfb86bd689e3/help-on-line-segment-intersection-algorithm
        /// <summary>
        ///     Intersects two line segments.
        /// </summary>
        /// <param name="lineSegment1Start">The line segment1 start.</param>
        /// <param name="lineSegment1End">The line segment1 end.</param>
        /// <param name="lineSegment2Start">The line segment2 start.</param>
        /// <param name="lineSegment2End">The line segment2 end.</param>
        /// <returns></returns>
        public static IntersectionResult LSIntersection(this Vector2 lineSegment1Start,
            Vector2 lineSegment1End,
            Vector2 lineSegment2Start,
            Vector2 lineSegment2End)
        {
            double deltaACy = lineSegment1Start.Y - lineSegment2Start.Y;
            double deltaDCx = lineSegment2End.X - lineSegment2Start.X;
            double deltaACx = lineSegment1Start.X - lineSegment2Start.X;
            double deltaDCy = lineSegment2End.Y - lineSegment2Start.Y;
            double deltaBAx = lineSegment1End.X - lineSegment1Start.X;
            double deltaBAy = lineSegment1End.Y - lineSegment1Start.Y;

            var denominator = deltaBAx * deltaDCy - deltaBAy * deltaDCx;
            var numerator = deltaACy * deltaDCx - deltaACx * deltaDCy;

            if (Math.Abs(denominator) < float.Epsilon)
            {
                if (Math.Abs(numerator) < float.Epsilon)
                {
                    // collinear. Potentially infinite intersection points.
                    // Check and return one of them.
                    if (lineSegment1Start.X >= lineSegment2Start.X && lineSegment1Start.X <= lineSegment2End.X)
                    {
                        return new IntersectionResult(true, lineSegment1Start);
                    }
                    if (lineSegment2Start.X >= lineSegment1Start.X && lineSegment2Start.X <= lineSegment1End.X)
                    {
                        return new IntersectionResult(true, lineSegment2Start);
                    }
                    return new IntersectionResult();
                }
                // parallel
                return new IntersectionResult();
            }

            var r = numerator / denominator;
            if (r < 0 || r > 1)
            {
                return new IntersectionResult();
            }

            var s = (deltaACy * deltaBAx - deltaACx * deltaBAy) / denominator;
            if (s < 0 || s > 1)
            {
                return new IntersectionResult();
            }

            return new IntersectionResult(
                true,
                new Vector2((float)(lineSegment1Start.X + r * deltaBAx), (float)(lineSegment1Start.Y + r * deltaBAy)));
        }

        /// <summary>
        ///     Gets the vectors movement collision.
        /// </summary>
        /// <param name="startPoint1">The start point1.</param>
        /// <param name="endPoint1">The end point1.</param>
        /// <param name="v1">The v1.</param>
        /// <param name="startPoint2">The start point2.</param>
        /// <param name="v2">The v2.</param>
        /// <param name="delay">The delay.</param>
        /// <returns></returns>
        public static object[] VectorMovementCollision(Vector2 startPoint1,
            Vector2 endPoint1,
            float v1,
            Vector2 startPoint2,
            float v2,
            float delay = 0f)
        {
            float sP1x = startPoint1.X,
                sP1y = startPoint1.Y,
                eP1x = endPoint1.X,
                eP1y = endPoint1.Y,
                sP2x = startPoint2.X,
                sP2y = startPoint2.Y;

            float d = eP1x - sP1x, e = eP1y - sP1y;
            float dist = (float)Math.Sqrt(d * d + e * e), t1 = float.NaN;
            float S = Math.Abs(dist) > float.Epsilon ? v1 * d / dist : 0,
                K = Math.Abs(dist) > float.Epsilon ? v1 * e / dist : 0f;

            float r = sP2x - sP1x, j = sP2y - sP1y;
            var c = r * r + j * j;


            if (dist > 0f)
            {
                if (Math.Abs(v1 - float.MaxValue) < float.Epsilon)
                {
                    var t = dist / v1;
                    t1 = v2 * t >= 0f ? t : float.NaN;
                }
                else if (Math.Abs(v2 - float.MaxValue) < float.Epsilon)
                {
                    t1 = 0f;
                }
                else
                {
                    float a = S * S + K * K - v2 * v2, b = -r * S - j * K;

                    if (Math.Abs(a) < float.Epsilon)
                    {
                        if (Math.Abs(b) < float.Epsilon)
                        {
                            t1 = Math.Abs(c) < float.Epsilon ? 0f : float.NaN;
                        }
                        else
                        {
                            var t = -c / (2 * b);
                            t1 = v2 * t >= 0f ? t : float.NaN;
                        }
                    }
                    else
                    {
                        var sqr = b * b - a * c;
                        if (sqr >= 0)
                        {
                            var nom = (float)Math.Sqrt(sqr);
                            var t = (-nom - b) / a;
                            t1 = v2 * t >= 0f ? t : float.NaN;
                            t = (nom - b) / a;
                            var t2 = v2 * t >= 0f ? t : float.NaN;

                            if (!float.IsNaN(t2) && !float.IsNaN(t1))
                            {
                                if (t1 >= delay && t2 >= delay)
                                {
                                    t1 = Math.Min(t1, t2);
                                }
                                else if (t2 >= delay)
                                {
                                    t1 = t2;
                                }
                            }
                        }
                    }
                }
            }
            else if (Math.Abs(dist) < float.Epsilon)
            {
                t1 = 0f;
            }

            return new object[] { t1, !float.IsNaN(t1) ? new Vector2(sP1x + S * t1, sP1y + K * t1) : new Vector2() };
        }

        /// <summary>
        ///     Returns the total distance of a path.
        /// </summary>
        /// <param name="path">The path.</param>
        /// <returns></returns>
        public static float LSPathLength(this List<Vector2> path)
        {
            var distance = 0f;
            for (var i = 0; i < path.Count - 1; i++)
            {
                distance += path[i].LSDistance(path[i + 1]);
            }
            return distance;
        }

        /// <summary>
        ///     Converts a 3D path to 2D
        /// </summary>
        /// <param name="path">The path.</param>
        /// <returns></returns>
        public static List<Vector2> LSTo2D(this List<Vector3> path)
        {
            return path.Select(point => point.To2D()).ToList();
        }

        /// <summary>
        ///     Returns the two intersection points between two circles.
        /// </summary>
        /// <param name="center1">The center1.</param>
        /// <param name="center2">The center2.</param>
        /// <param name="radius1">The radius1.</param>
        /// <param name="radius2">The radius2.</param>
        /// <returns></returns>
        public static Vector2[] CircleCircleIntersection(Vector2 center1, Vector2 center2, float radius1, float radius2)
        {
            var D = center1.LSDistance(center2);
            //The Circles dont intersect:
            if (D > radius1 + radius2 || (D <= Math.Abs(radius1 - radius2)))
            {
                return new Vector2[] { };
            }

            var A = (radius1 * radius1 - radius2 * radius2 + D * D) / (2 * D);
            var H = (float)Math.Sqrt(radius1 * radius1 - A * A);
            var Direction = (center2 - center1).Normalized();
            var PA = center1 + A * Direction;
            var S1 = PA + H * Direction.LSPerpendicular();
            var S2 = PA - H * Direction.LSPerpendicular();
            return new[] { S1, S2 };
        }

        /// <summary>
        ///     Checks if the two floats are close to each other.
        /// </summary>
        /// <param name="a">a.</param>
        /// <param name="b">The b.</param>
        /// <param name="eps">The epsilon.</param>
        /// <returns></returns>
        public static bool Close(float a, float b, float eps)
        {
            if (Math.Abs(eps) < float.Epsilon)
            {
                eps = (float)1e-9;
            }
            return Math.Abs(a - b) <= eps;
        }

        /// <summary>
        ///     Rotates the vector around the set position.
        ///     Angle is in radians.
        /// </summary>
        /// <param name="rotated">The rotated.</param>
        /// <param name="around">The around.</param>
        /// <param name="angle">The angle.</param>
        /// <returns></returns>
        public static Vector2 RotateAroundPoint(this Vector2 rotated, Vector2 around, float angle)
        {
            var sin = Math.Sin(angle);
            var cos = Math.Cos(angle);

            var x = cos * (rotated.X - around.X) - sin * (rotated.Y - around.Y) + around.X;
            var y = sin * (rotated.X - around.X) + cos * (rotated.Y - around.Y) + around.Y;

            return new Vector2((float)x, (float)y);
        }

        /// <summary>
        ///     Moves the polygone to the set position. It dosent rotate the polygone.
        /// </summary>
        /// <param name="polygon">The polygon.</param>
        /// <param name="moveTo">The move to.</param>
        /// <returns></returns>




        /// <summary>
        ///     Joins all the polygones in the list in one polygone if they interect.
        /// </summary>
        /// <param name="sList">The polygon list.</param>
        /// <returns></returns>



        /// <summary>
        ///     Converts a list of <see cref="IntPoint" /> to a polygon.
        /// </summary>
        /// <param name="v">The int points.</param>
        /// <returns></returns>
        public static Polygon ToPolygon(this List<System.Drawing.Point> v)
        {
            var polygon = new Polygon();
            foreach (var point in v)
            { }
            return polygon;
        }

        /// <summary>
        ///     Represents an intersection result.
        /// </summary>
        public struct IntersectionResult
        {
            /// <summary>
            ///     If they intersect.
            /// </summary>
            public bool Intersects;

            /// <summary>
            ///     The point
            /// </summary>
            public Vector2 Point;

            /// <summary>
            ///     Initializes a new instance of the <see cref="IntersectionResult" /> struct.
            /// </summary>
            /// <param name="Intersects">if set to <c>true</c>, they insersect.</param>
            /// <param name="Point">The point.</param>
            public IntersectionResult(bool Intersects = false, Vector2 Point = new Vector2())
            {
                this.Intersects = Intersects;
                this.Point = Point;
            }
        }

        /// <summary>
        ///     Represents the projection information.
        /// </summary>
        public struct ProjectionInfo
        {
            /// <summary>
            ///     The is on segment
            /// </summary>
            public bool IsOnSegment;

            /// <summary>
            ///     The line point
            /// </summary>
            public Vector2 LinePoint;

            /// <summary>
            ///     The segment point
            /// </summary>
            public Vector2 SegmentPoint;

            /// <summary>
            ///     Initializes a new instance of the <see cref="ProjectionInfo" /> struct.
            /// </summary>
            /// <param name="isOnSegment">if set to <c>true</c> [is on segment].</param>
            /// <param name="segmentPoint">The segment point.</param>
            /// <param name="linePoint">The line point.</param>
            public ProjectionInfo(bool isOnSegment, Vector2 segmentPoint, Vector2 linePoint)
            {
                IsOnSegment = isOnSegment;
                SegmentPoint = segmentPoint;
                LinePoint = linePoint;
            }
        }

        /// <summary>
        ///     Represents a polygon.
        /// </summary>
        public class Polygon
        {
            /// <summary>
            ///     The points
            /// </summary>
            public List<Vector2> Points = new List<Vector2>();

            /// <summary>
            ///     Represnets an arc polygon.
            /// </summary>
            public class Arc : Polygon
            {
                /// <summary>
                ///     The quality
                /// </summary>
                private readonly int _quality;

                /// <summary>
                ///     The angle
                /// </summary>
                public float Angle;

                /// <summary>
                ///     The end position
                /// </summary>
                public Vector2 EndPos;

                /// <summary>
                ///     The radius
                /// </summary>
                public float Radius;

                /// <summary>
                ///     The start position
                /// </summary>
                public Vector2 StartPos;

                /// <summary>
                ///     Initializes a new instance of the <see cref="Polygon.Arc" /> class.
                /// </summary>
                /// <param name="start">The start.</param>
                /// <param name="direction">The direction.</param>
                /// <param name="angle">The angle.</param>
                /// <param name="radius">The radius.</param>
                /// <param name="quality">The quality.</param>
                public Arc(Vector3 start, Vector3 direction, float angle, float radius, int quality = 20)
                    : this(start.To2D(), direction.To2D(), angle, radius, quality)
                {
                }

                /// <summary>
                ///     Initializes a new instance of the <see cref="Polygon.Arc" /> class.
                /// </summary>
                /// <param name="start">The start.</param>
                /// <param name="direction">The direction.</param>
                /// <param name="angle">The angle.</param>
                /// <param name="radius">The radius.</param>
                /// <param name="quality">The quality.</param>
                public Arc(Vector2 start, Vector2 direction, float angle, float radius, int quality = 20)
                {
                    StartPos = start;
                    EndPos = (direction - start).Normalized();
                    Angle = angle;
                    Radius = radius;
                    _quality = quality;
                }

                /// <summary>
                ///     Updates the polygon.
                /// </summary>
                /// <param name="offset">The offset.</param>
            }

            /// <summary>
            ///     Represents a line polygon.
            /// </summary>
            public class Line : Polygon
            {
                /// <summary>
                ///     The line end
                /// </summary>
                public Vector2 LineEnd;

                /// <summary>
                ///     The line start
                /// </summary>
                public Vector2 LineStart;

                /// <summary>
                ///     Initializes a new instance of the <see cref="Polygon.Line" /> class.
                /// </summary>
                /// <param name="start">The start.</param>
                /// <param name="end">The end.</param>
                /// <param name="length">The length.</param>
                public Line(Vector3 start, Vector3 end, float length = -1) : this(start.To2D(), end.To2D(), length)
                {
                }

                /// <summary>
                ///     Initializes a new instance of the <see cref="Polygon.Line" /> class.
                /// </summary>
                /// <param name="start">The start.</param>
                /// <param name="end">The end.</param>
                /// <param name="length">The length.</param>
                public Line(Vector2 start, Vector2 end, float length = -1)
                {
                    LineStart = start;
                    LineEnd = end;
                    if (length > 0)
                    {
                        Length = length;
                    }
                }

                /// <summary>
                ///     Gets or sets the length.
                /// </summary>
                /// <value>
                ///     The length.
                /// </value>
                public float Length
                {
                    get { return LineStart.LSDistance(LineEnd); }
                    set { LineEnd = (LineEnd - LineStart).Normalized() * value + LineStart; }
                }


            }

            /// <summary>
            ///     Represents a circle polygon.
            /// </summary>
            public class Circle : Polygon
            {
                /// <summary>
                ///     The quality
                /// </summary>
                private readonly int _quality;

                /// <summary>
                ///     The center
                /// </summary>
                public Vector2 Center;

                /// <summary>
                ///     The radius
                /// </summary>
                public float Radius;

                /// <summary>
                ///     Initializes a new instance of the <see cref="Circle" /> class.
                /// </summary>
                /// <param name="center">The center.</param>
                /// <param name="radius">The radius.</param>
                /// <param name="quality">The quality.</param>
                public Circle(Vector3 center, float radius, int quality = 20) : this(center.To2D(), radius, quality)
                {
                }

                /// <summary>
                ///     Initializes a new instance of the <see cref="Circle" /> class.
                /// </summary>
                /// <param name="center">The center.</param>
                /// <param name="radius">The radius.</param>
                /// <param name="quality">The quality.</param>
                public Circle(Vector2 center, float radius, int quality = 20)
                {
                    Center = center;
                    Radius = radius;
                    _quality = quality;
                }
            }

            /// <summary>
            ///     Represents a rectangle polygon.
            /// </summary>
            public class Rectangle : Polygon
            {
                /// <summary>
                ///     The end
                /// </summary>
                public Vector2 End;

                /// <summary>
                ///     The start
                /// </summary>
                public Vector2 Start;

                /// <summary>
                ///     The width
                /// </summary>
                public float Width;

                /// <summary>
                ///     Initializes a new instance of the <see cref="Rectangle" /> class.
                /// </summary>
                /// <param name="start">The start.</param>
                /// <param name="end">The end.</param>
                /// <param name="width">The width.</param>
                public Rectangle(Vector3 start, Vector3 end, float width) : this(start.To2D(), end.To2D(), width)
                {
                }

                /// <summary>
                ///     Initializes a new instance of the <see cref="Rectangle" /> class.
                /// </summary>
                /// <param name="start">The start.</param>
                /// <param name="end">The end.</param>
                /// <param name="width">The width.</param>
                public Rectangle(Vector2 start, Vector2 end, float width)
                {
                    Start = start;
                    End = end;
                    Width = width;
                }

                /// <summary>
                ///     Gets the direction.
                /// </summary>
                /// <value>
                ///     The direction.
                /// </value>
                public Vector2 Direction
                {
                    get { return (End - Start).Normalized(); }
                }

                /// <summary>
                ///     Gets the perpendicular.
                /// </summary>
                /// <value>
                ///     The perpendicular.
                /// </value>
                public Vector2 Perpendicular
                {
                    get { return Direction.LSPerpendicular(); }
                }


            }

            /// <summary>
            ///     Represents a ring polygon.
            /// </summary>
            public class Ring : Polygon
            {
                /// <summary>
                ///     The quality
                /// </summary>
                private readonly int _quality;

                /// <summary>
                ///     The center
                /// </summary>
                public Vector2 Center;

                /// <summary>
                ///     The inner radius
                /// </summary>
                public float InnerRadius;

                /// <summary>
                ///     The outer radius
                /// </summary>
                public float OuterRadius;

                /// <summary>
                ///     Initializes a new instance of the <see cref="Polygon.Ring" /> class.
                /// </summary>
                /// <param name="center">The center.</param>
                /// <param name="innerRadius">The inner radius.</param>
                /// <param name="outerRadius">The outer radius.</param>
                /// <param name="quality">The quality.</param>
                public Ring(Vector3 center, float innerRadius, float outerRadius, int quality = 20)
                    : this(center.To2D(), innerRadius, outerRadius, quality)
                {
                }

                /// <summary>
                ///     Initializes a new instance of the <see cref="Polygon.Ring" /> class.
                /// </summary>
                /// <param name="center">The center.</param>
                /// <param name="innerRadius">The inner radius.</param>
                /// <param name="outerRadius">The outer radius.</param>
                /// <param name="quality">The quality.</param>
                public Ring(Vector2 center, float innerRadius, float outerRadius, int quality = 20)
                {
                    Center = center;
                    InnerRadius = innerRadius;
                    OuterRadius = outerRadius;
                    _quality = quality;
                }
            }

            /// <summary>
            ///     Represnets a sector polygon.
            /// </summary>
            public class Sector : Polygon
            {
                /// <summary>
                ///     The quality
                /// </summary>
                private readonly int _quality;

                /// <summary>
                ///     The angle
                /// </summary>
                public float Angle;

                /// <summary>
                ///     The center
                /// </summary>
                public Vector2 Center;

                /// <summary>
                ///     The direction
                /// </summary>
                public Vector2 Direction;

                /// <summary>
                ///     The radius
                /// </summary>
                public float Radius;

                /// <summary>
                ///     Initializes a new instance of the <see cref="Polygon.Sector" /> class.
                /// </summary>
                /// <param name="center">The center.</param>
                /// <param name="direction">The direction.</param>
                /// <param name="angle">The angle.</param>
                /// <param name="radius">The radius.</param>
                /// <param name="quality">The quality.</param>
                public Sector(Vector3 center, Vector3 direction, float angle, float radius, int quality = 20)
                    : this(center.To2D(), direction.To2D(), angle, radius, quality)
                {
                }

                /// <summary>
                ///     Initializes a new instance of the <see cref="Polygon.Sector" /> class.
                /// </summary>
                /// <param name="center">The center.</param>
                /// <param name="direction">The direction.</param>
                /// <param name="angle">The angle.</param>
                /// <param name="radius">The radius.</param>
                /// <param name="quality">The quality.</param>
                public Sector(Vector2 center, Vector2 direction, float angle, float radius, int quality = 20)
                {
                    Center = center;
                    Direction = (direction - center).Normalized();
                    Angle = angle;
                    Radius = radius;
                    _quality = quality;
                }




                /// <summary>
                ///     Rotates Line by angle/radian
                /// </summary>
                /// <param name="point1"></param>
                /// <param name="point2"></param>
                /// <param name="value"></param>
                /// <param name="radian">True for radian values, false for degree</param>
                /// <returns></returns>
                public Vector2 RotateLineFromPoint(Vector2 point1, Vector2 point2, float value, bool radian = true)
                {
                    var angle = !radian ? value * Math.PI / 180 : value;
                    var line = Vector2.Subtract(point2, point1);

                    var newline = new Vector2
                    {
                        X = (float)(line.X * Math.Cos(angle) - line.Y * Math.Sin(angle)),
                        Y = (float)(line.X * Math.Sin(angle) + line.Y * Math.Cos(angle))
                    };

                    return Vector2.Add(newline, point1);
                }
            }
        }
    }

    static class EnumerableExtensions
    {
        public static T MinOrDefault<T, R>(this IEnumerable<T> container, Func<T, R> valuingFoo) where R : IComparable
        {
            var enumerator = container.GetEnumerator();
            if (!enumerator.MoveNext())
            {
                return default(T);
            }

            var minElem = enumerator.Current;
            var minVal = valuingFoo(minElem);

            while (enumerator.MoveNext())
            {
                var currVal = valuingFoo(enumerator.Current);

                if (currVal.CompareTo(minVal) < 0)
                {
                    minVal = currVal;
                    minElem = enumerator.Current;
                }
            }

            return minElem;
        }
    }

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

    static class MenuExtensions
    {
        public static void AddStringList(this Menu m, string uniqueId, string displayName, string[] values, int defaultValue)
        {
            var mode = m.Add(uniqueId, new Slider(displayName, defaultValue, 0, values.Length - 1));
            mode.DisplayName = displayName + ": " + values[mode.CurrentValue];
            mode.OnValueChange += delegate (ValueBase<int> sender, ValueBase<int>.ValueChangeArgs args)
            {
                sender.DisplayName = displayName + ": " + values[args.NewValue];
            };
        }
    }

    static class Utility
    {
        public static bool IsWall(this Vector3 position)
        {
            return NavMesh.GetCollisionFlags(position).HasFlag(CollisionFlags.Wall);
        }

        public static bool IsWall(this Vector2 position)
        {
            return position.To3D().IsWall();
        }

        public static bool PlayerWindingUp;

        public static AIHeroClient Player
        {
            get
            {
                return ObjectManager.Player;
            }
        }
        
    }

    static class Items
    {
        public static InventorySlot GetWardSlot()
        {
            var wardIds = new[] { 3340, 3350, 3361, 3154, 2045, 2049, 2050, 2044 };
            return (from wardId in wardIds
                    where Item.CanUseItem(wardId)
                    select ObjectManager.Player.InventoryItems.FirstOrDefault(slot => slot.Id == (ItemId)wardId))
                .FirstOrDefault();
        }
    }

    public static class Orbwalking
    {
        /// <summary>
        ///     Spells that are not attacks even if they have the "attack" word in their name.
        /// </summary>
        private static readonly string[] NoAttacks =
        {
            "volleyattack", "volleyattackwithsound",
            "jarvanivcataclysmattack", "monkeykingdoubleattack", "shyvanadoubleattack", "shyvanadoubleattackdragon",
            "zyragraspingplantattack", "zyragraspingplantattack2", "zyragraspingplantattackfire",
            "zyragraspingplantattack2fire", "viktorpowertransfer", "sivirwattackbounce", "asheqattacknoonhit",
            "elisespiderlingbasicattack", "heimertyellowbasicattack", "heimertyellowbasicattack2",
            "heimertbluebasicattack", "annietibbersbasicattack", "annietibbersbasicattack2",
            "yorickdecayedghoulbasicattack", "yorickravenousghoulbasicattack", "yorickspectralghoulbasicattack",
            "malzaharvoidlingbasicattack", "malzaharvoidlingbasicattack2", "malzaharvoidlingbasicattack3",
            "kindredwolfbasicattack"
        };


        /// <summary>
        ///     Spells that are attacks even if they dont have the "attack" word in their name.
        /// </summary>
        private static readonly string[] Attacks =
        {
            "caitlynheadshotmissile", "frostarrow", "garenslash2",
            "kennenmegaproc", "masteryidoublestrike", "quinnwenhanced", "renektonexecute", "renektonsuperexecute",
            "rengarnewpassivebuffdash", "trundleq", "xenzhaothrust", "xenzhaothrust2", "xenzhaothrust3", "viktorqbuff", "lucianpassiveshot"
        };

        private static AIHeroClient Player = ObjectManager.Player;
        /// <summary>
        ///     Returns the auto-attack range of local player with respect to the target.
        /// </summary>
        /// <param name="target">The target.</param>
        /// <returns>System.Single.</returns>
        public static float GetRealAutoAttackRange(AttackableUnit target)
        {
            var result = ObjectManager.Player.GetAutoAttackRange(target);
            if (target.IsValidTarget())
            {
                return result;
            }
            return result;


        }

        /// <summary>
        ///     Returns true if the unit is melee
        /// </summary>
        /// <param name="unit">The unit.</param>
        /// <returns><c>true</c> if the specified unit is melee; otherwise, <c>false</c>.</returns>
        public static bool IsMelee(this Obj_AI_Base unit)
        {
            return unit.CombatType == GameObjectCombatType.Melee;
        }

        /// <summary>
        ///     Returns the auto-attack range of the target.
        /// </summary>
        /// <param name="target">The target.</param>
        /// <returns>System.Single.</returns>
        public static float GetAttackRange(AIHeroClient target)
        {
            var result = target.AttackRange + target.BoundingRadius;
            return result;
        }

        /// <summary>
        ///     Returns true if the target is in auto-attack range.
        /// </summary>
        /// <param name="target">The target.</param>
        /// <returns><c>true</c> if XXXX, <c>false</c> otherwise.</returns>
        public static bool InAutoAttackRange(AttackableUnit target)
        {
            if (!target.IsValidTarget())
            {
                return false;
            }
            var myRange = GetRealAutoAttackRange(target);
            return
                Vector2.DistanceSquared(
                    target is Obj_AI_Base ? (((Obj_AI_Base)target).ServerPosition).To2D() : (target.Position).To2D(),
                    (Player.ServerPosition).To2D()) <= myRange * myRange;
        }

        /// <summary>
        ///     Returns true if the spellname is an auto-attack.
        /// </summary>
        /// <param name="name">The name.</param>
        /// <returns><c>true</c> if the name is an auto attack; otherwise, <c>false</c>.</returns>
        public static bool IsAutoAttack(string name)
        {
            return (name.ToLower().Contains("attack") && !NoAttacks.Contains(name.ToLower())) || Attacks.Contains(name.ToLower());
        }
    }

    /// <summary>
    ///     Provides custom events.
    /// </summary>
    public static class CustomEvents
    {
        /// <summary>
        ///     Provides custom events regarding the game.
        /// </summary>
        public class Game
        {
            /// <summary>
            ///     The delegate for <see cref="Game.OnGameEnd" />
            /// </summary>
            /// <param name="args">The <see cref="EventArgs" /> instance containing the event data.</param>
            public delegate void OnGameEnded(EventArgs args);

            /// <summary>
            ///     The delegate for <see cref="Game.OnGameLoad" />
            /// </summary>
            /// <param name="args">The <see cref="EventArgs" /> instance containing the event data.</param>
            public delegate void OnGameLoaded(EventArgs args);

            /// <summary>
            ///     The notified subscribers
            /// </summary>
            private static readonly List<Delegate> NotifiedSubscribers = new List<Delegate>();

            /// <summary>
            ///     The nexus list
            /// </summary>
            private static readonly List<Obj_HQ> NexusList = new List<Obj_HQ>();

            /// <summary>
            ///     The end game called
            /// </summary>
            private static bool _endGameCalled;

            /// <summary>
            ///     Initializes static members of the <see cref="Game" /> class.
            /// </summary>
            static Game()
            {
                Core.DelayAction(Initialize, 0);
            }

            /// <summary>
            ///     Initializes this instance.
            /// </summary>
            public static void Initialize()
            {
                foreach (var hq in ObjectManager.Get<Obj_HQ>().Where(hq => hq.IsValid))
                {
                    NexusList.Add(hq);
                }

                if (EloBuddy.Game.Mode == GameMode.Running)
                {
                    //Otherwise the .ctor didn't return yet and no callback will occur
                    Core.DelayAction(() => { Game_OnGameStart(new EventArgs()); }, 500);
                }
                else
                {
                    EloBuddy.Game.OnLoad += Game_OnGameStart;
                }
            }

            /// <summary>
            ///     Fired when the game updates.
            /// </summary>
            /// <param name="args">The <see cref="EventArgs" /> instance containing the event data.</param>
            private static void Game_OnGameUpdate(EventArgs args)
            {
                if (OnGameLoad != null)
                {
                    foreach (var subscriber in OnGameLoad.GetInvocationList()
                        .Where(s => !NotifiedSubscribers.Contains(s)))
                    {
                        NotifiedSubscribers.Add(subscriber);
                        try
                        {
                            subscriber.DynamicInvoke(new EventArgs());
                        }
                        catch (Exception ex)
                        {
                            Console.WriteLine(ex);
                        }
                    }
                }

                if (NexusList.Count == 0 || _endGameCalled)
                {
                    return;
                }

                foreach (var nexus in NexusList)
                {
                    if (nexus != null && nexus.IsValid && nexus.Health <= 0)
                    {
                        if (OnGameEnd != null)
                        {
                            OnGameEnd(new EventArgs());
                            _endGameCalled = true; // Don't spam the event.
                        }
                    }
                }
            }


            /// <summary>
            ///     Occurs when the game loads. This will be fired if the game is already loaded.
            /// </summary>
            public static event OnGameLoaded OnGameLoad;


            /// <summary>
            ///     Occurs when the game ends. This is meant as a better replacement to <see cref="LeagueSharp.Game.OnEnd" />.
            /// </summary>
            public static event OnGameEnded OnGameEnd;

            /// <summary>
            ///     Fired when the game is started.
            /// </summary>
            /// <param name="args">The <see cref="EventArgs" /> instance containing the event data.</param>
            private static void Game_OnGameStart(EventArgs args)
            {
                EloBuddy.Game.OnUpdate += Game_OnGameUpdate;

                if (OnGameLoad != null)
                {
                    foreach (var subscriber in OnGameLoad.GetInvocationList()
                        .Where(s => !NotifiedSubscribers.Contains(s)))
                    {
                        NotifiedSubscribers.Add(subscriber);
                        try
                        {
                            subscriber.DynamicInvoke(new EventArgs());
                        }
                        catch (Exception ex)
                        {
                            Console.WriteLine(ex);
                        }
                    }
                }
            }
        }

        /// <summary>
        ///     Provides custom events regarding units.
        /// </summary>
        public class Unit
        {
            /// <summary>
            ///     The delegate for <see cref="Unit.OnDash" />
            /// </summary>
            /// <param name="sender">The sender.</param>
            /// <param name="args">The arguments.</param>
            public delegate void OnDashed(Obj_AI_Base sender, Dash.DashEventArgs args);

            /// <summary>
            ///     The delegate for <see cref="Unit.OnLevelUp" />
            /// </summary>
            /// <param name="sender">The sender.</param>
            /// <param name="args">The <see cref="OnLevelUpEventArgs" /> instance containing the event data.</param>
            public delegate void OnLeveledUp(Obj_AI_Base sender, OnLevelUpEventArgs args);

            /// <summary>
            ///     The delegate for <see cref="Unit.OnLevelUpSpell" />
            /// </summary>
            /// <param name="sender">The sender.</param>
            /// <param name="args">The <see cref="OnLevelUpSpellEventArgs" /> instance containing the event data.</param>
            public delegate void OnLeveledUpSpell(Obj_AI_Base sender, OnLevelUpSpellEventArgs args);

            /// <summary>
            /// Occurs when a unit levels up.
            /// </summary>
            public static event OnLeveledUp OnLevelUp;

            /// <summary>
            ///     Initializes static members of the <see cref="Unit" /> class.
            /// </summary>
            static Unit()
            {
                EloBuddy.Game.OnProcessPacket += PacketHandler;

                //Initializes ondash class:
                ObjectManager.Player.IsDashing();
            }

            /// <summary>
            ///     Handles packets.
            /// </summary>
            /// <param name="args">The <see cref="GamePacketEventArgs" /> instance containing the event data.</param>
            private static void PacketHandler(GamePacketEventArgs args)
            {
            }

            /// <summary>
            ///     Occurs when a unit dashes.
            /// </summary>
            public static event OnDashed OnDash;

            /// <summary>
            ///     Triggers the on dash.
            /// </summary>
            /// <param name="sender">The sender.</param>
            /// <param name="args">The arguments.</param>
            public static void TriggerOnDash(Obj_AI_Base sender, Dash.DashEventArgs args)
            {
                var dashHandler = OnDash;
                if (dashHandler != null)
                {
                    dashHandler(sender, args);
                }
            }

            /// <summary>
            ///     The event arguments for the <see cref="Unit.OnLevelUp" /> event.
            /// </summary>
            public class OnLevelUpEventArgs : EventArgs
            {
                /// <summary>
                ///     The new level
                /// </summary>
                public int NewLevel;

                /// <summary>
                ///     The remaining points
                /// </summary>
                public int RemainingPoints;
            }

            /// <summary>
            ///     The event arguments for the <see cref="Unit.OnLevelUpSpell" /> event.
            /// </summary>
            public class OnLevelUpSpellEventArgs : EventArgs
            {
                /// <summary>
                ///     The remainingpoints
                /// </summary>
                public int Remainingpoints;

                /// <summary>
                ///     The spell identifier
                /// </summary>
                public int SpellId;

                /// <summary>
                ///     The spell level
                /// </summary>
                public int SpellLevel;

                /// <summary>
                ///     Initializes a new instance of the <see cref="OnLevelUpSpellEventArgs" /> class.
                /// </summary>
                internal OnLevelUpSpellEventArgs()
                {
                }
            }
        }
    }

    /// <summary>
    ///     Gets information about dashes, and provides events.
    /// </summary>
    
    public static class DelayAction
    {
        #region Static Fields

        public static List<Action> ActionList = new List<Action>();

        #endregion

        #region Constructors and Destructors

        static DelayAction()
        {
            Game.OnUpdate += GameOnOnGameUpdate;
        }

        #endregion

        #region Delegates

        public delegate void Callback();

        #endregion

        #region Public Methods and Operators

        public static void Add(int time, Callback func)
        {
            var action = new Action(time, func);
            ActionList.Add(action);
        }

        #endregion

        #region Methods

        private static void GameOnOnGameUpdate(EventArgs args)
        {
            for (var i = ActionList.Count - 1; i >= 0; i--)
            {
                if (ActionList[i].Time <= Utils.GameTimeTickCount)
                {
                    try
                    {
                        if (ActionList[i].CallbackObject != null)
                        {
                            ActionList[i].CallbackObject();
                            //Will somehow result in calling ALL non-internal marked classes of the called assembly and causes NullReferenceExceptions.
                        }
                    }
                    catch (Exception)
                    {
                        // ignored
                    }

                    ActionList.RemoveAt(i);
                }
            }
        }

        #endregion

        public struct Action
        {
            #region Fields

            public Callback CallbackObject;

            public int Time;

            #endregion

            #region Constructors and Destructors

            public Action(int time, Callback callback)
            {
                this.Time = time + Utils.GameTimeTickCount;
                this.CallbackObject = callback;
            }

            #endregion
        }
    }

    /// <summary>
    ///     Provides cached heroes.
    /// </summary>
    public class HeroManager
    {
        /// <summary>
        ///     Initializes static members of the <see cref="HeroManager" /> class.
        /// </summary>
        static HeroManager()
        {
            if (Game.Mode == GameMode.Running)
            {
                Game_OnStart(new EventArgs());
            }
            Game.OnLoad += Game_OnStart;
        }

        /// <summary>
        ///     Gets all heroes.
        /// </summary>
        /// <value>
        ///     All heroes.
        /// </value>
        public static List<AIHeroClient> AllHeroes { get; private set; }

        /// <summary>
        ///     Gets the allies.
        /// </summary>
        /// <value>
        ///     The allies.
        /// </value>
        public static List<AIHeroClient> Allies { get; private set; }

        /// <summary>
        ///     Gets the enemies.
        /// </summary>
        /// <value>
        ///     The enemies.
        /// </value>
        public static List<AIHeroClient> Enemies { get; private set; }

        /// <summary>
        ///     Gets the Local Player
        /// </summary>
        public static AIHeroClient Player { get; private set; }

        /// <summary>
        ///     Fired when the game starts.
        /// </summary>
        /// <param name="args">The <see cref="EventArgs" /> instance containing the event data.</param>
        private static void Game_OnStart(EventArgs args)
        {
            AllHeroes = ObjectManager.Get<AIHeroClient>().ToList();
            Allies = AllHeroes.FindAll(o => o.IsAlly);
            Enemies = AllHeroes.FindAll(o => o.IsEnemy);
            Player = AllHeroes.Find(x => x.IsMe);
        }
    }
   
    /// <summary>
    ///     The render class allows you to draw stuff using SharpDX easier.
    /// </summary>
    public static class Render
    {
        /// <summary>
        ///     Gets the device.
        /// </summary>
        /// <value>The device.</value>
        public static Device Device
        {
            get { return Drawing.Direct3DDevice; }
        }

        /// <summary>
        ///     A base class that renders objects.
        /// </summary>
        public class RenderObject : IDisposable
        {
            /// <summary>
            ///     Delegate that gets if the object is visible.
            /// </summary>
            /// <param name="sender">The sender.</param>
            /// <returns><c>true</c> if the object is visible, <c>false</c> otherwise.</returns>
            public delegate bool VisibleConditionDelegate(RenderObject sender);

            /// <summary>
            ///     <c>true</c> if the render object is visible
            /// </summary>
            private bool _visible = true;

            /// <summary>
            ///     The layer
            /// </summary>
            public float Layer;

            /// <summary>
            ///     The visible condition delegate.
            /// </summary>
            //public VisibleConditionDelegate VisibleCondition;
            public VisibleConditionDelegate VisibleCondition;

            /// <summary>
            ///     Gets or sets a value indicating whether this <see cref="RenderObject" /> is visible.
            /// </summary>
            /// <value><c>true</c> if visible; otherwise, <c>false</c>.</value>
            public bool Visible
            {
                get { return VisibleCondition != null ? VisibleCondition(this) : _visible; }
                set { _visible = value; }
            }

            /// <summary>
            ///     Performs application-defined tasks associated with freeing, releasing, or resetting unmanaged resources.
            /// </summary>
            public virtual void Dispose()
            {
            }

            /// <summary>
            ///     Called when the render object is drawn.
            /// </summary>
            public virtual void OnDraw()
            {
            }

            /// <summary>
            ///     Called when the scene has ended..
            /// </summary>
            public virtual void OnEndScene()
            {
            }

            /// <summary>
            ///     Called before the DirectX device is reset.
            /// </summary>
            public virtual void OnPreReset()
            {
            }

            /// <summary>
            ///     Called after the DirectX device is reset.
            /// </summary>
            public virtual void OnPostReset()
            {
            }

            /// <summary>
            ///     Determines whether this instace has a valid layer.
            /// </summary>
            /// <returns><c>true</c> if has a valid layer; otherwise, <c>false</c>.</returns>
            public bool HasValidLayer()
            {
                return Layer >= -5 && Layer <= 5;
            }
        }

        /// <summary>
        ///     Draws circles.
        /// </summary>
        public class Circle : RenderObject
        {
            
            /// <summary>
            ///     The vertices
            /// </summary>
            private static VertexBuffer _vertices;

            /// <summary>
            ///     The vertex elements
            /// </summary>
            private static VertexElement[] _vertexElements;

            /// <summary>
            ///     The vertex declaration
            /// </summary>
            private static VertexDeclaration _vertexDeclaration;

            /// <summary>
            ///     The sprite effect
            /// </summary>
            private static Effect _effect;

            /// <summary>
            ///     The technique
            /// </summary>
            private static EffectHandle _technique;

            /// <summary>
            ///     <c>true</c> if this instanced initialized.
            /// </summary>
            private static bool _initialized;

            /// <summary>
            ///     The offset
            /// </summary>
            private static Vector3 _offset = new Vector3(0, 0, 0);

            /// <summary>
            ///     Initializes a new instance of the <see cref="Circle" /> class.
            /// </summary>
            /// <param name="unit">The unit.</param>
            /// <param name="radius">The radius.</param>
            /// <param name="color">The color.</param>
            /// <param name="width">The width.</param>
            /// <param name="zDeep">if set to <c>true</c> [z deep].</param>
            public Circle(GameObject unit, float radius, Color color, int width = 1, bool zDeep = false)
            {
                Color = color;
                Unit = unit;
                Radius = radius;
                Width = width;
                ZDeep = zDeep;
            }

            /// <summary>
            ///     Initializes a new instance of the <see cref="Circle" /> class.
            /// </summary>
            /// <param name="unit">The unit.</param>
            /// <param name="offset">The offset.</param>
            /// <param name="radius">The radius.</param>
            /// <param name="color">The color.</param>
            /// <param name="width">The width.</param>
            /// <param name="zDeep">if set to <c>true</c> [z deep].</param>
            public Circle(GameObject unit, Vector3 offset, float radius, Color color, int width = 1, bool zDeep = false)
            {
                Color = color;
                Unit = unit;
                Radius = radius;
                Width = width;
                ZDeep = zDeep;
                Offset = offset;
            }

            /// <summary>
            ///     Initializes a new instance of the <see cref="Circle" /> class.
            /// </summary>
            /// <param name="position">The position.</param>
            /// <param name="offset">The offset.</param>
            /// <param name="radius">The radius.</param>
            /// <param name="color">The color.</param>
            /// <param name="width">The width.</param>
            /// <param name="zDeep">if set to <c>true</c> [z deep].</param>
            public Circle(Vector3 position, Vector3 offset, float radius, Color color, int width = 1, bool zDeep = false)
            {
                Color = color;
                Position = position;
                Radius = radius;
                Width = width;
                ZDeep = zDeep;
                Offset = offset;
            }

            /// <summary>
            ///     Initializes a new instance of the <see cref="Circle" /> class.
            /// </summary>
            /// <param name="position">The position.</param>
            /// <param name="radius">The radius.</param>
            /// <param name="color">The color.</param>
            /// <param name="width">The width.</param>
            /// <param name="zDeep">if set to <c>true</c> [z deep].</param>
            public Circle(Vector3 position, float radius, Color color, int width = 1, bool zDeep = false)
            {
                Color = color;
                Position = position;
                Radius = radius;
                Width = width;
                ZDeep = zDeep;
            }

            /// <summary>
            ///     Gets or sets the position.
            /// </summary>
            /// <value>The position.</value>
            public Vector3 Position { get; set; }

            /// <summary>
            ///     Gets or sets the unit.
            /// </summary>
            /// <value>The unit.</value>
            public GameObject Unit { get; set; }

            /// <summary>
            ///     Gets or sets the radius.
            /// </summary>
            /// <value>The radius.</value>
            public float Radius { get; set; }

            /// <summary>
            ///     Gets or sets the color.
            /// </summary>
            /// <value>The color.</value>
            public Color Color { get; set; }

            /// <summary>
            ///     Gets or sets the width.
            /// </summary>
            /// <value>The width.</value>
            public int Width { get; set; }

            /// <summary>
            ///     Gets or sets a value indicating whether to enable depth buffering.
            /// </summary>
            /// <value><c>true</c> if depth buffering enabled; otherwise, <c>false</c>.</value>
            public bool ZDeep { get; set; }

            /// <summary>
            ///     Gets or sets the offset.
            /// </summary>
            /// <value>The offset.</value>
            public Vector3 Offset
            {
                get { return _offset; }
                set { _offset = value; }
            }

            /// <summary>
            ///     Called when the circle is drawn.
            /// </summary>
            public override void OnDraw()
            {
                try
                {
                    if (Unit != null && Unit.IsValid)
                    {
                        DrawCircle(Unit.Position + _offset, Radius, Color, Width, ZDeep);
                    }
                    else if ((Position + _offset).To2D().IsValid())
                    {
                        DrawCircle(Position + _offset, Radius, Color, Width, ZDeep);
                    }
                }
                catch (Exception e)
                {
                    Console.WriteLine(@"Common.Render.Circle.OnEndScene: " + e);
                }
            }

            /// <summary>
            ///     Creates the vertexes.
            /// </summary>
            public static void CreateVertexes()
            {
                const float x = 6000f;
                _vertices = new VertexBuffer(
                    Device, Utilities.SizeOf<Vector4>() * 2 * 6, Usage.WriteOnly, VertexFormat.None, Pool.Managed);

                _vertices.Lock(0, 0, LockFlags.None).WriteRange(
                    new[]
                    {
                        //T1
                        new Vector4(-x, 0f, -x, 1.0f), new Vector4(),
                        new Vector4(-x, 0f, x, 1.0f), new Vector4(),
                        new Vector4(x, 0f, -x, 1.0f), new Vector4(),

                        //T2
                        new Vector4(-x, 0f, x, 1.0f), new Vector4(),
                        new Vector4(x, 0f, x, 1.0f), new Vector4(),
                        new Vector4(x, 0f, -x, 1.0f), new Vector4()
                    });
                _vertices.Unlock();

                _vertexElements = new[]
                {
                    new VertexElement(
                        0, 0, DeclarationType.Float4, DeclarationMethod.Default, DeclarationUsage.Position, 0),
                    new VertexElement(
                        0, 16, DeclarationType.Float4, DeclarationMethod.Default, DeclarationUsage.Color, 0),
                    VertexElement.VertexDeclarationEnd
                };

                _vertexDeclaration = new VertexDeclaration(Device, _vertexElements);

                #region Effect

                try
                {
                    /*   
                    _effect = Effect.FromString(Device, @"
                    struct VS_S
                     {
                         float4 Position : POSITION;
                         float4 Color : COLOR0;
                         float4 Position3D : TEXCOORD0;
                     };

                     float4x4 ProjectionMatrix;
                     float4 CircleColor;
                     float Radius;
                     float Border;
                     bool zEnabled;
                     VS_S VS( VS_S input )
                     {
                         VS_S output = (VS_S)0;

                         output.Position = mul(input.Position, ProjectionMatrix);
                         output.Color = input.Color;
                         output.Position3D = input.Position;
                         return output;
                     }

                     float4 PS( VS_S input ) : COLOR
                     {
                         VS_S output = (VS_S)0;
                         output = input;

                         float4 v = output.Position3D; 
                         float distance = Radius - sqrt(v.x * v.x + v.z*v.z); // Distance to the circle arc.

                         output.Color.x = CircleColor.x;
                         output.Color.y = CircleColor.y;
                         output.Color.z = CircleColor.z;

                         if(distance < Border && distance > -Border)
                         {
                             output.Color.w = (CircleColor.w - CircleColor.w * abs(distance * 1.75 / Border));
                         }
                         else
                         {
                             output.Color.w = 0;
                         }

                         if(Border < 1 && distance >= 0)
                         {
                             output.Color.w = CircleColor.w;
                         }

                         return output.Color;
                     }

                     technique Main {
                         pass P0 {
                             ZEnable = zEnabled;
                             AlphaBlendEnable = TRUE;
                             DestBlend = INVSRCALPHA;
                             SrcBlend = SRCALPHA;
                             VertexShader = compile vs_2_0 VS();
                             PixelShader  = compile ps_2_0 PS();
                         }
                     }", ShaderFlags.None);
                    */
                    var compiledEffect = new byte[]
                    {
                        0x01, 0x09, 0xFF, 0xFE, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
                        0x02, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x04, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x50, 0x72, 0x6F, 0x6A,
                        0x65, 0x63, 0x74, 0x69, 0x6F, 0x6E, 0x4D, 0x61, 0x74, 0x72, 0x69, 0x78, 0x00, 0x00, 0x00, 0x00,
                        0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xA4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00,
                        0x43, 0x69, 0x72, 0x63, 0x6C, 0x65, 0x43, 0x6F, 0x6C, 0x6F, 0x72, 0x00, 0x03, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0xD4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00,
                        0x52, 0x61, 0x64, 0x69, 0x75, 0x73, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
                        0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x42, 0x6F, 0x72, 0x64,
                        0x65, 0x72, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2C, 0x01, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x7A, 0x45, 0x6E, 0x61, 0x62, 0x6C, 0x65, 0x64,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
                        0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
                        0x01, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
                        0x01, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
                        0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
                        0x0F, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x50, 0x30, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00,
                        0x4D, 0x61, 0x69, 0x6E, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
                        0x03, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x94, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB4, 0x00, 0x00, 0x00, 0xD0, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x01, 0x00, 0x00, 0x28, 0x01, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF4, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x01, 0x00, 0x00, 0x00, 0xEC, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x01, 0x00, 0x00, 0x3C, 0x01, 0x00, 0x00,
                        0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x01, 0x00, 0x00, 0x5C, 0x01, 0x00, 0x00,
                        0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x7C, 0x01, 0x00, 0x00,
                        0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x01, 0x00, 0x00, 0x9C, 0x01, 0x00, 0x00,
                        0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x01, 0x00, 0x00, 0xBC, 0x01, 0x00, 0x00,
                        0x93, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD8, 0x01, 0x00, 0x00, 0xD4, 0x01, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0xFF, 0xFF, 0xFF, 0xFF, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x04, 0x00, 0x00,
                        0x00, 0x02, 0xFF, 0xFF, 0xFE, 0xFF, 0x38, 0x00, 0x43, 0x54, 0x41, 0x42, 0x1C, 0x00, 0x00, 0x00,
                        0xAA, 0x00, 0x00, 0x00, 0x00, 0x02, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x20, 0xA3, 0x00, 0x00, 0x00, 0x58, 0x00, 0x00, 0x00, 0x02, 0x00, 0x05, 0x00,
                        0x01, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00,
                        0x02, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x8C, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00,
                        0x9C, 0x00, 0x00, 0x00, 0x02, 0x00, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
                        0x70, 0x00, 0x00, 0x00, 0x42, 0x6F, 0x72, 0x64, 0x65, 0x72, 0x00, 0xAB, 0x00, 0x00, 0x03, 0x00,
                        0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x43, 0x69, 0x72, 0x63,
                        0x6C, 0x65, 0x43, 0x6F, 0x6C, 0x6F, 0x72, 0x00, 0x01, 0x00, 0x03, 0x00, 0x01, 0x00, 0x04, 0x00,
                        0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x52, 0x61, 0x64, 0x69, 0x75, 0x73, 0x00, 0x70,
                        0x73, 0x5F, 0x32, 0x5F, 0x30, 0x00, 0x4D, 0x69, 0x63, 0x72, 0x6F, 0x73, 0x6F, 0x66, 0x74, 0x20,
                        0x28, 0x52, 0x29, 0x20, 0x48, 0x4C, 0x53, 0x4C, 0x20, 0x53, 0x68, 0x61, 0x64, 0x65, 0x72, 0x20,
                        0x43, 0x6F, 0x6D, 0x70, 0x69, 0x6C, 0x65, 0x72, 0x20, 0x39, 0x2E, 0x32, 0x39, 0x2E, 0x39, 0x35,
                        0x32, 0x2E, 0x33, 0x31, 0x31, 0x31, 0x00, 0xAB, 0xFE, 0xFF, 0x7C, 0x00, 0x50, 0x52, 0x45, 0x53,
                        0x01, 0x02, 0x58, 0x46, 0xFE, 0xFF, 0x30, 0x00, 0x43, 0x54, 0x41, 0x42, 0x1C, 0x00, 0x00, 0x00,
                        0x8B, 0x00, 0x00, 0x00, 0x01, 0x02, 0x58, 0x46, 0x02, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x00, 0x00,
                        0x00, 0x01, 0x00, 0x20, 0x88, 0x00, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x02, 0x00, 0x01, 0x00,
                        0x01, 0x00, 0x00, 0x00, 0x4C, 0x00, 0x00, 0x00, 0x5C, 0x00, 0x00, 0x00, 0x6C, 0x00, 0x00, 0x00,
                        0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x5C, 0x00, 0x00, 0x00,
                        0x42, 0x6F, 0x72, 0x64, 0x65, 0x72, 0x00, 0xAB, 0x00, 0x00, 0x03, 0x00, 0x01, 0x00, 0x01, 0x00,
                        0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x43, 0x69, 0x72, 0x63, 0x6C, 0x65, 0x43, 0x6F,
                        0x6C, 0x6F, 0x72, 0x00, 0x01, 0x00, 0x03, 0x00, 0x01, 0x00, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x74, 0x78, 0x00, 0x4D, 0x69, 0x63, 0x72, 0x6F, 0x73, 0x6F, 0x66, 0x74,
                        0x20, 0x28, 0x52, 0x29, 0x20, 0x48, 0x4C, 0x53, 0x4C, 0x20, 0x53, 0x68, 0x61, 0x64, 0x65, 0x72,
                        0x20, 0x43, 0x6F, 0x6D, 0x70, 0x69, 0x6C, 0x65, 0x72, 0x20, 0x39, 0x2E, 0x32, 0x39, 0x2E, 0x39,
                        0x35, 0x32, 0x2E, 0x33, 0x31, 0x31, 0x31, 0x00, 0xFE, 0xFF, 0x0C, 0x00, 0x50, 0x52, 0x53, 0x49,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0x1A, 0x00,
                        0x43, 0x4C, 0x49, 0x54, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xBF,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0x1F, 0x00, 0x46, 0x58, 0x4C, 0x43,
                        0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x30, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x02, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x40, 0xA0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x02, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
                        0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
                        0x03, 0x00, 0x00, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
                        0xF0, 0xF0, 0xF0, 0xF0, 0x0F, 0x0F, 0x0F, 0x0F, 0xFF, 0xFF, 0x00, 0x00, 0x51, 0x00, 0x00, 0x05,
                        0x06, 0x00, 0x0F, 0xA0, 0x00, 0x00, 0xE0, 0x3F, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80, 0xBF,
                        0x00, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x07, 0xB0,
                        0x05, 0x00, 0x00, 0x03, 0x00, 0x00, 0x08, 0x80, 0x00, 0x00, 0xAA, 0xB0, 0x00, 0x00, 0xAA, 0xB0,
                        0x04, 0x00, 0x00, 0x04, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0xB0, 0x00, 0x00, 0x00, 0xB0,
                        0x00, 0x00, 0xFF, 0x80, 0x07, 0x00, 0x00, 0x02, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x80,
                        0x06, 0x00, 0x00, 0x02, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x80, 0x02, 0x00, 0x00, 0x03,
                        0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x81, 0x04, 0x00, 0x00, 0xA0, 0x02, 0x00, 0x00, 0x03,
                        0x00, 0x00, 0x02, 0x80, 0x00, 0x00, 0x00, 0x81, 0x05, 0x00, 0x00, 0xA1, 0x58, 0x00, 0x00, 0x04,
                        0x00, 0x00, 0x02, 0x80, 0x00, 0x00, 0x55, 0x80, 0x06, 0x00, 0x55, 0xA0, 0x06, 0x00, 0xAA, 0xA0,
                        0x02, 0x00, 0x00, 0x03, 0x00, 0x00, 0x04, 0x80, 0x00, 0x00, 0x00, 0x80, 0x05, 0x00, 0x00, 0xA1,
                        0x58, 0x00, 0x00, 0x04, 0x00, 0x00, 0x02, 0x80, 0x00, 0x00, 0xAA, 0x80, 0x06, 0x00, 0x55, 0xA0,
                        0x00, 0x00, 0x55, 0x80, 0x05, 0x00, 0x00, 0x03, 0x00, 0x00, 0x04, 0x80, 0x00, 0x00, 0x00, 0x80,
                        0x06, 0x00, 0x00, 0xA0, 0x58, 0x00, 0x00, 0x04, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x80,
                        0x06, 0x00, 0xAA, 0xA0, 0x06, 0x00, 0x55, 0xA0, 0x01, 0x00, 0x00, 0x02, 0x00, 0x00, 0x08, 0x80,
                        0x06, 0x00, 0x55, 0xA0, 0x58, 0x00, 0x00, 0x04, 0x00, 0x00, 0x01, 0x80, 0x01, 0x00, 0x00, 0xA0,
                        0x00, 0x00, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x80, 0x05, 0x00, 0x00, 0x03, 0x00, 0x00, 0x04, 0x80,
                        0x00, 0x00, 0xAA, 0x80, 0x00, 0x00, 0x00, 0xA0, 0x23, 0x00, 0x00, 0x02, 0x00, 0x00, 0x04, 0x80,
                        0x00, 0x00, 0xAA, 0x80, 0x04, 0x00, 0x00, 0x04, 0x00, 0x00, 0x04, 0x80, 0x03, 0x00, 0xFF, 0xA0,
                        0x00, 0x00, 0xAA, 0x81, 0x03, 0x00, 0xFF, 0xA0, 0x58, 0x00, 0x00, 0x04, 0x00, 0x00, 0x02, 0x80,
                        0x00, 0x00, 0x55, 0x80, 0x06, 0x00, 0xFF, 0xA0, 0x00, 0x00, 0xAA, 0x80, 0x58, 0x00, 0x00, 0x04,
                        0x00, 0x00, 0x08, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x55, 0x80, 0x03, 0x00, 0xFF, 0xA0,
                        0x01, 0x00, 0x00, 0x02, 0x00, 0x00, 0x07, 0x80, 0x02, 0x00, 0xE4, 0xA0, 0x01, 0x00, 0x00, 0x02,
                        0x00, 0x08, 0x0F, 0x80, 0x00, 0x00, 0xE4, 0x80, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x4C, 0x01, 0x00, 0x00, 0x00, 0x02, 0xFE, 0xFF, 0xFE, 0xFF, 0x34, 0x00, 0x43, 0x54, 0x41, 0x42,
                        0x1C, 0x00, 0x00, 0x00, 0x9B, 0x00, 0x00, 0x00, 0x00, 0x02, 0xFE, 0xFF, 0x01, 0x00, 0x00, 0x00,
                        0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x94, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00,
                        0x02, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x54, 0x00, 0x00, 0x00,
                        0x50, 0x72, 0x6F, 0x6A, 0x65, 0x63, 0x74, 0x69, 0x6F, 0x6E, 0x4D, 0x61, 0x74, 0x72, 0x69, 0x78,
                        0x00, 0xAB, 0xAB, 0xAB, 0x03, 0x00, 0x03, 0x00, 0x04, 0x00, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x76, 0x73, 0x5F, 0x32, 0x5F, 0x30, 0x00, 0x4D, 0x69, 0x63, 0x72, 0x6F,
                        0x73, 0x6F, 0x66, 0x74, 0x20, 0x28, 0x52, 0x29, 0x20, 0x48, 0x4C, 0x53, 0x4C, 0x20, 0x53, 0x68,
                        0x61, 0x64, 0x65, 0x72, 0x20, 0x43, 0x6F, 0x6D, 0x70, 0x69, 0x6C, 0x65, 0x72, 0x20, 0x39, 0x2E,
                        0x32, 0x39, 0x2E, 0x39, 0x35, 0x32, 0x2E, 0x33, 0x31, 0x31, 0x31, 0x00, 0x1F, 0x00, 0x00, 0x02,
                        0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x0F, 0x90, 0x1F, 0x00, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x80,
                        0x01, 0x00, 0x0F, 0x90, 0x09, 0x00, 0x00, 0x03, 0x00, 0x00, 0x01, 0xC0, 0x00, 0x00, 0xE4, 0x90,
                        0x00, 0x00, 0xE4, 0xA0, 0x09, 0x00, 0x00, 0x03, 0x00, 0x00, 0x02, 0xC0, 0x00, 0x00, 0xE4, 0x90,
                        0x01, 0x00, 0xE4, 0xA0, 0x09, 0x00, 0x00, 0x03, 0x00, 0x00, 0x04, 0xC0, 0x00, 0x00, 0xE4, 0x90,
                        0x02, 0x00, 0xE4, 0xA0, 0x09, 0x00, 0x00, 0x03, 0x00, 0x00, 0x08, 0xC0, 0x00, 0x00, 0xE4, 0x90,
                        0x03, 0x00, 0xE4, 0xA0, 0x01, 0x00, 0x00, 0x02, 0x00, 0x00, 0x0F, 0xD0, 0x01, 0x00, 0xE4, 0x90,
                        0x01, 0x00, 0x00, 0x02, 0x00, 0x00, 0x0F, 0xE0, 0x00, 0x00, 0xE4, 0x90, 0xFF, 0xFF, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x02, 0x58, 0x46, 0xFE, 0xFF, 0x25, 0x00,
                        0x43, 0x54, 0x41, 0x42, 0x1C, 0x00, 0x00, 0x00, 0x5F, 0x00, 0x00, 0x00, 0x00, 0x02, 0x58, 0x46,
                        0x01, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x20, 0x5C, 0x00, 0x00, 0x00,
                        0x30, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x00,
                        0x4C, 0x00, 0x00, 0x00, 0x7A, 0x45, 0x6E, 0x61, 0x62, 0x6C, 0x65, 0x64, 0x00, 0xAB, 0xAB, 0xAB,
                        0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x74, 0x78, 0x00, 0x4D, 0x69, 0x63, 0x72, 0x6F, 0x73, 0x6F, 0x66, 0x74, 0x20, 0x28, 0x52, 0x29,
                        0x20, 0x48, 0x4C, 0x53, 0x4C, 0x20, 0x53, 0x68, 0x61, 0x64, 0x65, 0x72, 0x20, 0x43, 0x6F, 0x6D,
                        0x70, 0x69, 0x6C, 0x65, 0x72, 0x20, 0x39, 0x2E, 0x32, 0x39, 0x2E, 0x39, 0x35, 0x32, 0x2E, 0x33,
                        0x31, 0x31, 0x31, 0x00, 0xFE, 0xFF, 0x02, 0x00, 0x43, 0x4C, 0x49, 0x54, 0x00, 0x00, 0x00, 0x00,
                        0xFE, 0xFF, 0x0C, 0x00, 0x46, 0x58, 0x4C, 0x43, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x10,
                        0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xF0, 0xF0, 0xF0,
                        0x0F, 0x0F, 0x0F, 0x0F, 0xFF, 0xFF, 0x00, 0x00
                    };
                    _effect = Effect.FromMemory(Device, compiledEffect, ShaderFlags.None);
                }
                catch (Exception e)
                {
                    Console.WriteLine(e);
                    return;
                }

                #endregion

                _technique = _effect.GetTechnique(0);

                if (!_initialized)
                {
                    _initialized = true;
                    Drawing.OnPreReset += OnPreReset;
                    Drawing.OnPreReset += OnPostReset;
                    AppDomain.CurrentDomain.DomainUnload += Dispose;
                }
            }

            /// <summary>
            ///     Handles the <see cref="E:PreReset" /> event.
            /// </summary>
            /// <param name="args">The <see cref="EventArgs" /> instance containing the event data.</param>
            private static void OnPreReset(EventArgs args)
            {
                if (_effect != null && !_effect.IsDisposed)
                {
                    _effect.OnLostDevice();
                }
            }

            /// <summary>
            ///     Handles the <see cref="E:PostReset" /> event.
            /// </summary>
            /// <param name="args">The <see cref="EventArgs" /> instance containing the event data.</param>
            private static void OnPostReset(EventArgs args)
            {
                if (_effect != null && !_effect.IsDisposed)
                {
                    _effect.OnResetDevice();
                }
            }

            /// <summary>
            ///     Disposes the circle.
            /// </summary>
            /// <param name="sender">The sender.</param>
            /// <param name="e">The <see cref="EventArgs" /> instance containing the event data.</param>
            private static void Dispose(object sender, EventArgs e)
            {
                if (_effect != null && !_effect.IsDisposed)
                {
                    _effect.Dispose();
                }

                if (_vertices != null && !_vertices.IsDisposed)
                {
                    _vertices.Dispose();
                }

                if (_vertexDeclaration != null && !_vertexDeclaration.IsDisposed)
                {
                    _vertexDeclaration.Dispose();
                }
            }

            /// <summary>
            ///     Draws the circle.
            /// </summary>
            /// <param name="position">The position.</param>
            /// <param name="radius">The radius.</param>
            /// <param name="color">The color.</param>
            /// <param name="width">The width.</param>
            /// <param name="zDeep">if set to <c>true</c> the circle will be drawn with depth buffering.</param>
            public static void DrawCircle(Vector3 position, float radius, Color color, int width = 5, bool zDeep = false)
            {

                try
                {
                    if (Device == null || Device.IsDisposed)
                    {
                        return;
                    }

                    if (_vertices == null)
                    {
                        CreateVertexes();
                    }

                    if (_vertices == null || _vertices.IsDisposed || _vertexDeclaration.IsDisposed || _effect.IsDisposed ||
                        _technique.IsDisposed)
                    {
                        return;
                    }

                    var olddec = Device.VertexDeclaration;

                    _effect.Technique = _technique;

                    _effect.Begin();
                    _effect.BeginPass(0);
                    _effect.SetValue(
                        "ProjectionMatrix", Matrix.Translation(position.LSSwitchYZ()) * Drawing.View * Drawing.Projection);
                    _effect.SetValue(
                        "CircleColor", new Vector4(color.R / 255f, color.G / 255f, color.B / 255f, color.A / 255f));
                    _effect.SetValue("Radius", radius);
                    _effect.SetValue("Border", 2f + width);
                    _effect.SetValue("zEnabled", zDeep);

                    Device.SetStreamSource(0, _vertices, 0, Utilities.SizeOf<Vector4>() * 2);
                    Device.VertexDeclaration = _vertexDeclaration;

                    Device.DrawPrimitives(PrimitiveType.TriangleList, 0, 2);

                    _effect.EndPass();
                    _effect.End();

                    Device.VertexDeclaration = olddec;
                }

                catch (Exception e)
                {
                    _vertices = null;
                    Console.WriteLine(@"DrawCircle: " + e);
                }

            }
        }
    }

    /// <summary>
    ///     An enum representing the order the minions should be listed.
    /// </summary>
    public enum MinionOrderTypes
    {
        /// <summary>
        ///     No order.
        /// </summary>
        None,

        /// <summary>
        ///     Ordered by the current health of the minion. (Least to greatest)
        /// </summary>
        Health,

        /// <summary>
        ///     Ordered by the maximum health of the minions. (Greatest to least)
        /// </summary>
        MaxHealth
    }

    /// <summary>
    ///     The team of the minion.
    /// </summary>
    public enum MinionTeam
    {
        /// <summary>
        ///     The minion is not on either team.
        /// </summary>
        Neutral,

        /// <summary>
        ///     The minions is an ally
        /// </summary>
        Ally,

        /// <summary>
        ///     The minions is an enemy
        /// </summary>
        Enemy,

        /// <summary>
        ///     The minion is not an ally
        /// </summary>
        NotAlly,

        /// <summary>
        ///     The minions is not an ally for the enemy
        /// </summary>
        NotAllyForEnemy,

        /// <summary>
        ///     Any minion.
        /// </summary>
        All
    }

    /// <summary>
    ///     The type of minion.
    /// </summary>
    public enum MinionTypes
    {
        /// <summary>
        ///     Ranged minions.
        /// </summary>
        Ranged,

        /// <summary>
        ///     Melee minions.
        /// </summary>
        Melee,

        /// <summary>
        ///     Any minion
        /// </summary>
        All,

        /// <summary>
        ///     Any wards. (TODO)
        /// </summary>
        [Obsolete("Wards have not been implemented yet in the minion manager.")]
        Wards
    }

    /// <summary>
    ///     Manages minions.
    /// </summary>
    public static class MinionManager
    {
        /// <summary>
        ///     Gets minions based on range, type, team and then orders them.
        /// </summary>
        /// <param name="from">The point to get the minions from.</param>
        /// <param name="range">The range.</param>
        /// <param name="type">The type.</param>
        /// <param name="team">The team.</param>
        /// <param name="order">The order.</param>
        /// <returns>List&lt;Obj_AI_Base&gt;.</returns>
        public static List<Obj_AI_Base> GetMinions(Vector3 from,
            float range,
            MinionTypes type = MinionTypes.All,
            MinionTeam team = MinionTeam.Enemy,
            MinionOrderTypes order = MinionOrderTypes.Health)
        {
            var result = (from minion in ObjectManager.Get<Obj_AI_Minion>()
                          where minion.IsValidTarget(range, false, @from)
                          let minionTeam = minion.Team
                          where
                              team == MinionTeam.Neutral && minionTeam == GameObjectTeam.Neutral ||
                              team == MinionTeam.Ally &&
                              minionTeam ==
                              (ObjectManager.Player.Team == GameObjectTeam.Chaos ? GameObjectTeam.Chaos : GameObjectTeam.Order) ||
                              team == MinionTeam.Enemy &&
                              minionTeam ==
                              (ObjectManager.Player.Team == GameObjectTeam.Chaos ? GameObjectTeam.Order : GameObjectTeam.Chaos) ||
                              team == MinionTeam.NotAlly && minionTeam != ObjectManager.Player.Team ||
                              team == MinionTeam.NotAllyForEnemy &&
                              (minionTeam == ObjectManager.Player.Team || minionTeam == GameObjectTeam.Neutral) ||
                              team == MinionTeam.All
                          where
                              minion.IsMelee() && type == MinionTypes.Melee || !minion.IsMelee() && type == MinionTypes.Ranged ||
                              type == MinionTypes.All
                          where
                              minionTeam == GameObjectTeam.Neutral && minion.MaxHealth > 5 && minion.IsHPBarRendered
                          select minion).Cast<Obj_AI_Base>().ToList();

            switch (order)
            {
                case MinionOrderTypes.Health:
                    result = result.OrderBy(o => o.Health).ToList();
                    break;
                case MinionOrderTypes.MaxHealth:
                    result = result.OrderByDescending(o => o.MaxHealth).ToList();
                    break;
            }

            return result;
        }
        /// <summary>
        ///     Gets the minions.
        /// </summary>
        /// <param name="range">The range.</param>
        /// <param name="type">The type.</param>
        /// <param name="team">The team.</param>
        /// <param name="order">The order.</param>
        /// <returns>List&lt;Obj_AI_Base&gt;.</returns>
        public static List<Obj_AI_Base> GetMinions(float range,
            MinionTypes type = MinionTypes.All,
            MinionTeam team = MinionTeam.Enemy,
            MinionOrderTypes order = MinionOrderTypes.Health)
        {
            return GetMinions(ObjectManager.Player.ServerPosition, range, type, team, order);
        }
    }
}
