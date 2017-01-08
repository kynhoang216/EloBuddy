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

        public static class DelayAction
        {
            public delegate void Callback();

            public static List<Action> ActionList = new List<Action>();

            static DelayAction()
            {
                Game.OnUpdate += GameOnOnGameUpdate;
            }

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

            public static void Add(int time, Callback func)
            {
                var action = new Action(time, func);
                ActionList.Add(action);
            }

            public struct Action
            {
                public Callback CallbackObject;
                public int Time;

                public Action(int time, Callback callback)
                {
                    Time = time + Utils.GameTimeTickCount;
                    CallbackObject = callback;
                }
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
                Utility.DelayAction.Add(0, Initialize);
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
                    Utility.DelayAction.Add(500, () => { Game_OnGameStart(new EventArgs()); });
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
            public delegate void OnDashed(Obj_AI_Base sender, Dash.DashItem args);

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
            public static void TriggerOnDash(Obj_AI_Base sender, Dash.DashItem args)
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
    public static class Dash
    {
        /// <summary>
        ///     The detected dashes
        /// </summary>
        private static readonly Dictionary<int, DashItem> DetectedDashes = new Dictionary<int, DashItem>();

        /// <summary>
        ///     Initializes static members of the <see cref="Dash" /> class.
        /// </summary>
        static Dash()
        {
            Obj_AI_Base.OnNewPath += ObjAiHeroOnOnNewPath;
        }

        /// <summary>
        ///     Fired when a unit changes paths.
        /// </summary>
        /// <param name="sender">The sender.</param>
        /// <param name="args">The <see cref="GameObjectNewPathEventArgs" /> instance containing the event data.</param>
        private static void ObjAiHeroOnOnNewPath(Obj_AI_Base sender, GameObjectNewPathEventArgs args)
        {
            if (sender.IsValid() && sender.IsVisible && sender.IsHPBarRendered)
            {
                if (!DetectedDashes.ContainsKey(sender.NetworkId))
                {
                    DetectedDashes.Add(sender.NetworkId, new DashItem());
                }

                if (args.IsDash)
                {
                    var path = new List<Vector2> { (sender.ServerPosition).To2D() };
                    path.AddRange(args.Path.ToList().To2D());

                    DetectedDashes[sender.NetworkId].StartTick = Utils.TickCount;
                    DetectedDashes[sender.NetworkId].Speed = args.Speed;
                    DetectedDashes[sender.NetworkId].StartPos = (sender.ServerPosition).To2D();
                    DetectedDashes[sender.NetworkId].Unit = sender;
                    DetectedDashes[sender.NetworkId].Path = path;
                    DetectedDashes[sender.NetworkId].EndPos = DetectedDashes[sender.NetworkId].Path.Last();
                    DetectedDashes[sender.NetworkId].EndTick = DetectedDashes[sender.NetworkId].StartTick +
                                                               (int)
                                                                   (1000 *
                                                                    (DetectedDashes[sender.NetworkId].EndPos.Distance(
                                                                        DetectedDashes[sender.NetworkId].StartPos) /
                                                                     DetectedDashes[sender.NetworkId].Speed));
                    DetectedDashes[sender.NetworkId].Duration = DetectedDashes[sender.NetworkId].EndTick -
                                                                DetectedDashes[sender.NetworkId].StartTick;

                    CustomEvents.Unit.TriggerOnDash(DetectedDashes[sender.NetworkId].Unit,
                        DetectedDashes[sender.NetworkId]);
                }
                else
                {
                    DetectedDashes[sender.NetworkId].EndTick = 0;
                }
            }
        }


        /// <summary>
        ///     Determines whether this instance is dashing.
        /// </summary>
        /// <param name="unit">The unit.</param>
        /// <returns></returns>
        public static bool IsDashing(this Obj_AI_Base unit)
        {
            if (DetectedDashes.ContainsKey(unit.NetworkId) && unit.Path.Length != 0 && unit.IsHPBarRendered && unit.IsVisible && !unit.IsDead)
            {
                return DetectedDashes[unit.NetworkId].EndTick != 0;
            }
            return false;
        }


        /// <summary>
        ///     Gets the dash information.
        /// </summary>
        /// <param name="unit">The unit.</param>
        /// <returns></returns>
        public static DashItem GetDashInfo(this Obj_AI_Base unit)
        {
            return DetectedDashes.ContainsKey(unit.NetworkId) ? DetectedDashes[unit.NetworkId] : new DashItem();
        }

        /// <summary>
        ///     Represents a dash.
        /// </summary>
        public class DashItem
        {
            /// <summary>
            ///     The duration
            /// </summary>
            public int Duration;

            /// <summary>
            ///     The end position
            /// </summary>
            public Vector2 EndPos;

            /// <summary>
            ///     The end tick
            /// </summary>
            public int EndTick;

            /// <summary>
            ///     <c>true</c> if the dash was a blink, else <c>false</c>
            /// </summary>
            public bool IsBlink;

            /// <summary>
            ///     The path
            /// </summary>
            public List<Vector2> Path;

            /// <summary>
            ///     The speed
            /// </summary>
            public float Speed;

            /// <summary>
            ///     The start position
            /// </summary>
            public Vector2 StartPos;

            /// <summary>
            ///     The start tick
            /// </summary>
            public int StartTick;

            /// <summary>
            ///     The unit
            /// </summary>
            public Obj_AI_Base Unit;
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
    ///     Windows Messages
    ///     Defined in <![CDATA[winuser.h]]> from Windows SDK v6.1
    ///     Documentation pulled from MSDN.
    /// </summary>
    public enum WindowsMessages : uint
    {
        /// <summary>
        ///     The WM_NULL message performs no operation. An application sends the WM_NULL message if it wants to post a message
        ///     that the recipient window will ignore.
        /// </summary>
        WM_NULL = 0x0000,

        /// <summary>
        ///     The WM_CREATE message is sent when an application requests that a window be created by calling the CreateWindowEx
        ///     or CreateWindow function. (The message is sent before the function returns.) The window procedure of the new window
        ///     receives this message after the window is created, but before the window becomes visible.
        /// </summary>
        WM_CREATE = 0x0001,

        /// <summary>
        ///     The WM_DESTROY message is sent when a window is being destroyed. It is sent to the window procedure of the window
        ///     being destroyed after the window is removed from the screen.
        ///     This message is sent first to the window being destroyed and then to the child windows (if any) as they are
        ///     destroyed. During the processing of the message, it can be assumed that all child windows still exist.
        ///     ///
        /// </summary>
        WM_DESTROY = 0x0002,

        /// <summary>
        ///     The WM_MOVE message is sent after a window has been moved.
        /// </summary>
        WM_MOVE = 0x0003,

        /// <summary>
        ///     The WM_SIZE message is sent to a window after its size has changed.
        /// </summary>
        WM_SIZE = 0x0005,

        /// <summary>
        ///     The WM_ACTIVATE message is sent to both the window being activated and the window being deactivated. If the windows
        ///     use the same input queue, the message is sent synchronously, first to the window procedure of the top-level window
        ///     being deactivated, then to the window procedure of the top-level window being activated. If the windows use
        ///     different input queues, the message is sent asynchronously, so the window is activated immediately.
        /// </summary>
        WM_ACTIVATE = 0x0006,

        /// <summary>
        ///     The WM_SETFOCUS message is sent to a window after it has gained the keyboard focus.
        /// </summary>
        WM_SETFOCUS = 0x0007,

        /// <summary>
        ///     The WM_KILLFOCUS message is sent to a window immediately before it loses the keyboard focus.
        /// </summary>
        WM_KILLFOCUS = 0x0008,

        /// <summary>
        ///     The WM_ENABLE message is sent when an application changes the enabled state of a window. It is sent to the window
        ///     whose enabled state is changing. This message is sent before the EnableWindow function returns, but after the
        ///     enabled state (WS_DISABLED style bit) of the window has changed.
        /// </summary>
        WM_ENABLE = 0x000A,

        /// <summary>
        ///     An application sends the WM_SETREDRAW message to a window to allow changes in that window to be redrawn or to
        ///     prevent changes in that window from being redrawn.
        /// </summary>
        WM_SETREDRAW = 0x000B,

        /// <summary>
        ///     An application sends a WM_SETTEXT message to set the text of a window.
        /// </summary>
        WM_SETTEXT = 0x000C,

        /// <summary>
        ///     An application sends a WM_GETTEXT message to copy the text that corresponds to a window into a buffer provided by
        ///     the caller.
        /// </summary>
        WM_GETTEXT = 0x000D,

        /// <summary>
        ///     An application sends a WM_GETTEXTLENGTH message to determine the length, in characters, of the text associated with
        ///     a window.
        /// </summary>
        WM_GETTEXTLENGTH = 0x000E,

        /// <summary>
        ///     The WM_PAINT message is sent when the system or another application makes a request to paint a portion of an
        ///     application's window. The message is sent when the UpdateWindow or RedrawWindow function is called, or by the
        ///     DispatchMessage function when the application obtains a WM_PAINT message by using the GetMessage or PeekMessage
        ///     function.
        /// </summary>
        WM_PAINT = 0x000F,

        /// <summary>
        ///     The WM_CLOSE message is sent as a signal that a window or an application should terminate.
        /// </summary>
        WM_CLOSE = 0x0010,

        /// <summary>
        ///     The WM_QUERYENDSESSION message is sent when the user chooses to end the session or when an application calls one of
        ///     the system shutdown functions. If any application returns zero, the session is not ended. The system stops sending
        ///     WM_QUERYENDSESSION messages as soon as one application returns zero.
        ///     After processing this message, the system sends the WM_ENDSESSION message with the <c>wParam</c> parameter set to
        ///     the
        ///     results of the WM_QUERYENDSESSION message.
        /// </summary>
        WM_QUERYENDSESSION = 0x0011,

        /// <summary>
        ///     The WM_QUERYOPEN message is sent to an icon when the user requests that the window be restored to its previous size
        ///     and position.
        /// </summary>
        WM_QUERYOPEN = 0x0013,

        /// <summary>
        ///     The WM_ENDSESSION message is sent to an application after the system processes the results of the
        ///     WM_QUERYENDSESSION message. The WM_ENDSESSION message informs the application whether the session is ending.
        /// </summary>
        WM_ENDSESSION = 0x0016,

        /// <summary>
        ///     The WM_QUIT message indicates a request to terminate an application and is generated when the application calls the
        ///     PostQuitMessage function. It causes the GetMessage function to return zero.
        /// </summary>
        WM_QUIT = 0x0012,

        /// <summary>
        ///     The WM_ERASEBKGND message is sent when the window background must be erased (for example, when a window is
        ///     resized). The message is sent to prepare an invalidated portion of a window for painting.
        /// </summary>
        WM_ERASEBKGND = 0x0014,

        /// <summary>
        ///     This message is sent to all top-level windows when a change is made to a system color setting.
        /// </summary>
        WM_SYSCOLORCHANGE = 0x0015,

        /// <summary>
        ///     The WM_SHOWWINDOW message is sent to a window when the window is about to be hidden or shown.
        /// </summary>
        WM_SHOWWINDOW = 0x0018,

        /// <summary>
        ///     An application sends the WM_WININICHANGE message to all top-level windows after making a change to the WIN.INI
        ///     file. The SystemParametersInfo function sends this message after an application uses the function to change a
        ///     setting in WIN.INI.
        ///     Note  The WM_WININICHANGE message is provided only for compatibility with earlier versions of the system.
        ///     Applications should use the WM_SETTINGCHANGE message.
        /// </summary>
        WM_WININICHANGE = 0x001A,

        /// <summary>
        ///     An application sends the WM_WININICHANGE message to all top-level windows after making a change to the WIN.INI
        ///     file. The SystemParametersInfo function sends this message after an application uses the function to change a
        ///     setting in WIN.INI.
        ///     Note  The WM_WININICHANGE message is provided only for compatibility with earlier versions of the system.
        ///     Applications should use the WM_SETTINGCHANGE message.
        /// </summary>
        WM_SETTINGCHANGE = WM_WININICHANGE,

        /// <summary>
        ///     The WM_DEVMODECHANGE message is sent to all top-level windows whenever the user changes device-mode settings.
        /// </summary>
        WM_DEVMODECHANGE = 0x001B,

        /// <summary>
        ///     The WM_ACTIVATEAPP message is sent when a window belonging to a different application than the active window is
        ///     about to be activated. The message is sent to the application whose window is being activated and to the
        ///     application whose window is being deactivated.
        /// </summary>
        WM_ACTIVATEAPP = 0x001C,

        /// <summary>
        ///     An application sends the WM_FONTCHANGE message to all top-level windows in the system after changing the pool of
        ///     font resources.
        /// </summary>
        WM_FONTCHANGE = 0x001D,

        /// <summary>
        ///     A message that is sent whenever there is a change in the system time.
        /// </summary>
        WM_TIMECHANGE = 0x001E,

        /// <summary>
        ///     The WM_CANCELMODE message is sent to cancel certain modes, such as mouse capture. For example, the system sends
        ///     this message to the active window when a dialog box or message box is displayed. Certain functions also send this
        ///     message explicitly to the specified window regardless of whether it is the active window. For example, the
        ///     EnableWindow function sends this message when disabling the specified window.
        /// </summary>
        WM_CANCELMODE = 0x001F,

        /// <summary>
        ///     The WM_SETCURSOR message is sent to a window if the mouse causes the cursor to move within a window and mouse input
        ///     is not captured.
        /// </summary>
        WM_SETCURSOR = 0x0020,

        /// <summary>
        ///     The WM_MOUSEACTIVATE message is sent when the cursor is in an inactive window and the user presses a mouse button.
        ///     The parent window receives this message only if the child window passes it to the <c>DefWindowProc</c> function.
        /// </summary>
        WM_MOUSEACTIVATE = 0x0021,

        /// <summary>
        ///     The WM_CHILDACTIVATE message is sent to a child window when the user clicks the window's title bar or when the
        ///     window is activated, moved, or sized.
        /// </summary>
        WM_CHILDACTIVATE = 0x0022,

        /// <summary>
        ///     The WM_QUEUESYNC message is sent by a computer-based training (CBT) application to separate user-input messages
        ///     from other messages sent through the WH_JOURNALPLAYBACK Hook procedure.
        /// </summary>
        WM_QUEUESYNC = 0x0023,

        /// <summary>
        ///     The WM_GETMINMAXINFO message is sent to a window when the size or position of the window is about to change. An
        ///     application can use this message to override the window's default maximized size and position, or its default
        ///     minimum or maximum tracking size.
        /// </summary>
        WM_GETMINMAXINFO = 0x0024,

        /// <summary>
        ///     Windows NT 3.51 and earlier: The WM_PAINTICON message is sent to a minimized window when the icon is to be painted.
        ///     This message is not sent by newer versions of Microsoft Windows, except in unusual circumstances explained in the
        ///     Remarks.
        /// </summary>
        WM_PAINTICON = 0x0026,

        /// <summary>
        ///     Windows NT 3.51 and earlier: The WM_ICONERASEBKGND message is sent to a minimized window when the background of the
        ///     icon must be filled before painting the icon. A window receives this message only if a class icon is defined for
        ///     the window; otherwise, WM_ERASEBKGND is sent. This message is not sent by newer versions of Windows.
        /// </summary>
        WM_ICONERASEBKGND = 0x0027,

        /// <summary>
        ///     The WM_NEXTDLGCTL message is sent to a dialog box procedure to set the keyboard focus to a different control in the
        ///     dialog box.
        /// </summary>
        WM_NEXTDLGCTL = 0x0028,

        /// <summary>
        ///     The WM_SPOOLERSTATUS message is sent from Print Manager whenever a job is added to or removed from the Print
        ///     Manager queue.
        /// </summary>
        WM_SPOOLERSTATUS = 0x002A,

        /// <summary>
        ///     The WM_DRAWITEM message is sent to the parent window of an owner-drawn button, combo box, list box, or menu when a
        ///     visual aspect of the button, combo box, list box, or menu has changed.
        /// </summary>
        WM_DRAWITEM = 0x002B,

        /// <summary>
        ///     The WM_MEASUREITEM message is sent to the owner window of a combo box, list box, list view control, or menu item
        ///     when the control or menu is created.
        /// </summary>
        WM_MEASUREITEM = 0x002C,

        /// <summary>
        ///     Sent to the owner of a list box or combo box when the list box or combo box is destroyed or when items are removed
        ///     by the LB_DELETESTRING, LB_RESETCONTENT, CB_DELETESTRING, or CB_RESETCONTENT message. The system sends a
        ///     WM_DELETEITEM message for each deleted item. The system sends the WM_DELETEITEM message for any deleted list box or
        ///     combo box item with nonzero item data.
        /// </summary>
        WM_DELETEITEM = 0x002D,

        /// <summary>
        ///     Sent by a list box with the LBS_WANTKEYBOARDINPUT style to its owner in response to a WM_KEYDOWN message.
        /// </summary>
        WM_VKEYTOITEM = 0x002E,

        /// <summary>
        ///     Sent by a list box with the LBS_WANTKEYBOARDINPUT style to its owner in response to a WM_CHAR message.
        /// </summary>
        WM_CHARTOITEM = 0x002F,

        /// <summary>
        ///     An application sends a WM_SETFONT message to specify the font that a control is to use when drawing text.
        /// </summary>
        WM_SETFONT = 0x0030,

        /// <summary>
        ///     An application sends a WM_GETFONT message to a control to retrieve the font with which the control is currently
        ///     drawing its text.
        /// </summary>
        WM_GETFONT = 0x0031,

        /// <summary>
        ///     An application sends a WM_SETHOTKEY message to a window to associate a hot key with the window. When the user
        ///     presses the hot key, the system activates the window.
        /// </summary>
        WM_SETHOTKEY = 0x0032,

        /// <summary>
        ///     An application sends a WM_GETHOTKEY message to determine the hot key associated with a window.
        /// </summary>
        WM_GETHOTKEY = 0x0033,

        /// <summary>
        ///     The WM_QUERYDRAGICON message is sent to a minimized (iconic) window. The window is about to be dragged by the user
        ///     but does not have an icon defined for its class. An application can return a handle to an icon or cursor. The
        ///     system displays this cursor or icon while the user drags the icon.
        /// </summary>
        WM_QUERYDRAGICON = 0x0037,

        /// <summary>
        ///     The system sends the WM_COMPAREITEM message to determine the relative position of a new item in the sorted list of
        ///     an owner-drawn combo box or list box. Whenever the application adds a new item, the system sends this message to
        ///     the owner of a combo box or list box created with the CBS_SORT or LBS_SORT style.
        /// </summary>
        WM_COMPAREITEM = 0x0039,

        /// <summary>
        ///     Active Accessibility sends the WM_GETOBJECT message to obtain information about an accessible object contained in a
        ///     server application.
        ///     Applications never send this message directly. It is sent only by Active Accessibility in response to calls to
        ///     AccessibleObjectFromPoint, AccessibleObjectFromEvent, or AccessibleObjectFromWindow. However, server applications
        ///     handle this message.
        /// </summary>
        WM_GETOBJECT = 0x003D,

        /// <summary>
        ///     The WM_COMPACTING message is sent to all top-level windows when the system detects more than 12.5 percent of system
        ///     time over a 30- to 60-second interval is being spent compacting memory. This indicates that system memory is low.
        /// </summary>
        WM_COMPACTING = 0x0041,

        /// <summary>
        ///     WM_COMMNOTIFY is Obsolete for Win32-Based Applications
        /// </summary>
        [Obsolete]
        WM_COMMNOTIFY = 0x0044,

        /// <summary>
        ///     The WM_WINDOWPOSCHANGING message is sent to a window whose size, position, or place in the Z order is about to
        ///     change as a result of a call to the <c>SetWindowPos</c> function or another window-management function.
        /// </summary>
        WM_WINDOWPOSCHANGING = 0x0046,

        /// <summary>
        ///     The WM_WINDOWPOSCHANGED message is sent to a window whose size, position, or place in the Z order has changed as a
        ///     result of a call to the <c>SetWindowPos</c> function or another window-management function.
        /// </summary>
        WM_WINDOWPOSCHANGED = 0x0047,

        /// <summary>
        ///     Notifies applications that the system, typically a battery-powered personal computer, is about to enter a suspended
        ///     mode.
        ///     Use: POWERBROADCAST
        /// </summary>
        [Obsolete]
        WM_POWER = 0x0048,

        /// <summary>
        ///     An application sends the WM_COPYDATA message to pass data to another application.
        /// </summary>
        WM_COPYDATA = 0x004A,

        /// <summary>
        ///     The WM_CANCELJOURNAL message is posted to an application when a user cancels the application's journaling
        ///     activities. The message is posted with a NULL window handle.
        /// </summary>
        WM_CANCELJOURNAL = 0x004B,

        /// <summary>
        ///     Sent by a common control to its parent window when an event has occurred or the control requires some information.
        /// </summary>
        WM_NOTIFY = 0x004E,

        /// <summary>
        ///     The WM_INPUTLANGCHANGEREQUEST message is posted to the window with the focus when the user chooses a new input
        ///     language, either with the hotkey (specified in the Keyboard control panel application) or from the indicator on the
        ///     system taskbar. An application can accept the change by passing the message to the <c>DefWindowProc</c> function or
        ///     reject
        ///     the change (and prevent it from taking place) by returning immediately.
        /// </summary>
        WM_INPUTLANGCHANGEREQUEST = 0x0050,

        /// <summary>
        ///     The WM_INPUTLANGCHANGE message is sent to the topmost affected window after an application's input language has
        ///     been changed. You should make any application-specific settings and pass the message to the <c>DefWindowProc</c>
        ///     function,
        ///     which passes the message to all first-level child windows. These child windows can pass the message to
        ///     <c>DefWindowProc</c> to have it pass the message to their child windows, and so on.
        /// </summary>
        WM_INPUTLANGCHANGE = 0x0051,

        /// <summary>
        ///     Sent to an application that has initiated a training card with Microsoft Windows Help. The message informs the
        ///     application when the user clicks an author-able button. An application initiates a training card by specifying the
        ///     HELP_TCARD command in a call to the WinHelp function.
        /// </summary>
        WM_TCARD = 0x0052,

        /// <summary>
        ///     Indicates that the user pressed the F1 key. If a menu is active when F1 is pressed, WM_HELP is sent to the window
        ///     associated with the menu; otherwise, WM_HELP is sent to the window that has the keyboard focus. If no window has
        ///     the keyboard focus, WM_HELP is sent to the currently active window.
        /// </summary>
        WM_HELP = 0x0053,

        /// <summary>
        ///     The WM_USERCHANGED message is sent to all windows after the user has logged on or off. When the user logs on or
        ///     off, the system updates the user-specific settings. The system sends this message immediately after updating the
        ///     settings.
        /// </summary>
        WM_USERCHANGED = 0x0054,

        /// <summary>
        ///     Determines if a window accepts ANSI or Unicode structures in the WM_NOTIFY notification message. WM_NOTIFYFORMAT
        ///     messages are sent from a common control to its parent window and from the parent window to the common control.
        /// </summary>
        WM_NOTIFYFORMAT = 0x0055,

        /// <summary>
        ///     The WM_CONTEXTMENU message notifies a window that the user clicked the right mouse button (right-clicked) in the
        ///     window.
        /// </summary>
        WM_CONTEXTMENU = 0x007B,

        /// <summary>
        ///     The WM_STYLECHANGING message is sent to a window when the SetWindowLong function is about to change one or more of
        ///     the window's styles.
        /// </summary>
        WM_STYLECHANGING = 0x007C,

        /// <summary>
        ///     The WM_STYLECHANGED message is sent to a window after the SetWindowLong function has changed one or more of the
        ///     window's styles
        /// </summary>
        WM_STYLECHANGED = 0x007D,

        /// <summary>
        ///     The WM_DISPLAYCHANGE message is sent to all windows when the display resolution has changed.
        /// </summary>
        WM_DISPLAYCHANGE = 0x007E,

        /// <summary>
        ///     The WM_GETICON message is sent to a window to retrieve a handle to the large or small icon associated with a
        ///     window. The system displays the large icon in the ALT+TAB dialog, and the small icon in the window caption.
        /// </summary>
        WM_GETICON = 0x007F,

        /// <summary>
        ///     An application sends the WM_SETICON message to associate a new large or small icon with a window. The system
        ///     displays the large icon in the ALT+TAB dialog box, and the small icon in the window caption.
        /// </summary>
        WM_SETICON = 0x0080,

        /// <summary>
        ///     The WM_NCCREATE message is sent prior to the WM_CREATE message when a window is first created.
        /// </summary>
        WM_NCCREATE = 0x0081,

        /// <summary>
        ///     The WM_NCDESTROY message informs a window that its non-client area is being destroyed. The DestroyWindow function
        ///     sends the WM_NCDESTROY message to the window following the WM_DESTROY message. WM_DESTROY is used to free the
        ///     allocated memory object associated with the window.
        ///     The WM_NCDESTROY message is sent after the child windows have been destroyed. In contrast, WM_DESTROY is sent
        ///     before the child windows are destroyed.
        /// </summary>
        WM_NCDESTROY = 0x0082,

        /// <summary>
        ///     The WM_NCCALCSIZE message is sent when the size and position of a window's client area must be calculated. By
        ///     processing this message, an application can control the content of the window's client area when the size or
        ///     position of the window changes.
        /// </summary>
        WM_NCCALCSIZE = 0x0083,

        /// <summary>
        ///     The WM_NCHITTEST message is sent to a window when the cursor moves, or when a mouse button is pressed or released.
        ///     If the mouse is not captured, the message is sent to the window beneath the cursor. Otherwise, the message is sent
        ///     to the window that has captured the mouse.
        /// </summary>
        WM_NCHITTEST = 0x0084,

        /// <summary>
        ///     The WM_NCPAINT message is sent to a window when its frame must be painted.
        /// </summary>
        WM_NCPAINT = 0x0085,

        /// <summary>
        ///     The WM_NCACTIVATE message is sent to a window when its non-client area needs to be changed to indicate an active or
        ///     inactive state.
        /// </summary>
        WM_NCACTIVATE = 0x0086,

        /// <summary>
        ///     The WM_GETDLGCODE message is sent to the window procedure associated with a control. By default, the system handles
        ///     all keyboard input to the control; the system interprets certain types of keyboard input as dialog box navigation
        ///     keys. To override this default behavior, the control can respond to the WM_GETDLGCODE message to indicate the types
        ///     of input it wants to process itself.
        /// </summary>
        WM_GETDLGCODE = 0x0087,

        /// <summary>
        ///     The WM_SYNCPAINT message is used to synchronize painting while avoiding linking independent GUI threads.
        /// </summary>
        WM_SYNCPAINT = 0x0088,

        /// <summary>
        ///     The WM_NCMOUSEMOVE message is posted to a window when the cursor is moved within the non-client area of the window.
        ///     This message is posted to the window that contains the cursor. If a window has captured the mouse, this message is
        ///     not posted.
        /// </summary>
        WM_NCMOUSEMOVE = 0x00A0,

        /// <summary>
        ///     The WM_NCLBUTTONDOWN message is posted when the user presses the left mouse button while the cursor is within the
        ///     non-client area of a window. This message is posted to the window that contains the cursor. If a window has
        ///     captured
        ///     the mouse, this message is not posted.
        /// </summary>
        WM_NCLBUTTONDOWN = 0x00A1,

        /// <summary>
        ///     The WM_NCLBUTTONUP message is posted when the user releases the left mouse button while the cursor is within the
        ///     non-client area of a window. This message is posted to the window that contains the cursor. If a window has
        ///     captured
        ///     the mouse, this message is not posted.
        /// </summary>
        WM_NCLBUTTONUP = 0x00A2,

        /// <summary>
        ///     The WM_NCLBUTTONDBLCLK message is posted when the user double-clicks the left mouse button while the cursor is
        ///     within the non-client area of a window. This message is posted to the window that contains the cursor. If a window
        ///     has captured the mouse, this message is not posted.
        /// </summary>
        WM_NCLBUTTONDBLCLK = 0x00A3,

        /// <summary>
        ///     The WM_NCRBUTTONDOWN message is posted when the user presses the right mouse button while the cursor is within the
        ///     non-client area of a window. This message is posted to the window that contains the cursor. If a window has
        ///     captured
        ///     the mouse, this message is not posted.
        /// </summary>
        WM_NCRBUTTONDOWN = 0x00A4,

        /// <summary>
        ///     The WM_NCRBUTTONUP message is posted when the user releases the right mouse button while the cursor is within the
        ///     non-client area of a window. This message is posted to the window that contains the cursor. If a window has
        ///     captured
        ///     the mouse, this message is not posted.
        /// </summary>
        WM_NCRBUTTONUP = 0x00A5,

        /// <summary>
        ///     The WM_NCRBUTTONDBLCLK message is posted when the user double-clicks the right mouse button while the cursor is
        ///     within the non-client area of a window. This message is posted to the window that contains the cursor. If a window
        ///     has captured the mouse, this message is not posted.
        /// </summary>
        WM_NCRBUTTONDBLCLK = 0x00A6,

        /// <summary>
        ///     The WM_NCMBUTTONDOWN message is posted when the user presses the middle mouse button while the cursor is within the
        ///     non-client area of a window. This message is posted to the window that contains the cursor. If a window has
        ///     captured
        ///     the mouse, this message is not posted.
        /// </summary>
        WM_NCMBUTTONDOWN = 0x00A7,

        /// <summary>
        ///     The WM_NCMBUTTONUP message is posted when the user releases the middle mouse button while the cursor is within the
        ///     non-client area of a window. This message is posted to the window that contains the cursor. If a window has
        ///     captured
        ///     the mouse, this message is not posted.
        /// </summary>
        WM_NCMBUTTONUP = 0x00A8,

        /// <summary>
        ///     The WM_NCMBUTTONDBLCLK message is posted when the user double-clicks the middle mouse button while the cursor is
        ///     within the non-client area of a window. This message is posted to the window that contains the cursor. If a window
        ///     has captured the mouse, this message is not posted.
        /// </summary>
        WM_NCMBUTTONDBLCLK = 0x00A9,

        /// <summary>
        ///     The WM_NCXBUTTONDOWN message is posted when the user presses the first or second X button while the cursor is in
        ///     the non-client area of a window. This message is posted to the window that contains the cursor. If a window has
        ///     captured the mouse, this message is not posted.
        /// </summary>
        WM_NCXBUTTONDOWN = 0x00AB,

        /// <summary>
        ///     The WM_NCXBUTTONUP message is posted when the user releases the first or second X button while the cursor is in the
        ///     non-client area of a window. This message is posted to the window that contains the cursor. If a window has
        ///     captured
        ///     the mouse, this message is not posted.
        /// </summary>
        WM_NCXBUTTONUP = 0x00AC,

        /// <summary>
        ///     The WM_NCXBUTTONDBLCLK message is posted when the user double-clicks the first or second X button while the cursor
        ///     is in the non-client area of a window. This message is posted to the window that contains the cursor. If a window
        ///     has captured the mouse, this message is not posted.
        /// </summary>
        WM_NCXBUTTONDBLCLK = 0x00AD,

        /// <summary>
        ///     The WM_INPUT_DEVICE_CHANGE message is sent to the window that registered to receive raw input. A window receives
        ///     this message through its <c>WindowProc</c> function.
        /// </summary>
        WM_INPUT_DEVICE_CHANGE = 0x00FE,

        /// <summary>
        ///     The WM_INPUT message is sent to the window that is getting raw input.
        /// </summary>
        WM_INPUT = 0x00FF,

        /// <summary>
        ///     This message filters for keyboard messages.
        /// </summary>
        WM_KEYFIRST = 0x0100,

        /// <summary>
        ///     The WM_KEYDOWN message is posted to the window with the keyboard focus when a non-system key is pressed. A
        ///     non-system
        ///     key is a key that is pressed when the ALT key is not pressed.
        /// </summary>
        WM_KEYDOWN = 0x0100,

        /// <summary>
        ///     The WM_KEYUP message is posted to the window with the keyboard focus when a non-system key is released. A
        ///     non-system
        ///     key is a key that is pressed when the ALT key is not pressed, or a keyboard key that is pressed when a window has
        ///     the keyboard focus.
        /// </summary>
        WM_KEYUP = 0x0101,

        /// <summary>
        ///     The WM_CHAR message is posted to the window with the keyboard focus when a WM_KEYDOWN message is translated by the
        ///     TranslateMessage function. The WM_CHAR message contains the character code of the key that was pressed.
        /// </summary>
        WM_CHAR = 0x0102,

        /// <summary>
        ///     The WM_DEADCHAR message is posted to the window with the keyboard focus when a WM_KEYUP message is translated by
        ///     the TranslateMessage function. WM_DEADCHAR specifies a character code generated by a dead key. A dead key is a key
        ///     that generates a character, such as the umlaut (double-dot), that is combined with another character to form a
        ///     composite character. For example, the umlaut-O character (Ö) is generated by typing the dead key for the umlaut
        ///     character, and then typing the O key.
        /// </summary>
        WM_DEADCHAR = 0x0103,

        /// <summary>
        ///     The WM_SYSKEYDOWN message is posted to the window with the keyboard focus when the user presses the F10 key (which
        ///     activates the menu bar) or holds down the ALT key and then presses another key. It also occurs when no window
        ///     currently has the keyboard focus; in this case, the WM_SYSKEYDOWN message is sent to the active window. The window
        ///     that receives the message can distinguish between these two contexts by checking the context code in the
        ///     <c>lParam</c> parameter.
        /// </summary>
        WM_SYSKEYDOWN = 0x0104,

        /// <summary>
        ///     The WM_SYSKEYUP message is posted to the window with the keyboard focus when the user releases a key that was
        ///     pressed while the ALT key was held down. It also occurs when no window currently has the keyboard focus; in this
        ///     case, the WM_SYSKEYUP message is sent to the active window. The window that receives the message can distinguish
        ///     between these two contexts by checking the context code in the <c>lParam</c> parameter.
        /// </summary>
        WM_SYSKEYUP = 0x0105,

        /// <summary>
        ///     The WM_SYSCHAR message is posted to the window with the keyboard focus when a WM_SYSKEYDOWN message is translated
        ///     by the TranslateMessage function. It specifies the character code of a system character key — that is, a character
        ///     key that is pressed while the ALT key is down.
        /// </summary>
        WM_SYSCHAR = 0x0106,

        /// <summary>
        ///     The WM_SYSDEADCHAR message is sent to the window with the keyboard focus when a WM_SYSKEYDOWN message is translated
        ///     by the TranslateMessage function. WM_SYSDEADCHAR specifies the character code of a system dead key — that is, a
        ///     dead key that is pressed while holding down the ALT key.
        /// </summary>
        WM_SYSDEADCHAR = 0x0107,

        /// <summary>
        ///     The WM_UNICHAR message is posted to the window with the keyboard focus when a WM_KEYDOWN message is translated by
        ///     the TranslateMessage function. The WM_UNICHAR message contains the character code of the key that was pressed.
        ///     The WM_UNICHAR message is equivalent to WM_CHAR, but it uses Unicode Transformation Format (UTF)-32, whereas
        ///     WM_CHAR uses UTF-16. It is designed to send or post Unicode characters to ANSI windows and it can can handle
        ///     Unicode Supplementary Plane characters.
        /// </summary>
        WM_UNICHAR = 0x0109,

        /// <summary>
        ///     This message filters for keyboard messages.
        /// </summary>
        WM_KEYLAST = 0x0109,

        /// <summary>
        ///     Sent immediately before the IME generates the composition string as a result of a keystroke. A window receives this
        ///     message through its <c>WindowProc</c> function.
        /// </summary>
        WM_IME_STARTCOMPOSITION = 0x010D,

        /// <summary>
        ///     Sent to an application when the IME ends composition. A window receives this message through its <c>WindowProc</c>
        ///     function.
        /// </summary>
        WM_IME_ENDCOMPOSITION = 0x010E,

        /// <summary>
        ///     Sent to an application when the IME changes composition status as a result of a keystroke. A window receives this
        ///     message through its <c>WindowProc</c> function.
        /// </summary>
        WM_IME_COMPOSITION = 0x010F,

        /// <summary>
        ///     Sent to an application when the IME changes composition status as a result of a keystroke. A window receives this
        ///     message through its <c>WindowProc</c> function.
        /// </summary>
        WM_IME_KEYLAST = 0x010F,

        /// <summary>
        ///     The WM_INITDIALOG message is sent to the dialog box procedure immediately before a dialog box is displayed. Dialog
        ///     box procedures typically use this message to initialize controls and carry out any other initialization tasks that
        ///     affect the appearance of the dialog box.
        /// </summary>
        WM_INITDIALOG = 0x0110,

        /// <summary>
        ///     The WM_COMMAND message is sent when the user selects a command item from a menu, when a control sends a
        ///     notification message to its parent window, or when an accelerator keystroke is translated.
        /// </summary>
        WM_COMMAND = 0x0111,

        /// <summary>
        ///     A window receives this message when the user chooses a command from the Window menu, clicks the maximize button,
        ///     minimize button, restore button, close button, or moves the form. You can stop the form from moving by filtering
        ///     this out.
        /// </summary>
        WM_SYSCOMMAND = 0x0112,

        /// <summary>
        ///     The WM_TIMER message is posted to the installing thread's message queue when a timer expires. The message is posted
        ///     by the GetMessage or PeekMessage function.
        /// </summary>
        WM_TIMER = 0x0113,

        /// <summary>
        ///     The WM_HSCROLL message is sent to a window when a scroll event occurs in the window's standard horizontal scroll
        ///     bar. This message is also sent to the owner of a horizontal scroll bar control when a scroll event occurs in the
        ///     control.
        /// </summary>
        WM_HSCROLL = 0x0114,

        /// <summary>
        ///     The WM_VSCROLL message is sent to a window when a scroll event occurs in the window's standard vertical scroll bar.
        ///     This message is also sent to the owner of a vertical scroll bar control when a scroll event occurs in the control.
        /// </summary>
        WM_VSCROLL = 0x0115,

        /// <summary>
        ///     The WM_INITMENU message is sent when a menu is about to become active. It occurs when the user clicks an item on
        ///     the menu bar or presses a menu key. This allows the application to modify the menu before it is displayed.
        /// </summary>
        WM_INITMENU = 0x0116,

        /// <summary>
        ///     The WM_INITMENUPOPUP message is sent when a drop-down menu or submenu is about to become active. This allows an
        ///     application to modify the menu before it is displayed, without changing the entire menu.
        /// </summary>
        WM_INITMENUPOPUP = 0x0117,

        /// <summary>
        ///     The WM_MENUSELECT message is sent to a menu's owner window when the user selects a menu item.
        /// </summary>
        WM_MENUSELECT = 0x011F,

        /// <summary>
        ///     The WM_MENUCHAR message is sent when a menu is active and the user presses a key that does not correspond to any
        ///     mnemonic or accelerator key. This message is sent to the window that owns the menu.
        /// </summary>
        WM_MENUCHAR = 0x0120,

        /// <summary>
        ///     The WM_ENTERIDLE message is sent to the owner window of a modal dialog box or menu that is entering an idle state.
        ///     A modal dialog box or menu enters an idle state when no messages are waiting in its queue after it has processed
        ///     one or more previous messages.
        /// </summary>
        WM_ENTERIDLE = 0x0121,

        /// <summary>
        ///     The WM_MENURBUTTONUP message is sent when the user releases the right mouse button while the cursor is on a menu
        ///     item.
        /// </summary>
        WM_MENURBUTTONUP = 0x0122,

        /// <summary>
        ///     The WM_MENUDRAG message is sent to the owner of a drag-and-drop menu when the user drags a menu item.
        /// </summary>
        WM_MENUDRAG = 0x0123,

        /// <summary>
        ///     The WM_MENUGETOBJECT message is sent to the owner of a drag-and-drop menu when the mouse cursor enters a menu item
        ///     or moves from the center of the item to the top or bottom of the item.
        /// </summary>
        WM_MENUGETOBJECT = 0x0124,

        /// <summary>
        ///     The WM_UNINITMENUPOPUP message is sent when a drop-down menu or submenu has been destroyed.
        /// </summary>
        WM_UNINITMENUPOPUP = 0x0125,

        /// <summary>
        ///     The WM_MENUCOMMAND message is sent when the user makes a selection from a menu.
        /// </summary>
        WM_MENUCOMMAND = 0x0126,

        /// <summary>
        ///     An application sends the WM_CHANGEUISTATE message to indicate that the user interface (UI) state should be changed.
        /// </summary>
        WM_CHANGEUISTATE = 0x0127,

        /// <summary>
        ///     An application sends the WM_UPDATEUISTATE message to change the user interface (UI) state for the specified window
        ///     and all its child windows.
        /// </summary>
        WM_UPDATEUISTATE = 0x0128,

        /// <summary>
        ///     An application sends the WM_QUERYUISTATE message to retrieve the user interface (UI) state for a window.
        /// </summary>
        WM_QUERYUISTATE = 0x0129,

        /// <summary>
        ///     The WM_CTLCOLORMSGBOX message is sent to the owner window of a message box before Windows draws the message box. By
        ///     responding to this message, the owner window can set the text and background colors of the message box by using the
        ///     given display device context handle.
        /// </summary>
        WM_CTLCOLORMSGBOX = 0x0132,

        /// <summary>
        ///     An edit control that is not read-only or disabled sends the WM_CTLCOLOREDIT message to its parent window when the
        ///     control is about to be drawn. By responding to this message, the parent window can use the specified device context
        ///     handle to set the text and background colors of the edit control.
        /// </summary>
        WM_CTLCOLOREDIT = 0x0133,

        /// <summary>
        ///     Sent to the parent window of a list box before the system draws the list box. By responding to this message, the
        ///     parent window can set the text and background colors of the list box by using the specified display device context
        ///     handle.
        /// </summary>
        WM_CTLCOLORLISTBOX = 0x0134,

        /// <summary>
        ///     The WM_CTLCOLORBTN message is sent to the parent window of a button before drawing the button. The parent window
        ///     can change the button's text and background colors. However, only owner-drawn buttons respond to the parent window
        ///     processing this message.
        /// </summary>
        WM_CTLCOLORBTN = 0x0135,

        /// <summary>
        ///     The WM_CTLCOLORDLG message is sent to a dialog box before the system draws the dialog box. By responding to this
        ///     message, the dialog box can set its text and background colors using the specified display device context handle.
        /// </summary>
        WM_CTLCOLORDLG = 0x0136,

        /// <summary>
        ///     The WM_CTLCOLORSCROLLBAR message is sent to the parent window of a scroll bar control when the control is about to
        ///     be drawn. By responding to this message, the parent window can use the display context handle to set the background
        ///     color of the scroll bar control.
        /// </summary>
        WM_CTLCOLORSCROLLBAR = 0x0137,

        /// <summary>
        ///     A static control, or an edit control that is read-only or disabled, sends the WM_CTLCOLORSTATIC message to its
        ///     parent window when the control is about to be drawn. By responding to this message, the parent window can use the
        ///     specified device context handle to set the text and background colors of the static control.
        /// </summary>
        WM_CTLCOLORSTATIC = 0x0138,

        /// <summary>
        ///     Use WM_MOUSEFIRST to specify the first mouse message. Use the PeekMessage() Function.
        /// </summary>
        WM_MOUSEFIRST = 0x0200,

        /// <summary>
        ///     The WM_MOUSEMOVE message is posted to a window when the cursor moves. If the mouse is not captured, the message is
        ///     posted to the window that contains the cursor. Otherwise, the message is posted to the window that has captured the
        ///     mouse.
        /// </summary>
        WM_MOUSEMOVE = 0x0200,

        /// <summary>
        ///     The WM_LBUTTONDOWN message is posted when the user presses the left mouse button while the cursor is in the client
        ///     area of a window. If the mouse is not captured, the message is posted to the window beneath the cursor. Otherwise,
        ///     the message is posted to the window that has captured the mouse.
        /// </summary>
        WM_LBUTTONDOWN = 0x0201,

        /// <summary>
        ///     The WM_LBUTTONUP message is posted when the user releases the left mouse button while the cursor is in the client
        ///     area of a window. If the mouse is not captured, the message is posted to the window beneath the cursor. Otherwise,
        ///     the message is posted to the window that has captured the mouse.
        /// </summary>
        WM_LBUTTONUP = 0x0202,

        /// <summary>
        ///     The WM_LBUTTONDBLCLK message is posted when the user double-clicks the left mouse button while the cursor is in the
        ///     client area of a window. If the mouse is not captured, the message is posted to the window beneath the cursor.
        ///     Otherwise, the message is posted to the window that has captured the mouse.
        /// </summary>
        WM_LBUTTONDBLCLK = 0x0203,

        /// <summary>
        ///     The WM_RBUTTONDOWN message is posted when the user presses the right mouse button while the cursor is in the client
        ///     area of a window. If the mouse is not captured, the message is posted to the window beneath the cursor. Otherwise,
        ///     the message is posted to the window that has captured the mouse.
        /// </summary>
        WM_RBUTTONDOWN = 0x0204,

        /// <summary>
        ///     The WM_RBUTTONUP message is posted when the user releases the right mouse button while the cursor is in the client
        ///     area of a window. If the mouse is not captured, the message is posted to the window beneath the cursor. Otherwise,
        ///     the message is posted to the window that has captured the mouse.
        /// </summary>
        WM_RBUTTONUP = 0x0205,

        /// <summary>
        ///     The WM_RBUTTONDBLCLK message is posted when the user double-clicks the right mouse button while the cursor is in
        ///     the client area of a window. If the mouse is not captured, the message is posted to the window beneath the cursor.
        ///     Otherwise, the message is posted to the window that has captured the mouse.
        /// </summary>
        WM_RBUTTONDBLCLK = 0x0206,

        /// <summary>
        ///     The WM_MBUTTONDOWN message is posted when the user presses the middle mouse button while the cursor is in the
        ///     client area of a window. If the mouse is not captured, the message is posted to the window beneath the cursor.
        ///     Otherwise, the message is posted to the window that has captured the mouse.
        /// </summary>
        WM_MBUTTONDOWN = 0x0207,

        /// <summary>
        ///     The WM_MBUTTONUP message is posted when the user releases the middle mouse button while the cursor is in the client
        ///     area of a window. If the mouse is not captured, the message is posted to the window beneath the cursor. Otherwise,
        ///     the message is posted to the window that has captured the mouse.
        /// </summary>
        WM_MBUTTONUP = 0x0208,

        /// <summary>
        ///     The WM_MBUTTONDBLCLK message is posted when the user double-clicks the middle mouse button while the cursor is in
        ///     the client area of a window. If the mouse is not captured, the message is posted to the window beneath the cursor.
        ///     Otherwise, the message is posted to the window that has captured the mouse.
        /// </summary>
        WM_MBUTTONDBLCLK = 0x0209,

        /// <summary>
        ///     The WM_MOUSEWHEEL message is sent to the focus window when the mouse wheel is rotated. The <c>DefWindowProc</c>
        ///     function
        ///     propagates the message to the window's parent. There should be no internal forwarding of the message, since
        ///     <c>DefWindowProc</c> propagates it up the parent chain until it finds a window that processes it.
        /// </summary>
        WM_MOUSEWHEEL = 0x020A,

        /// <summary>
        ///     The WM_XBUTTONDOWN message is posted when the user presses the first or second X button while the cursor is in the
        ///     client area of a window. If the mouse is not captured, the message is posted to the window beneath the cursor.
        ///     Otherwise, the message is posted to the window that has captured the mouse.
        /// </summary>
        WM_XBUTTONDOWN = 0x020B,

        /// <summary>
        ///     The WM_XBUTTONUP message is posted when the user releases the first or second X button while the cursor is in the
        ///     client area of a window. If the mouse is not captured, the message is posted to the window beneath the cursor.
        ///     Otherwise, the message is posted to the window that has captured the mouse.
        /// </summary>
        WM_XBUTTONUP = 0x020C,

        /// <summary>
        ///     The WM_XBUTTONDBLCLK message is posted when the user double-clicks the first or second X button while the cursor is
        ///     in the client area of a window. If the mouse is not captured, the message is posted to the window beneath the
        ///     cursor. Otherwise, the message is posted to the window that has captured the mouse.
        /// </summary>
        WM_XBUTTONDBLCLK = 0x020D,

        /// <summary>
        ///     The WM_MOUSEHWHEEL message is sent to the focus window when the mouse's horizontal scroll wheel is tilted or
        ///     rotated. The <c>DefWindowProc</c> function propagates the message to the window's parent. There should be no
        ///     internal
        ///     forwarding of the message, since <c>DefWindowProc</c> propagates it up the parent chain until it finds a window
        ///     that
        ///     processes it.
        /// </summary>
        WM_MOUSEHWHEEL = 0x020E,

        /// <summary>
        ///     Use WM_MOUSELAST to specify the last mouse message. Used with PeekMessage() Function.
        /// </summary>
        WM_MOUSELAST = 0x020E,

        /// <summary>
        ///     The WM_PARENTNOTIFY message is sent to the parent of a child window when the child window is created or destroyed,
        ///     or when the user clicks a mouse button while the cursor is over the child window. When the child window is being
        ///     created, the system sends WM_PARENTNOTIFY just before the CreateWindow or CreateWindowEx function that creates the
        ///     window returns. When the child window is being destroyed, the system sends the message before any processing to
        ///     destroy the window takes place.
        /// </summary>
        WM_PARENTNOTIFY = 0x0210,

        /// <summary>
        ///     The WM_ENTERMENULOOP message informs an application's main window procedure that a menu modal loop has been
        ///     entered.
        /// </summary>
        WM_ENTERMENULOOP = 0x0211,

        /// <summary>
        ///     The WM_EXITMENULOOP message informs an application's main window procedure that a menu modal loop has been exited.
        /// </summary>
        WM_EXITMENULOOP = 0x0212,

        /// <summary>
        ///     The WM_NEXTMENU message is sent to an application when the right or left arrow key is used to switch between the
        ///     menu bar and the system menu.
        /// </summary>
        WM_NEXTMENU = 0x0213,

        /// <summary>
        ///     The WM_SIZING message is sent to a window that the user is resizing. By processing this message, an application can
        ///     monitor the size and position of the drag rectangle and, if needed, change its size or position.
        /// </summary>
        WM_SIZING = 0x0214,

        /// <summary>
        ///     The WM_CAPTURECHANGED message is sent to the window that is losing the mouse capture.
        /// </summary>
        WM_CAPTURECHANGED = 0x0215,

        /// <summary>
        ///     The WM_MOVING message is sent to a window that the user is moving. By processing this message, an application can
        ///     monitor the position of the drag rectangle and, if needed, change its position.
        /// </summary>
        WM_MOVING = 0x0216,

        /// <summary>
        ///     Notifies applications that a power-management event has occurred.
        /// </summary>
        WM_POWERBROADCAST = 0x0218,

        /// <summary>
        ///     Notifies an application of a change to the hardware configuration of a device or the computer.
        /// </summary>
        WM_DEVICECHANGE = 0x0219,

        /// <summary>
        ///     An application sends the WM_MDICREATE message to a multiple-document interface (MDI) client window to create an MDI
        ///     child window.
        /// </summary>
        WM_MDICREATE = 0x0220,

        /// <summary>
        ///     An application sends the WM_MDIDESTROY message to a multiple-document interface (MDI) client window to close an MDI
        ///     child window.
        /// </summary>
        WM_MDIDESTROY = 0x0221,

        /// <summary>
        ///     An application sends the WM_MDIACTIVATE message to a multiple-document interface (MDI) client window to instruct
        ///     the client window to activate a different MDI child window.
        /// </summary>
        WM_MDIACTIVATE = 0x0222,

        /// <summary>
        ///     An application sends the WM_MDIRESTORE message to a multiple-document interface (MDI) client window to restore an
        ///     MDI child window from maximized or minimized size.
        /// </summary>
        WM_MDIRESTORE = 0x0223,

        /// <summary>
        ///     An application sends the WM_MDINEXT message to a multiple-document interface (MDI) client window to activate the
        ///     next or previous child window.
        /// </summary>
        WM_MDINEXT = 0x0224,

        /// <summary>
        ///     An application sends the WM_MDIMAXIMIZE message to a multiple-document interface (MDI) client window to maximize an
        ///     MDI child window. The system resizes the child window to make its client area fill the client window. The system
        ///     places the child window's window menu icon in the rightmost position of the frame window's menu bar, and places the
        ///     child window's restore icon in the leftmost position. The system also appends the title bar text of the child
        ///     window to that of the frame window.
        /// </summary>
        WM_MDIMAXIMIZE = 0x0225,

        /// <summary>
        ///     An application sends the WM_MDITILE message to a multiple-document interface (MDI) client window to arrange all of
        ///     its MDI child windows in a tile format.
        /// </summary>
        WM_MDITILE = 0x0226,

        /// <summary>
        ///     An application sends the WM_MDICASCADE message to a multiple-document interface (MDI) client window to arrange all
        ///     its child windows in a cascade format.
        /// </summary>
        WM_MDICASCADE = 0x0227,

        /// <summary>
        ///     An application sends the WM_MDIICONARRANGE message to a multiple-document interface (MDI) client window to arrange
        ///     all minimized MDI child windows. It does not affect child windows that are not minimized.
        /// </summary>
        WM_MDIICONARRANGE = 0x0228,

        /// <summary>
        ///     An application sends the WM_MDIGETACTIVE message to a multiple-document interface (MDI) client window to retrieve
        ///     the handle to the active MDI child window.
        /// </summary>
        WM_MDIGETACTIVE = 0x0229,

        /// <summary>
        ///     An application sends the WM_MDISETMENU message to a multiple-document interface (MDI) client window to replace the
        ///     entire menu of an MDI frame window, to replace the window menu of the frame window, or both.
        /// </summary>
        WM_MDISETMENU = 0x0230,

        /// <summary>
        ///     The WM_ENTERSIZEMOVE message is sent one time to a window after it enters the moving or sizing modal loop. The
        ///     window enters the moving or sizing modal loop when the user clicks the window's title bar or sizing border, or when
        ///     the window passes the WM_SYSCOMMAND message to the <c>DefWindowProc</c> function and the <c>wParam</c> parameter of
        ///     the message
        ///     specifies the SC_MOVE or SC_SIZE value. The operation is complete when <c>DefWindowProc</c> returns.
        ///     The system sends the WM_ENTERSIZEMOVE message regardless of whether the dragging of full windows is enabled.
        /// </summary>
        WM_ENTERSIZEMOVE = 0x0231,

        /// <summary>
        ///     The WM_EXITSIZEMOVE message is sent one time to a window, after it has exited the moving or sizing modal loop. The
        ///     window enters the moving or sizing modal loop when the user clicks the window's title bar or sizing border, or when
        ///     the window passes the WM_SYSCOMMAND message to the <c>DefWindowProc</c> function and the <c>wParam</c> parameter of
        ///     the message
        ///     specifies the SC_MOVE or SC_SIZE value. The operation is complete when <c>DefWindowProc</c> returns.
        /// </summary>
        WM_EXITSIZEMOVE = 0x0232,

        /// <summary>
        ///     Sent when the user drops a file on the window of an application that has registered itself as a recipient of
        ///     dropped files.
        /// </summary>
        WM_DROPFILES = 0x0233,

        /// <summary>
        ///     An application sends the WM_MDIREFRESHMENU message to a multiple-document interface (MDI) client window to refresh
        ///     the window menu of the MDI frame window.
        /// </summary>
        WM_MDIREFRESHMENU = 0x0234,

        /// <summary>
        ///     Sent to an application when a window is activated. A window receives this message through its <c>WindowProc</c>
        ///     function.
        /// </summary>
        WM_IME_SETCONTEXT = 0x0281,

        /// <summary>
        ///     Sent to an application to notify it of changes to the IME window. A window receives this message through its
        ///     <c>WindowProc</c> function.
        /// </summary>
        WM_IME_NOTIFY = 0x0282,

        /// <summary>
        ///     Sent by an application to direct the IME window to carry out the requested command. The application uses this
        ///     message to control the IME window that it has created. To send this message, the application calls the SendMessage
        ///     function with the following parameters.
        /// </summary>
        WM_IME_CONTROL = 0x0283,

        /// <summary>
        ///     Sent to an application when the IME window finds no space to extend the area for the composition window. A window
        ///     receives this message through its <c>WindowProc</c> function.
        /// </summary>
        WM_IME_COMPOSITIONFULL = 0x0284,

        /// <summary>
        ///     Sent to an application when the operating system is about to change the current IME. A window receives this message
        ///     through its <c>WindowProc</c> function.
        /// </summary>
        WM_IME_SELECT = 0x0285,

        /// <summary>
        ///     Sent to an application when the IME gets a character of the conversion result. A window receives this message
        ///     through its <c>WindowProc</c> function.
        /// </summary>
        WM_IME_CHAR = 0x0286,

        /// <summary>
        ///     Sent to an application to provide commands and request information. A window receives this message through its
        ///     <c>WindowProc</c> function.
        /// </summary>
        WM_IME_REQUEST = 0x0288,

        /// <summary>
        ///     Sent to an application by the IME to notify the application of a key press and to keep message order. A window
        ///     receives this message through its <c>WindowProc</c> function.
        /// </summary>
        WM_IME_KEYDOWN = 0x0290,

        /// <summary>
        ///     Sent to an application by the IME to notify the application of a key release and to keep message order. A window
        ///     receives this message through its <c>WindowProc</c> function.
        /// </summary>
        WM_IME_KEYUP = 0x0291,

        /// <summary>
        ///     The WM_MOUSEHOVER message is posted to a window when the cursor hovers over the client area of the window for the
        ///     period of time specified in a prior call to TrackMouseEvent.
        /// </summary>
        WM_MOUSEHOVER = 0x02A1,

        /// <summary>
        ///     The WM_MOUSELEAVE message is posted to a window when the cursor leaves the client area of the window specified in a
        ///     prior call to TrackMouseEvent.
        /// </summary>
        WM_MOUSELEAVE = 0x02A3,

        /// <summary>
        ///     The WM_NCMOUSEHOVER message is posted to a window when the cursor hovers over the non-client area of the window for
        ///     the period of time specified in a prior call to TrackMouseEvent.
        /// </summary>
        WM_NCMOUSEHOVER = 0x02A0,

        /// <summary>
        ///     The WM_NCMOUSELEAVE message is posted to a window when the cursor leaves the non-client area of the window
        ///     specified
        ///     in a prior call to TrackMouseEvent.
        /// </summary>
        WM_NCMOUSELEAVE = 0x02A2,

        /// <summary>
        ///     The WM_WTSSESSION_CHANGE message notifies applications of changes in session state.
        /// </summary>
        WM_WTSSESSION_CHANGE = 0x02B1,

        /// <summary>
        ///     The WM_TABLET_FIRST messages notifies applications of change in session state of a tablet.
        /// </summary>
        WM_TABLET_FIRST = 0x02c0,

        /// <summary>
        ///     The WM_TABLET_FIRST messages notifies applications of change in session state of a tablet.
        /// </summary>
        WM_TABLET_LAST = 0x02df,

        /// <summary>
        ///     An application sends a WM_CUT message to an edit control or combo box to delete (cut) the current selection, if
        ///     any, in the edit control and copy the deleted text to the clipboard in CF_TEXT format.
        /// </summary>
        WM_CUT = 0x0300,

        /// <summary>
        ///     An application sends the WM_COPY message to an edit control or combo box to copy the current selection to the
        ///     clipboard in CF_TEXT format.
        /// </summary>
        WM_COPY = 0x0301,

        /// <summary>
        ///     An application sends a WM_PASTE message to an edit control or combo box to copy the current content of the
        ///     clipboard to the edit control at the current caret position. Data is inserted only if the clipboard contains data
        ///     in CF_TEXT format.
        /// </summary>
        WM_PASTE = 0x0302,

        /// <summary>
        ///     An application sends a WM_CLEAR message to an edit control or combo box to delete (clear) the current selection, if
        ///     any, from the edit control.
        /// </summary>
        WM_CLEAR = 0x0303,

        /// <summary>
        ///     An application sends a WM_UNDO message to an edit control to undo the last operation. When this message is sent to
        ///     an edit control, the previously deleted text is restored or the previously added text is deleted.
        /// </summary>
        WM_UNDO = 0x0304,

        /// <summary>
        ///     The WM_RENDERFORMAT message is sent to the clipboard owner if it has delayed rendering a specific clipboard format
        ///     and if an application has requested data in that format. The clipboard owner must render data in the specified
        ///     format and place it on the clipboard by calling the SetClipboardData function.
        /// </summary>
        WM_RENDERFORMAT = 0x0305,

        /// <summary>
        ///     The WM_RENDERALLFORMATS message is sent to the clipboard owner before it is destroyed, if the clipboard owner has
        ///     delayed rendering one or more clipboard formats. For the content of the clipboard to remain available to other
        ///     applications, the clipboard owner must render data in all the formats it is capable of generating, and place the
        ///     data on the clipboard by calling the SetClipboardData function.
        /// </summary>
        WM_RENDERALLFORMATS = 0x0306,

        /// <summary>
        ///     The WM_DESTROYCLIPBOARD message is sent to the clipboard owner when a call to the EmptyClipboard function empties
        ///     the clipboard.
        /// </summary>
        WM_DESTROYCLIPBOARD = 0x0307,

        /// <summary>
        ///     The WM_DRAWCLIPBOARD message is sent to the first window in the clipboard viewer chain when the content of the
        ///     clipboard changes. This enables a clipboard viewer window to display the new content of the clipboard.
        /// </summary>
        WM_DRAWCLIPBOARD = 0x0308,

        /// <summary>
        ///     The WM_PAINTCLIPBOARD message is sent to the clipboard owner by a clipboard viewer window when the clipboard
        ///     contains data in the CF_OWNERDISPLAY format and the clipboard viewer's client area needs repainting.
        /// </summary>
        WM_PAINTCLIPBOARD = 0x0309,

        /// <summary>
        ///     The WM_VSCROLLCLIPBOARD message is sent to the clipboard owner by a clipboard viewer window when the clipboard
        ///     contains data in the CF_OWNERDISPLAY format and an event occurs in the clipboard viewer's vertical scroll bar. The
        ///     owner should scroll the clipboard image and update the scroll bar values.
        /// </summary>
        WM_VSCROLLCLIPBOARD = 0x030A,

        /// <summary>
        ///     The WM_SIZECLIPBOARD message is sent to the clipboard owner by a clipboard viewer window when the clipboard
        ///     contains data in the CF_OWNERDISPLAY format and the clipboard viewer's client area has changed size.
        /// </summary>
        WM_SIZECLIPBOARD = 0x030B,

        /// <summary>
        ///     The WM_ASKCBFORMATNAME message is sent to the clipboard owner by a clipboard viewer window to request the name of a
        ///     CF_OWNERDISPLAY clipboard format.
        /// </summary>
        WM_ASKCBFORMATNAME = 0x030C,

        /// <summary>
        ///     The WM_CHANGECBCHAIN message is sent to the first window in the clipboard viewer chain when a window is being
        ///     removed from the chain.
        /// </summary>
        WM_CHANGECBCHAIN = 0x030D,

        /// <summary>
        ///     The WM_HSCROLLCLIPBOARD message is sent to the clipboard owner by a clipboard viewer window. This occurs when the
        ///     clipboard contains data in the CF_OWNERDISPLAY format and an event occurs in the clipboard viewer's horizontal
        ///     scroll bar. The owner should scroll the clipboard image and update the scroll bar values.
        /// </summary>
        WM_HSCROLLCLIPBOARD = 0x030E,

        /// <summary>
        ///     This message informs a window that it is about to receive the keyboard focus, giving the window the opportunity to
        ///     realize its logical palette when it receives the focus.
        /// </summary>
        WM_QUERYNEWPALETTE = 0x030F,

        /// <summary>
        ///     The WM_PALETTEISCHANGING message informs applications that an application is going to realize its logical palette.
        /// </summary>
        WM_PALETTEISCHANGING = 0x0310,

        /// <summary>
        ///     This message is sent by the OS to all top-level and overlapped windows after the window with the keyboard focus
        ///     realizes its logical palette.
        ///     This message enables windows that do not have the keyboard focus to realize their logical palettes and update their
        ///     client areas.
        /// </summary>
        WM_PALETTECHANGED = 0x0311,

        /// <summary>
        ///     The WM_HOTKEY message is posted when the user presses a hot key registered by the RegisterHotKey function. The
        ///     message is placed at the top of the message queue associated with the thread that registered the hot key.
        /// </summary>
        WM_HOTKEY = 0x0312,

        /// <summary>
        ///     The WM_PRINT message is sent to a window to request that it draw itself in the specified device context, most
        ///     commonly in a printer device context.
        /// </summary>
        WM_PRINT = 0x0317,

        /// <summary>
        ///     The WM_PRINTCLIENT message is sent to a window to request that it draw its client area in the specified device
        ///     context, most commonly in a printer device context.
        /// </summary>
        WM_PRINTCLIENT = 0x0318,

        /// <summary>
        ///     The WM_APPCOMMAND message notifies a window that the user generated an application command event, for example, by
        ///     clicking an application command button using the mouse or typing an application command key on the keyboard.
        /// </summary>
        WM_APPCOMMAND = 0x0319,

        /// <summary>
        ///     The WM_THEMECHANGED message is broadcast to every window following a theme change event. Examples of theme change
        ///     events are the activation of a theme, the deactivation of a theme, or a transition from one theme to another.
        /// </summary>
        WM_THEMECHANGED = 0x031A,

        /// <summary>
        ///     Sent when the contents of the clipboard have changed.
        /// </summary>
        WM_CLIPBOARDUPDATE = 0x031D,

        /// <summary>
        ///     The system will send a window the WM_DWMCOMPOSITIONCHANGED message to indicate that the availability of desktop
        ///     composition has changed.
        /// </summary>
        WM_DWMCOMPOSITIONCHANGED = 0x031E,

        /// <summary>
        ///     WM_DWMNCRENDERINGCHANGED is called when the non-client area rendering status of a window has changed. Only windows
        ///     that have set the flag DWM_BLURBEHIND.fTransitionOnMaximized to true will get this message.
        /// </summary>
        WM_DWMNCRENDERINGCHANGED = 0x031F,

        /// <summary>
        ///     Sent to all top-level windows when the colorization color has changed.
        /// </summary>
        WM_DWMCOLORIZATIONCOLORCHANGED = 0x0320,

        /// <summary>
        ///     WM_DWMWINDOWMAXIMIZEDCHANGE will let you know when a DWM composed window is maximized. You also have to register
        ///     for this message as well. You'd have other window go opaque when this message is sent.
        /// </summary>
        WM_DWMWINDOWMAXIMIZEDCHANGE = 0x0321,

        /// <summary>
        ///     Sent to request extended title bar information. A window receives this message through its <c>WindowProc</c>
        ///     function.
        /// </summary>
        WM_GETTITLEBARINFOEX = 0x033F,

        /// <summary>
        ///     Functions use this constant for filtering purposes.
        /// </summary>
        WM_HANDHELDFIRST = 0x0358,

        /// <summary>
        ///     Functions use this constant for filtering purposes.
        /// </summary>
        WM_HANDHELDLAST = 0x035F,

        /// <summary>
        ///     Functions use this constant for filtering purposes.
        /// </summary>
        WM_AFXFIRST = 0x0360,

        /// <summary>
        ///     Functions use this constant for filtering purposes.
        /// </summary>
        WM_AFXLAST = 0x037F,

        /// <summary>
        ///     Functions use this constant for filtering purposes.
        /// </summary>
        WM_PENWINFIRST = 0x0380,

        /// <summary>
        ///     Functions use this constant for filtering purposes.
        /// </summary>
        WM_PENWINLAST = 0x038F,

        /// <summary>
        ///     The WM_APP constant is used by applications to help define private messages, usually of the form WM_APP+X, where X
        ///     is an integer value.
        /// </summary>
        WM_APP = 0x8000,

        /// <summary>
        ///     The WM_USER constant is used by applications to help define private messages for use by private window classes,
        ///     usually of the form WM_USER+X, where X is an integer value.
        /// </summary>
        WM_USER = 0x0400,

        /// <summary>
        ///     An application sends the WM_CPL_LAUNCH message to Windows Control Panel to request that a Control Panel application
        ///     be started.
        /// </summary>
        WM_CPL_LAUNCH = WM_USER + 0x1000,

        /// <summary>
        ///     The WM_CPL_LAUNCHED message is sent when a Control Panel application, started by the WM_CPL_LAUNCH message, has
        ///     closed. The WM_CPL_LAUNCHED message is sent to the window identified by the <c>wParam</c> parameter of the
        ///     WM_CPL_LAUNCH
        ///     message that started the application.
        /// </summary>
        WM_CPL_LAUNCHED = WM_USER + 0x1001,

        /// <summary>
        ///     WM_SYSTIMER is a well-known yet still undocumented message. Windows uses WM_SYSTIMER for internal actions like
        ///     scrolling.
        /// </summary>
        WM_SYSTIMER = 0x118,

        /// <summary>
        ///     The accessibility state has changed.
        /// </summary>
        WM_HSHELL_ACCESSIBILITYSTATE = 11,

        /// <summary>
        ///     The shell should activate its main window.
        /// </summary>
        WM_HSHELL_ACTIVATESHELLWINDOW = 3,

        /// <summary>
        ///     The user completed an input event (for example, pressed an application command button on the mouse or an
        ///     application command key on the keyboard), and the application did not handle the WM_APPCOMMAND message generated by
        ///     that input.
        ///     If the Shell procedure handles the WM_COMMAND message, it should not call CallNextHookEx. See the Return Value
        ///     section for more information.
        /// </summary>
        WM_HSHELL_APPCOMMAND = 12,

        /// <summary>
        ///     A window is being minimized or maximized. The system needs the coordinates of the minimized rectangle for the
        ///     window.
        /// </summary>
        WM_HSHELL_GETMINRECT = 5,

        /// <summary>
        ///     Keyboard language was changed or a new keyboard layout was loaded.
        /// </summary>
        WM_HSHELL_LANGUAGE = 8,

        /// <summary>
        ///     The title of a window in the task bar has been redrawn.
        /// </summary>
        WM_HSHELL_REDRAW = 6,

        /// <summary>
        ///     The user has selected the task list. A shell application that provides a task list should return TRUE to prevent
        ///     Windows from starting its task list.
        /// </summary>
        WM_HSHELL_TASKMAN = 7,

        /// <summary>
        ///     A top-level, unowned window has been created. The window exists when the system calls this hook.
        /// </summary>
        WM_HSHELL_WINDOWCREATED = 1,

        /// <summary>
        ///     A top-level, unowned window is about to be destroyed. The window still exists when the system calls this hook.
        /// </summary>
        WM_HSHELL_WINDOWDESTROYED = 2,

        /// <summary>
        ///     The activation has changed to a different top-level, unowned window.
        /// </summary>
        WM_HSHELL_WINDOWACTIVATED = 4,

        /// <summary>
        ///     A top-level window is being replaced. The window exists when the system calls this hook.
        /// </summary>
        WM_HSHELL_WINDOWREPLACED = 13
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