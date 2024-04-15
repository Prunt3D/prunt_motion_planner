-----------------------------------------------------------------------------
--                                                                         --
--                   Part of the Prunt Motion Controller                   --
--                                                                         --
--            Copyright (C) 2024 Liam Powell (liam@prunt3d.com)            --
--                                                                         --
--  This program is free software: you can redistribute it and/or modify   --
--  it under the terms of the GNU General Public License as published by   --
--  the Free Software Foundation, either version 3 of the License, or      --
--  (at your option) any later version.                                    --
--                                                                         --
--  This program is distributed in the hope that it will be useful,        --
--  but WITHOUT ANY WARRANTY; without even the implied warranty of         --
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          --
--  GNU General Public License for more details.                           --
--                                                                         --
--  You should have received a copy of the GNU General Public License      --
--  along with this program.  If not, see <http://www.gnu.org/licenses/>.  --
--                                                                         --
-----------------------------------------------------------------------------

with Ada.Unchecked_Conversion;

package body Motion_Planner.Planner.Kinematic_Limiter is

   procedure Run (Block : in out Execution_Block) is
      function Curve_Corner_Distance (Finishing_Corner : Corners_Index) return Length is
         Start_Curve_Half_Distance : constant Length :=
           Distance_At_T (Block.Beziers (Finishing_Corner - 1), 1.0) -
           Distance_At_T (Block.Beziers (Finishing_Corner - 1), 0.5);
         End_Curve_Half_Distance   : constant Length := Distance_At_T (Block.Beziers (Finishing_Corner), 0.5);
         Mid_Distance              : constant Length :=
           abs
           (Point_At_T (Block.Beziers (Finishing_Corner), 0.0) -
            Point_At_T (Block.Beziers (Finishing_Corner - 1), 1.0));
      begin
         return Start_Curve_Half_Distance + Mid_Distance + End_Curve_Half_Distance;
      end Curve_Corner_Distance;
   begin
      Block.Corner_Velocity_Limits (Block.Corner_Velocity_Limits'First) := 0.0 * mm / s;
      Block.Corner_Velocity_Limits (Block.Corner_Velocity_Limits'Last)  := 0.0 * mm / s;

      for I in Block.Corner_Velocity_Limits'First + 1 .. Block.Corner_Velocity_Limits'Last - 1 loop
         declare
            Limit           : Velocity := Velocity'Min (Block.Segment_Feedrates (I), Block.Segment_Feedrates (I + 1));
            Optimal_Profile : Feedrate_Profile_Times;

            Inverse_Curvature : constant Length := PH_Beziers.Inverse_Curvature (Block.Beziers (I));
         begin
            --  Inverse curvature range is 0..Length'Last. Make sure to avoid overflow here.  GCC with optimisation
            --  enabled may transform sqrt(x)*sqrt(y) to sqrt(x*y) etc., but that should be fine in optimised builds
            --  with Ada's checks disabled as the Velocity'Min call will immediately discard the resulting infinity.
            Limit := Velocity'Min (Limit, Block.Limits.Acceleration_Max**(1 / 2) * Inverse_Curvature**(1 / 2));
            Limit := Velocity'Min (Limit, Block.Limits.Jerk_Max**(1 / 3) * Inverse_Curvature**(2 / 3));
            Limit := Velocity'Min (Limit, Block.Limits.Snap_Max**(1 / 4) * Inverse_Curvature**(3 / 4));
            Limit := Velocity'Min (Limit, Block.Limits.Crackle_Max**(1 / 5) * Inverse_Curvature**(4 / 5));

            --  TODO: Add limit based on interpolation time.
            --  TODO: Snap and crackle limits currently do not match the paper and are likely overly conservative.

            Optimal_Profile :=
              Optimal_Profile_For_Distance
                (Block.Corner_Velocity_Limits (I - 1),
                 Curve_Corner_Distance (I),
                 Block.Limits.Acceleration_Max,
                 Block.Limits.Jerk_Max,
                 Block.Limits.Snap_Max,
                 Block.Limits.Crackle_Max);
            Limit           :=
              Velocity'Min
                (Limit,
                 Fast_Velocity_At_Max_Time
                   (Optimal_Profile, 0.97 * Block.Limits.Crackle_Max, Block.Corner_Velocity_Limits (I - 1)));
            --  The 0.97 here ensures that no feedrate profiles end up with a very small accel/decel part which can
            --  lead to numerical errors that cause kinematic limits to be greatly exceeded for a single interpolation
            --  period. If this is removed, then the sanity check in Feedrate_Profile_Generator also needs to be
            --  removed.
            --
            --  TODO: Check whether this actually matters in practice.

            Block.Corner_Velocity_Limits (I) := Limit;
         end;
      end loop;

      for I in reverse Block.Corner_Velocity_Limits'First + 1 .. Block.Corner_Velocity_Limits'Last - 1 loop
         declare
            Optimal_Profile : Feedrate_Profile_Times;
         begin
            Optimal_Profile                  :=
              Optimal_Profile_For_Distance
                (Block.Corner_Velocity_Limits (I + 1),
                 Curve_Corner_Distance (I + 1),
                 Block.Limits.Acceleration_Max,
                 Block.Limits.Jerk_Max,
                 Block.Limits.Snap_Max,
                 Block.Limits.Crackle_Max);
            Block.Corner_Velocity_Limits (I) :=
              Velocity'Min
                (Block.Corner_Velocity_Limits (I),
                 Fast_Velocity_At_Max_Time
                   (Optimal_Profile, 0.97 * Block.Limits.Crackle_Max, Block.Corner_Velocity_Limits (I + 1)));
         end;
      end loop;
   end Run;

   function Optimal_Profile_For_Distance
     (Start_Vel        : Velocity;
      Distance         : Length;
      Acceleration_Max : Acceleration;
      Jerk_Max         : Jerk;
      Snap_Max         : Snap;
      Crackle_Max      : Crackle)
      return Feedrate_Profile_Times
   is
      D     : constant Length       := Distance;
      Vs    : constant Velocity     := Start_Vel;
      Am    : constant Acceleration := Acceleration_Max;
      Jm    : constant Jerk         := Jerk_Max;
      Sm    : constant Snap         := Snap_Max;
      Cm    : constant Crackle      := Crackle_Max;
      Cases : array (Feedrate_Profile_Times_Index) of Feedrate_Profile_Times;

      function Solve_Distance_At_Time
        (Profile : Feedrate_Profile_Times; Variable : Feedrate_Profile_Times_Index) return Feedrate_Profile_Times
      is
         Result : Feedrate_Profile_Times := Profile;

         Lower : Time := 0.0 * s;
         Upper : Time := 86_400.0 * s;
         --  A maximum of 24 hours should be more than enough unless you are using Prunt to control a space probe or
         --  a particle accelerator. It is not recommended to install Prunt on space probes or particle
         --  accelerators.

         type Casted_Time is mod 2**64;
         function Cast_Time is new Ada.Unchecked_Conversion (Time, Casted_Time);
         function Cast_Time is new Ada.Unchecked_Conversion (Casted_Time, Time);
      begin
         --  This probably breaks when not using IEEE 754 floats or on other weird systems, so try to check for
         --  that.
         pragma Assert (Time'Size = 64);
         pragma Assert (Casted_Time'Size = 64);
         pragma Assert (Cast_Time (86_400.0 * s) = 4_680_673_776_000_565_248);
         pragma Assert (Cast_Time (0.123_45 * s) = 4_593_559_930_647_147_132);

         loop
            Result (Variable) := Cast_Time (Cast_Time (Lower) + (Cast_Time (Upper) - Cast_Time (Lower)) / 2);
            exit when Lower = Result (Variable) or Upper = Result (Variable);
            if Fast_Distance_At_Max_Time (Result, Cm, Vs) <= D then
               Lower := Result (Variable);
            else
               Upper := Result (Variable);
            end if;
         end loop;

         return Result;
      end Solve_Distance_At_Time;

   begin
      --!pp off
      if Sm**2 < Jm * Cm then
         if Am >= Jm * (Jm / Sm + Sm / Cm) then
            Cases :=
            [
               --  Reachable: Sm, Jm, Am
               4 => [Sm / Cm, Jm / Sm - Sm / Cm, Am / Jm - Jm / Sm - Sm / Cm, 0.0 * s],
               --  Reachable: Sm, Jm
               3 => [Sm / Cm, Jm / Sm - Sm / Cm, 0.0 * s, 0.0 * s],
               --  Reachable: Sm
               2 => [Sm / Cm, 0.0 * s, 0.0 * s, 0.0 * s],
               --  Reachable: None
               1 => [0.0 * s, 0.0 * s, 0.0 * s, 0.0 * s]
            ];
         elsif Am >= 2.0 * Sm**3 / Cm**2 then
            Cases :=
            [
               --  Reachable: Sm, Am
               4 => [Sm / Cm, (0.25 * Sm**2 / Cm**2 + Am / Sm)**(1 / 2) - 1.5 * Sm / Cm, 0.0 * s, 0.0 * s],
               --  Impossible case.
               3 => [Sm / Cm, (0.25 * Sm**2 / Cm**2 + Am / Sm)**(1 / 2) - 1.5 * Sm / Cm, 0.0 * s, 0.0 * s],
               --  Reachable: Sm
               2 => [Sm / Cm, 0.0 * s, 0.0 * s, 0.0 * s],
               --  Reachable: None
               1 => [0.0 * s, 0.0 * s, 0.0 * s, 0.0 * s]
            ];
         else
            Cases :=
            [
               --  Reachable: Am
               4 => [(0.5 * Am / Cm)**(1 / 3), 0.0 * s, 0.0 * s, 0.0 * s],
               --  Impossible case.
               3 => [(0.5 * Am / Cm)**(1 / 3), 0.0 * s, 0.0 * s, 0.0 * s],
               --  Impossible case.
               2 => [(0.5 * Am / Cm)**(1 / 3), 0.0 * s, 0.0 * s, 0.0 * s],
               --  Reachable: None
               1 => [0.0 * s, 0.0 * s, 0.0 * s, 0.0 * s]
            ];
         end if;
      else
         if Am > 2.0 * Jm * (Jm / Cm)**(1 / 2) then
            Cases :=
            [
               --  Reachable: Jm, Am
               4 => [(Jm / Cm)**(1 / 2), 0.0 * s, Am / Jm - 2.0 * (Jm / Cm)**(1 / 2), 0.0 * s],
               --  Reachable: Jm
               3 => [(Jm / Cm)**(1 / 2), 0.0 * s, 0.0 * s, 0.0 * s],
               --  Impossible case.
               2 => [(Jm / Cm)**(1 / 2), 0.0 * s, 0.0 * s, 0.0 * s],
               --  Reachable: None
               1 => [0.0 * s, 0.0 * s, 0.0 * s, 0.0 * s]
            ];
         else
            Cases :=
            [
               --  Reachable: Am
               4 => [(Am / (2.0 * Cm))**(1 / 3), 0.0 * s, 0.0 * s, 0.0 * s],
               --  Impossible case.
               3 => [(Am / (2.0 * Cm))**(1 / 3), 0.0 * s, 0.0 * s, 0.0 * s],
               --  Impossible case.
               2 => [(Am / (2.0 * Cm))**(1 / 3), 0.0 * s, 0.0 * s, 0.0 * s],
               --  Reachable: None
               1 => [0.0 * s, 0.0 * s, 0.0 * s, 0.0 * s]
            ];
         end if;
      end if;
      --!pp on

      for I in reverse Cases'Range loop
         if I = Cases'First or D > Fast_Distance_At_Max_Time (Cases (I), Cm, Vs) then
            return Solve_Distance_At_Time (Cases (I), I);
            --  There are simple analytical solutions for a lot of these, but this is already fast so there is no
            --  reason to optimise it.
         end if;
      end loop;

      --  Unreachable.
      raise Program_Error;
   end Optimal_Profile_For_Distance;

end Motion_Planner.Planner.Kinematic_Limiter;
