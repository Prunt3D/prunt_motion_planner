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

package body Motion_Planner.Planner.Feedrate_Profile_Generator is

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
      for I in Block.Feedrate_Profiles'Range loop
         declare
            Profile : constant Feedrate_Profile_Times :=
              Optimal_Profile_For_Delta_V
                (Block.Corner_Velocity_Limits (I - 1) - Block.Corner_Velocity_Limits (I),
                 Block.Limits.Acceleration_Max,
                 Block.Limits.Jerk_Max,
                 Block.Limits.Snap_Max,
                 Block.Limits.Crackle_Max);

            Accel_Profile_Distance : constant Length :=
              Fast_Distance_At_Max_Time (Profile, Block.Limits.Crackle_Max, Block.Corner_Velocity_Limits (I - 1));
            Decel_Profile_Distance : constant Length :=
              Fast_Distance_At_Max_Time (Profile, -Block.Limits.Crackle_Max, Block.Corner_Velocity_Limits (I - 1));

            Curve_Distance : constant Length := Curve_Corner_Distance (I);
         begin
            pragma Assert (Curve_Distance < Length'Min (Accel_Profile_Distance, Decel_Profile_Distance));
         end;

         Block.Feedrate_Profiles (I).Accel :=
           Optimal_Profile_For_Delta_V
             (Block.Corner_Velocity_Limits (I - 1) - Block.Segment_Feedrates (I),
              Block.Limits.Acceleration_Max,
              Block.Limits.Jerk_Max,
              Block.Limits.Snap_Max,
              Block.Limits.Crackle_Max);
         Block.Feedrate_Profiles (I).Decel :=
           Optimal_Profile_For_Delta_V
             (Block.Corner_Velocity_Limits (I) - Block.Segment_Feedrates (I),
              Block.Limits.Acceleration_Max,
              Block.Limits.Jerk_Max,
              Block.Limits.Snap_Max,
              Block.Limits.Crackle_Max);

         declare
            Accel_Distance : Length            :=
              Fast_Distance_At_Max_Time
                (Block.Feedrate_Profiles (I).Accel, Block.Limits.Crackle_Max, Block.Corner_Velocity_Limits (I - 1));
            Coast_Velocity : constant Velocity := Block.Segment_Feedrates (I);
            Decel_Distance : Length            :=
              Fast_Distance_At_Max_Time (Block.Feedrate_Profiles (I).Decel, -Block.Limits.Crackle_Max, Coast_Velocity);
            Curve_Distance : constant Length   := Curve_Corner_Distance (I);
         begin
            if Accel_Distance + Decel_Distance <= Curve_Distance then
               Block.Feedrate_Profiles (I).Coast :=
                 (Curve_Distance - Accel_Distance - Decel_Distance) / Coast_Velocity;
            else
               Block.Feedrate_Profiles (I).Coast := 0.0 * s;
               declare
                  type Casted_Vel is mod 2**64;
                  function Cast_Vel is new Ada.Unchecked_Conversion (Velocity, Casted_Vel);
                  function Cast_Vel is new Ada.Unchecked_Conversion (Casted_Vel, Velocity);
                  Upper : Velocity := Block.Segment_Feedrates (I);
                  Lower : Velocity :=
                    Velocity'Max (Block.Corner_Velocity_Limits (I - 1), Block.Corner_Velocity_Limits (I));
                  Mid   : Velocity;
               begin
                  --  This probably breaks when not using IEEE 754 floats or on other weird systems, so try to check
                  --  for that.
                  pragma Assert (Velocity'Size = 64);
                  pragma Assert (Casted_Vel'Size = 64);
                  pragma Assert (Cast_Vel (86_400.0 * mm / s) = 4_680_673_776_000_565_248);
                  pragma Assert (Cast_Vel (0.123_45 * mm / s) = 4_593_559_930_647_147_132);

                  loop
                     Mid := Cast_Vel (Cast_Vel (Lower) + (Cast_Vel (Upper) - Cast_Vel (Lower)) / 2);
                     exit when Lower = Mid or Upper = Mid;

                     Block.Feedrate_Profiles (I).Accel :=
                       Optimal_Profile_For_Delta_V
                         (Block.Corner_Velocity_Limits (I - 1) - Mid,
                          Block.Limits.Acceleration_Max,
                          Block.Limits.Jerk_Max,
                          Block.Limits.Snap_Max,
                          Block.Limits.Crackle_Max);
                     Block.Feedrate_Profiles (I).Decel :=
                       Optimal_Profile_For_Delta_V
                         (Block.Corner_Velocity_Limits (I) - Mid,
                          Block.Limits.Acceleration_Max,
                          Block.Limits.Jerk_Max,
                          Block.Limits.Snap_Max,
                          Block.Limits.Crackle_Max);

                     Accel_Distance :=
                       Fast_Distance_At_Max_Time
                         (Block.Feedrate_Profiles (I).Accel,
                          Block.Limits.Crackle_Max,
                          Block.Corner_Velocity_Limits (I - 1));
                     Decel_Distance :=
                       Fast_Distance_At_Max_Time
                         (Block.Feedrate_Profiles (I).Decel,
                          Block.Limits.Crackle_Max,
                          Block.Corner_Velocity_Limits (I));

                     if Accel_Distance + Decel_Distance <= Curve_Distance then
                        Lower := Mid;
                     else
                        Upper := Mid;
                     end if;
                  end loop;
               end;
            end if;
         end;
      end loop;
   end Run;

   function Optimal_Profile_For_Delta_V
     (Delta_V          : Velocity;
      Acceleration_Max : Acceleration;
      Jerk_Max         : Jerk;
      Snap_Max         : Snap;
      Crackle_Max      : Crackle)
      return Feedrate_Profile_Times
   is
      Vd : constant Velocity     := abs Delta_V;
      Am : constant Acceleration := Acceleration_Max;
      Jm : constant Jerk         := Jerk_Max;
      Sm : constant Snap         := Snap_Max;
      Cm : constant Crackle      := Crackle_Max;

      function Solve_Velocity_At_Time
        (Profile  : Feedrate_Profile_Times;
         Variable : Feedrate_Profile_Times_Index;
         Target   : Velocity)
         return Feedrate_Profile_Times
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
            if Fast_Velocity_At_Max_Time (Result, Cm, 0.0 * mm / s) <= Target then
               Lower := Result (Variable);
            else
               Upper := Result (Variable);
            end if;
         end loop;

         return Result;
      end Solve_Velocity_At_Time;
   begin
      --  This function is called a lot more than Optimal_Profile_For_Distance, so we use simple analytical solutions
      --  where they exist. In the one case where we resort to Solve_Velocity_At_Time, the analytical solution that
      --  Mathematica outputs involves a Cm**18, which is far outside the range of Dimensioned_Float for reasonable
      --  values of Cm.
      --
      --  For reference:
      --  ToRadicals[
      --    Solve[
      --      With[
      --        {T1 = Sm/Cm, T3 = 0, T4 = 0},
      --        v == Cm*T1*(T1 + T2)*(2*T1 + T2 + T3)*(4*T1 + 2*T2 + T3 + T4)
      --      ],
      --      T2,
      --      NonNegativeReals
      --    ]
      --  ]
      if Sm**2 < Jm * Cm then
         if Am >= Jm * (Jm / Sm + Sm / Cm) then
            if Vd > Am * (Am / Jm + Jm / Sm + Sm / Cm) then
               --  Reachable: Sm, Jm, Am
               return [Sm / Cm, Jm / Sm - Sm / Cm, Am / Jm - Jm / Sm - Sm / Cm, Vd / Am - Am / Jm - Jm / Sm - Sm / Cm];
            elsif Vd > 2.0 * Jm * (Jm / Sm + Sm / Cm)**2 then
               --  Reachable: Sm, Jm
               return
                 [Sm / Cm,
                 Jm / Sm - Sm / Cm,
                 0.5 * ((Jm / Sm + Sm / Cm)**2 + 4.0 * Vd / Jm)**(1 / 2) - 1.5 * (Jm / Sm + Sm / Cm),
                 0.0 * s];
            elsif Vd > 8.0 * Sm**4 / Cm**3 then
               --  Reachable: Sm
               return Solve_Velocity_At_Time ([Sm / Cm, 0.0 * s, 0.0 * s, 0.0 * s], 2, Vd);
            else
               --  Reachable: None
               return [(0.125 * Vd / Cm)**(1 / 4), 0.0 * s, 0.0 * s, 0.0 * s];
            end if;
         elsif Am >= 2.0 * Sm**3 / Cm**2 then
            if Vd > Am * (2.0 * (0.25 * Sm**2 / Cm**2 + Am / Sm)**(1 / 2) + Sm / Cm) then
               --  Reachable: Sm, Am
               return
                 [Sm / Cm,
                 (0.25 * Sm**2 / Cm**2 + Am / Sm)**(1 / 2) - 1.5 * Sm / Cm,
                 0.0 * s,
                 Vd / Am - Sm / Cm - 2.0 * (0.25 * Sm**2 / Cm**2 + Am / Sm)**(1 / 2)];
            elsif Vd > 8.0 * Sm**4 / Cm**3 then
               --  Reachable: Sm
               return Solve_Velocity_At_Time ([Sm / Cm, 0.0 * s, 0.0 * s, 0.0 * s], 2, Vd);
            else
               --  Reachable: None
               return [(0.125 * Vd / Cm)**(1 / 4), 0.0 * s, 0.0 * s, 0.0 * s];
            end if;
         else
            if Vd > 8.0 * Cm * (0.5 * Am / Cm)**(4 / 3) then
               --  Reachable: Am
               return [(0.5 * Am / Cm)**(1 / 3), 0.0 * s, 0.0 * s, Vd / Am - 4.0 * (0.5 * Am / Cm)**(1 / 3)];
            else
               --  Reachable: None
               return [(0.125 * Vd / Cm)**(1 / 4), 0.0 * s, 0.0 * s, 0.0 * s];
            end if;
         end if;
      else
         if Am > 2.0 * Jm * (Jm / Cm)**(1 / 2) then
            if Vd > Am * (Am / Jm + 2.0 * (Jm / Cm)**(1 / 2)) then
               --  Reachable: Jm, Am
               return
                 [(Jm / Cm)**(1 / 2),
                 0.0 * s,
                 Am / Jm - 2.0 * (Jm / Cm)**(1 / 2),
                 Vd / Am - Am / Jm - 2.0 * (Jm / Cm)**(1 / 2)];
            elsif Vd > 8.0 * Jm**2 / Cm then
               --  Reachable: Jm
               return [(Jm / Cm)**(1 / 2), 0.0 * s, (Jm / Cm + Vd / Jm)**(1 / 2) - 3.0 * (Jm / Cm)**(1 / 2), 0.0 * s];
            else
               --  Reachable: None
               return [(0.125 * Vd / Cm)**(1 / 4), 0.0 * s, 0.0 * s, 0.0 * s];
            end if;
         else
            if Vd > 8.0 * Cm * (0.5 * Am / Cm)**(4 / 3) then
               --  Reachable: Am
               return [(0.5 * Am / Cm)**(1 / 3), 0.0 * s, 0.0 * s, Vd / Am - 4.0 * (0.5 * Am / Cm)**(1 / 3)];
            else
               --  Reachable: None
               return [(0.125 * Vd / Cm)**(1 / 4), 0.0 * s, 0.0 * s, 0.0 * s];
            end if;
         end if;
      end if;
   end Optimal_Profile_For_Delta_V;

end Motion_Planner.Planner.Feedrate_Profile_Generator;
