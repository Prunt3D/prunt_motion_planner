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

with Physical_Types; use Physical_Types;

package Motion_Planner is

   type Kinematic_Parameters is record
      Lower_Pos_Limit         : Position         := [others => 0.0 * mm];
      Upper_Pos_Limit         : Position         := [others => 0.0 * mm];
      Ignore_E_In_XYZE        : Boolean          := False;
      Shift_Blended_Corners   : Boolean          := True;
      Tangential_Velocity_Max : Velocity         := 0.0 * mm / s;
      Axial_Velocity_Maxes    : Axial_Velocities := [others => 0.0 * mm / s];
      Pressure_Advance_Time   : Time             := 0.0 * s;
      Acceleration_Max        : Acceleration     := 0.0 * mm / s**2;
      Jerk_Max                : Jerk             := 0.0 * mm / s**3;
      Snap_Max                : Snap             := 0.0 * mm / s**4;
      Crackle_Max             : Crackle          := 0.0 * mm / s**5;
      Chord_Error_Max         : Length           := 0.0 * mm;
      Higher_Order_Scaler     : Position_Scale   := [others => 1.0];
   end record;

   type Max_Corners_Type is range 2 .. 2**63 - 1;

   type Feedrate_Profile_Times_Index is range 1 .. 4;
   type Feedrate_Profile_Times is array (Feedrate_Profile_Times_Index) of Time;

   type Feedrate_Profile is tagged record
      Accel : Feedrate_Profile_Times;
      Coast : Time;
      Decel : Feedrate_Profile_Times;
   end record;

   function Fast_Distance_At_Max_Time
     (Profile : Feedrate_Profile_Times; Max_Crackle : Crackle; Start_Vel : Velocity) return Length;
   function Fast_Velocity_At_Max_Time
     (Profile : Feedrate_Profile_Times; Max_Crackle : Crackle; Start_Vel : Velocity) return Velocity;
   --  These two functions use a symbolically equivalent equation to X_At_Time where T is the time at the end of the
   --  feedrate profile. They may not be numerically identical to the functions that take T as an input but this does
   --  not cause issues with the current design of the motion planner.

   function Total_Time (Times : Feedrate_Profile_Times) return Time;

   function Crackle_At_Time (Profile : Feedrate_Profile_Times; T : Time; Max_Crackle : Crackle) return Crackle;
   function Snap_At_Time (Profile : Feedrate_Profile_Times; T : Time; Max_Crackle : Crackle) return Snap;
   function Jerk_At_Time (Profile : Feedrate_Profile_Times; T : Time; Max_Crackle : Crackle) return Jerk;
   function Acceleration_At_Time
     (Profile : Feedrate_Profile_Times; T : Time; Max_Crackle : Crackle) return Acceleration;
   function Velocity_At_Time
     (Profile : Feedrate_Profile_Times; T : Time; Max_Crackle : Crackle; Start_Vel : Velocity) return Velocity;
   function Distance_At_Time
     (Profile : Feedrate_Profile_Times; T : Time; Max_Crackle : Crackle; Start_Vel : Velocity) return Length;

   function Total_Time (Profile : Feedrate_Profile) return Time;

   function Crackle_At_Time (Profile : Feedrate_Profile; T : Time; Max_Crackle : Crackle) return Crackle;
   function Snap_At_Time (Profile : Feedrate_Profile; T : Time; Max_Crackle : Crackle) return Snap;
   function Jerk_At_Time (Profile : Feedrate_Profile; T : Time; Max_Crackle : Crackle) return Jerk;
   function Acceleration_At_Time (Profile : Feedrate_Profile; T : Time; Max_Crackle : Crackle) return Acceleration;
   function Velocity_At_Time
     (Profile : Feedrate_Profile; T : Time; Max_Crackle : Crackle; Start_Vel : Velocity) return Velocity;
   function Distance_At_Time
     (Profile : Feedrate_Profile; T : Time; Max_Crackle : Crackle; Start_Vel : Velocity) return Length;
   function Distance_At_Time
     (Profile            :     Feedrate_Profile;
      T                  :     Time;
      Max_Crackle        :     Crackle;
      Start_Vel          :     Velocity;
      Is_Past_Accel_Part : out Boolean)
      return Length;

end Motion_Planner;
