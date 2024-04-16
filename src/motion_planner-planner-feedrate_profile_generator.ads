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

private generic
package Motion_Planner.Planner.Feedrate_Profile_Generator is

   procedure Run (Block : in out Execution_Block);
   --  Fills Block.Feedrate_Profiles with profiles based on Block.Corner_Velocity_Limits and Block.Params.

private

   function Optimal_Profile_For_Delta_V
     (Delta_V          : Velocity;
      Acceleration_Max : Acceleration;
      Jerk_Max         : Jerk;
      Snap_Max         : Snap;
      Crackle_Max      : Crackle)
      return Feedrate_Profile_Times;
   --  Compute the motion profile that achieves the given change in velocity in the lowest time without violationg any
   --  of the given constraints.

end Motion_Planner.Planner.Feedrate_Profile_Generator;
