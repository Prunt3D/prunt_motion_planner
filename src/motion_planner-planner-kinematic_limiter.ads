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
package Motion_Planner.Planner.Kinematic_Limiter is

   procedure Run (Block : in out Execution_Block);

private

   function Optimal_Profile_For_Distance
     (Start_Vel        : Velocity;
      Distance         : Length;
      Acceleration_Max : Acceleration;
      Jerk_Max         : Jerk;
      Snap_Max         : Snap;
      Crackle_Max      : Crackle)
     return Feedrate_Profile_Times;
   --  Compute the feedrate profile that has the lowest total time to travel the given distance without violating any
   --  of the given constraints. Note that there is no velocity limit here.

end Motion_Planner.Planner.Kinematic_Limiter;
