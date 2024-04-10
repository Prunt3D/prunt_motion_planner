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
package Motion_Planner.Planner.Preprocessor is

   procedure Enqueue (Comm : Command);

   procedure Setup (Initial_Limits : Kinematic_Limits; Initial_Scaler : Position_Offset_And_Scale);

   procedure Run (Block : aliased out Execution_Block);

private

   package Command_Queues_Interface is new Ada.Containers.Synchronized_Queue_Interfaces (Command);
   package Command_Queues is new Ada.Containers.Bounded_Synchronized_Queues
     (Command_Queues_Interface, Input_Queue_Length);

   Command_Queue : access Command_Queues.Queue := new Command_Queues.Queue;

   Last_Pos   : Scaled_Position;
   Limits     : Kinematic_Limits;
   Scaler     : Position_Offset_And_Scale;
   Setup_Done : Boolean := False;

   Corners           : access Block_Plain_Corners     := new Block_Plain_Corners (1 .. Corners_Index'Last);
   Segment_Feedrates : access Block_Segment_Feedrates := new Block_Segment_Feedrates (2 .. Corners_Index'Last);

end Motion_Planner.Planner.Preprocessor;
