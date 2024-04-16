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

with Ada.Containers.Synchronized_Queue_Interfaces;
with Ada.Containers.Bounded_Synchronized_Queues;
with Ada.Containers;
private with Motion_Planner.PH_Beziers;

generic
   type Flush_Extra_Data_Type is private;
   Flush_Extra_Data_Default : Flush_Extra_Data_Type;
   Initial_Position : Position;
   Home_Move_Minimum_Coast_Time : Time;
   with function Is_Homing_Move (Data : Flush_Extra_Data_Type) return Boolean;
   Max_Corners : Max_Corners_Type := 50_000;
   Preprocessor_Minimum_Move_Distance : Length := 0.001 * mm;
   Corner_Blender_Max_Computational_Error : Length := 0.001 * mm;
   Corner_Blender_Max_Secondary_Angle_To_Blend : Angle := 89.5 * deg;
   Input_Queue_Length : Ada.Containers.Count_Type := 1_000;
   Output_Queue_Length : Ada.Containers.Count_Type := 3;
package Motion_Planner.Planner is

   type Command_Kind is (Move_Kind, Flush_Kind, Flush_And_Reset_Position_Kind, Flush_And_Change_Parameters_Kind);

   type Command (Kind : Command_Kind := Move_Kind) is record
      case Kind is
         when Flush_Kind | Flush_And_Reset_Position_Kind | Flush_And_Change_Parameters_Kind =>
            Flush_Extra_Data : Flush_Extra_Data_Type;
            case Kind is
               when Flush_And_Reset_Position_Kind =>
                  Reset_Pos : Position;
               when Flush_And_Change_Parameters_Kind =>
                  New_Params : Kinematic_Parameters;
               when others =>
                  null;
            end case;
         when Move_Kind =>
            Pos      : Position;
            Feedrate : Velocity;
      end case;
   end record;

   type Corners_Index is new Max_Corners_Type'Base range 0 .. Max_Corners;

   task Runner is
      entry Setup (In_Params : Kinematic_Parameters);
   end Runner;

   type Execution_Block (N_Corners : Corners_Index := 0) is private;
   --  N_Corners may be 0 or 1, in which case there are no segments.

   --  First Finishing_Corner = 2. If N_Corners < 2 then these functions must not be called.

   function Segment_Time (Block : Execution_Block; Finishing_Corner : Corners_Index) return Time;
   --  Returns the total time for a given segment.

   function Segment_Corner_Distance (Block : Execution_Block; Finishing_Corner : Corners_Index) return Length;
   --  Returns the distance between the two original corners for a given segment.

   function Segment_Pos_At_Time
     (Block              :     Execution_Block;
      Finishing_Corner   :     Corners_Index;
      Time_Into_Segment  :     Time;
      Is_Past_Accel_Part : out Boolean)
     return Position;
   --  Returns the position at a given time in to a segment. Is_Past_Accel_Part indicates if the given time is past the
   --  acceleration part of the segment.

   function Next_Block_Pos (Block : Execution_Block) return Position;
   --  Returns the start position of the next block. At the end of a block, the motion executor should assume it is at
   --  this position, even if is not.

   function Flush_Extra_Data (Block : Execution_Block) return Flush_Extra_Data_Type;
   --  Return the data passed to the Enqueue procedure, or Flush_Extra_Data_Default if the block was filled before
   --  receiving a flush command.

   function Segment_Accel_Distance (Block : Execution_Block; Finishing_Corner : Corners_Index) return Length;
   --  Returns the length of the acceleration part of a segment.

   procedure Enqueue (Comm : Command; Ignore_Bounds : Boolean := False);
   --  Send a new command to the planner queue. May be called before Setup, but will block once the queue if full.

   procedure Dequeue (Block : out Execution_Block);
   --  Pop a block from the queue of processed blocks. Will block until a block is ready.

   Out_Of_Bounds_Error : exception;

private
   use Motion_Planner.PH_Beziers;

   --  Preprocessor
   type Block_Plain_Corners is array (Corners_Index range <>) of Scaled_Position;
   type Block_Segment_Feedrates is array (Corners_Index range <>) of Velocity;

   --  Corner_Blender
   type Block_Beziers is array (Corners_Index range <>) of PH_Bezier;

   --  Feedrate_Profile_Generator
   type Block_Feedrate_Profiles is array (Corners_Index range <>) of Feedrate_Profile;

   --  Kinematic_Limiter
   type Block_Corner_Velocity_Limits is array (Corners_Index range <>) of Velocity;

   type Execution_Block (N_Corners : Corners_Index := 0) is record
      --  TODO: Having all these fields accessible before the relevant stage is called is not ideal, but using a
      --  discriminated type with a discriminant to indicate the stage causes a stack overflow when trying to change
      --  the discriminant without making a copy as GCC tries to copy the whole thing to the stack. In the future we
      --  could possibly use SPARK to ensure stages do not touch fields that are not yet assigned.

      --  Having so many discriminated types here may seem like it will cause performance issues, but in practice it is
      --  faster than the same code without discriminated types (refer to the no-discriminated-records branch).

      --  Preprocessor
      Flush_Extra_Data  : Flush_Extra_Data_Type;
      Next_Block_Pos    : Scaled_Position;
      Params            : Kinematic_Parameters;
      Corners           : Block_Plain_Corners (1 .. N_Corners);  --  Adjusted with scaler.
      Segment_Feedrates : Block_Segment_Feedrates (2 .. N_Corners);  -- Adjusted with scaler.

      --  Corner_Blender
      Beziers : Block_Beziers (1 .. N_Corners);

      --  Kinematic_Limiter
      Corner_Velocity_Limits : Block_Corner_Velocity_Limits (1 .. N_Corners);

      --  Feedrate_Profile_Generator
      Feedrate_Profiles : Block_Feedrate_Profiles (2 .. N_Corners);
   end record;

   --  TODO: It might make sense to create a custom type here that can hold n bytes rather than n records so lots of
   --  small blocks may be queued if the planner is flushed often.
   package Execution_Block_Queues_Interface is new Ada.Containers.Synchronized_Queue_Interfaces (Execution_Block);
   package Execution_Block_Queues is new Ada.Containers.Bounded_Synchronized_Queues
     (Execution_Block_Queues_Interface, Output_Queue_Length);

   Execution_Block_Queue : access Execution_Block_Queues.Queue := new Execution_Block_Queues.Queue;

end Motion_Planner.Planner;
