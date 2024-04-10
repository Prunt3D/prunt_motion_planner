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

with Ada.Text_IO;
with Ada.Exceptions;
with Motion_Planner.Planner.Preprocessor;
with Motion_Planner.Planner.Corner_Blender;
with Motion_Planner.Planner.Kinematic_Limiter;
with Motion_Planner.Planner.Feedrate_Profile_Generator;

package body Motion_Planner.Planner is

   package My_Preprocessor is new Preprocessor;
   package My_Corner_Blender is new Corner_Blender;
   package My_Kinematic_Limiter is new Kinematic_Limiter;
   package My_Feedrate_Profile_Generator is new Feedrate_Profile_Generator;

   procedure Enqueue (Comm : Command) is
   begin
      My_Preprocessor.Enqueue (Comm);
   end Enqueue;

   procedure Dequeue (Block : out Execution_Block) is
   begin
      Execution_Block_Queue.Dequeue (Block);
   end Dequeue;

   task body Runner is
      type Block_Wrapper is record
         Block : aliased Execution_Block;
      end record;

      Working : access Block_Wrapper := new Block_Wrapper;
   begin
      accept Setup (In_Scaler : Position_Scale; In_Limits : Kinematic_Limits) do
         My_Preprocessor.Setup (In_Limits, (Offset => [others => Length (0.0)], Scale => In_Scaler));
      end Setup;

      loop
         My_Preprocessor.Run (Working.Block);

         if Is_Homing_Move (Working.Block.Flush_Extra_Data) and Working.Block.N_Corners /= 2 then
            raise Constraint_Error with "Homing move must have exactly 2 corners.";
         end if;

         My_Corner_Blender.Run (Working.Block);
         loop
            My_Kinematic_Limiter.Run (Working.Block);
            My_Feedrate_Profile_Generator.Run (Working.Block);

            exit when (not Is_Homing_Move (Working.Block.Flush_Extra_Data))
              or else Working.Block.Feedrate_Profiles (2).Coast >= Home_Move_Minimum_Coast_Time;

            Working.Block.Segment_Feedrates (2) := Working.Block.Segment_Feedrates (2) * 0.9;
         end loop;

         Execution_Block_Queue.Enqueue (Working.Block);
      end loop;
   exception
      when E : others =>
         Ada.Text_IO.Put_Line ("Exception in Motion_Planner.Planner:");
         Ada.Text_IO.Put_Line (Ada.Exceptions.Exception_Information (E));
   end Runner;

   function Segment_Time (Block : Execution_Block; Finishing_Corner : Corners_Index) return Time is
   begin
      return Total_Time (Block.Feedrate_Profiles (Finishing_Corner));
   end Segment_Time;

   function Segment_Corner_Distance (Block : Execution_Block; Finishing_Corner : Corners_Index) return Length is
   begin
      return
        abs
        (Convert (Block.Scaler, Block.Corners (Finishing_Corner)) -
         Convert (Block.Scaler, Block.Corners (Finishing_Corner - 1)));
   end Segment_Corner_Distance;

   function Segment_Pos_At_Time
     (Block              :     Execution_Block;
      Finishing_Corner   :     Corners_Index;
      Time_Into_Segment  :     Time;
      Is_Past_Accel_Part : out Boolean)
      return Position
   is
      Start_Curve_Half_Distance : constant Length :=
        Distance_At_T (Block.Beziers (Finishing_Corner - 1), 1.0) -
        Distance_At_T (Block.Beziers (Finishing_Corner - 1), 0.5);
      End_Curve_Half_Distance   : constant Length := Distance_At_T (Block.Beziers (Finishing_Corner), 0.5);
      Mid_Distance              : constant Length :=
        abs
        (Point_At_T (Block.Beziers (Finishing_Corner), 0.0) - Point_At_T (Block.Beziers (Finishing_Corner - 1), 1.0));

      Distance : Length :=
        Distance_At_Time
          (Block.Feedrate_Profiles (Finishing_Corner),
           Time_Into_Segment,
           Block.Limits.Crackle_Max,
           Block.Corner_Velocity_Limits (Finishing_Corner - 1),
           Is_Past_Accel_Part);

      Pos : Scaled_Position;
   begin
      if Distance < Start_Curve_Half_Distance then
         Pos := Point_At_Distance (Block.Beziers (Finishing_Corner - 1), Distance + Start_Curve_Half_Distance);
      elsif Distance < Start_Curve_Half_Distance + Mid_Distance then
         Pos :=
           Point_At_T (Block.Beziers (Finishing_Corner - 1), 1.0) +
           (Point_At_T (Block.Beziers (Finishing_Corner), 0.0) -
              Point_At_T (Block.Beziers (Finishing_Corner - 1), 1.0)) *
             ((Distance - Start_Curve_Half_Distance) / Mid_Distance);
      else
         Pos :=
           Point_At_Distance (Block.Beziers (Finishing_Corner), Distance - Start_Curve_Half_Distance - Mid_Distance);
      end if;

      return Convert (Block.Scaler, Pos);
   end Segment_Pos_At_Time;

   function Next_Block_Pos (Block : Execution_Block) return Position is
   begin
      return Convert (Block.Scaler, Block.Next_Block_Pos);
   end Next_Block_Pos;

   function Flush_Extra_Data (Block : Execution_Block) return Flush_Extra_Data_Type is
   begin
      return Block.Flush_Extra_Data;
   end Flush_Extra_Data;

   function Segment_Accel_Distance (Block : Execution_Block; Finishing_Corner : Corners_Index) return Length is
   begin
      return
        Distance_At_Time
          (Profile     => Block.Feedrate_Profiles (Finishing_Corner),
           T           => Total_Time (Block.Feedrate_Profiles (Finishing_Corner).Accel),
           Max_Crackle => Block.Limits.Crackle_Max,
           Start_Vel   => Block.Corner_Velocity_Limits (Finishing_Corner - 1));
   end Segment_Accel_Distance;

end Motion_Planner.Planner;
