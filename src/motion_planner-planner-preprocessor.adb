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

package body Motion_Planner.Planner.Preprocessor is

   procedure Enqueue (Comm : Command) is
   begin
      Command_Queue.Enqueue (Comm);
   end Enqueue;

   procedure Setup (Initial_Limits : Kinematic_Limits; Initial_Scaler : Position_Offset_And_Scale) is
   begin
      if Setup_Done then
         raise Constraint_Error with "Setup already called.";
      end if;

      Limits     := Initial_Limits;
      Scaler     := Initial_Scaler;
      Last_Pos   := Convert (Scaler, Initial_Position);
      Setup_Done := True;
   end Setup;

   procedure Run (Block : aliased out Execution_Block) is
      Flush_Extra_Data : Flush_Extra_Data_Type := Flush_Extra_Data_Default;
      N_Corners        : Corners_Index         := 1;
      Block_N_Corners  : Corners_Index with
        Address => Block.N_Corners'Address;
      Next_Limits      : Kinematic_Limits;
   begin
      if not Setup_Done then
         raise Constraint_Error with "Setup not done.";
      end if;

      Next_Limits := Limits;

      Corners (1) := Last_Pos;

      loop
         declare
            Next_Command : Command;
         begin
            Command_Queue.Dequeue (Next_Command);

            case Next_Command.Kind is
               when Flush_Kind =>
                  Flush_Extra_Data := Next_Command.Flush_Extra_Data;
                  exit;
               when Flush_And_Reset_Position_Kind =>
                  Flush_Extra_Data := Next_Command.Flush_Extra_Data;
                  Last_Pos         := Convert (Scaler, Next_Command.Reset_Pos);
                  exit;
               when Flush_And_Change_Limits_Kind =>
                  Flush_Extra_Data := Next_Command.Flush_Extra_Data;
                  Next_Limits      := Next_Command.New_Limits;
                  exit;
               when Move_Kind =>
                  if abs (Convert (Scaler, Last_Pos) - Next_Command.Pos) >= Preprocessor_Minimum_Move_Distance then
                     N_Corners           := N_Corners + 1;
                     Last_Pos            := Convert (Scaler, Next_Command.Pos);
                     Corners (N_Corners) := Convert (Scaler, Next_Command.Pos);

                     --  Adjust tangential velocity limit to account for scale.
                     declare
                        Unscaled_Distance : constant Length :=
                          abs (Convert (Scaler, Corners (N_Corners - 1)) - Convert (Scaler, Corners (N_Corners)));
                        Scaled_Distance   : constant Length := abs (Corners (N_Corners - 1) - Corners (N_Corners));
                     begin
                        if Unscaled_Distance = 0.0 * mm then
                           Segment_Feedrates (N_Corners) := Next_Command.Feedrate;
                        else
                           Segment_Feedrates (N_Corners) :=
                             Next_Command.Feedrate * (Scaled_Distance / Unscaled_Distance);
                        end if;
                     end;
                     exit when N_Corners = Corners_Index'Last;
                  end if;
            end case;
         end;
      end loop;

      --  This is hacky and not portable, but if we try to assign to the entire record as you normally would then GCC
      --  insists on creating a whole Execution_Block on the stack.
      Block_N_Corners         := N_Corners;
      Block.Corners           := Corners (1 .. N_Corners);
      Block.Segment_Feedrates := Segment_Feedrates (2 .. N_Corners);
      Block.Flush_Extra_Data  := Flush_Extra_Data;
      Block.Next_Block_Pos    := Last_Pos;
      Block.Scaler            := Scaler;
      Block.Limits            := Limits;

      Limits := Next_Limits;
   end Run;

end Motion_Planner.Planner.Preprocessor;
