with Ada.Containers.Synchronized_Queue_Interfaces;
with Ada.Containers.Bounded_Synchronized_Queues;
with Ada.Containers;
with Motion_Planner.PH_Beziers; use Motion_Planner.PH_Beziers;

generic
   type Flush_Extra_Data_Type is private;
   Flush_Extra_Data_Default : Flush_Extra_Data_Type;
   Initial_Position : Scaled_Position;
   Max_Corners : Max_Corners_Type := 50_000;
   Preprocessor_Minimum_Move_Distance : Length := 0.001 * mm;
   Corner_Blender_Do_Shifting : Boolean := True;  --  TODO: Make this configurable at runtime.
   Corner_Blender_Max_Computational_Error : Length := 0.001 * mm;
   Corner_Blender_Max_Secondary_Angle_To_Blend : Angle := 89.5 * deg;
   Input_Queue_Length : Ada.Containers.Count_Type := 1_000;
   Output_Queue_Length : Ada.Containers.Count_Type := 3;
package Motion_Planner.Planner is

   type Command_Kind is (Move_Kind, Flush_Kind, Flush_And_Reset_Position_Kind);

   type Command (Kind : Command_Kind := Move_Kind) is record
      case Kind is
         when Flush_Kind | Flush_And_Reset_Position_Kind =>
            Flush_Extra_Data : Flush_Extra_Data_Type;
            case Kind is
               when Flush_And_Reset_Position_Kind =>
                  Reset_Pos : Scaled_Position;
               when others =>
                  null;
            end case;
         when Move_Kind =>
            Pos    : Scaled_Position;
            Limits : Kinematic_Limits;
      end case;
   end record;

   type Corners_Index is new Max_Corners_Type'Base range 0 .. Max_Corners;

   --  Preprocessor
   type Block_Plain_Corners is array (Corners_Index range <>) of Scaled_Position;
   type Block_Segment_Limits is array (Corners_Index range <>) of Kinematic_Limits;

   --  Corner_Blender
   type Block_Beziers is array (Corners_Index range <>) of PH_Bezier;
   type Block_Shifted_Corners is array (Corners_Index range <>) of Scaled_Position;
   type Block_Shifted_Corner_Error_Limits is array (Corners_Index range <>) of Length;

   --  Feedrate_Profile_Generator
   type Block_Feedrate_Profiles is array (Corners_Index range <>) of Feedrate_Profile;

   --  Kinematic_Limiter
   type Block_Corner_Velocity_Limits is array (Corners_Index range <>) of Velocity;

   --  N_Corners may be 0 or 1, in which case no movement should occur.
   type Execution_Block (N_Corners : Corners_Index := 0) is record
      --  TODO: Having all these fields accessible before the relevant stage is called is not ideal, but using a
      --  discriminated type with a discriminant to indicate the stage causes a stack overflow when trying to change
      --  the discriminant without making a copy as GCC tries to copy the whole thing to the stack. In the future we
      --  could possibly use SPARK to ensure stages do not touch fields that are not yet assigned.

      --  Having so many discriminated types here may seem like it will cause performance issues, but in practice it is
      --  faster than the same code without discriminated types (refer to the no-discriminated-records branch).

      --  Preprocessor
      Flush_Extra_Data : Flush_Extra_Data_Type;
      Corners          : Block_Plain_Corners (1 .. N_Corners);
      Segment_Limits   : Block_Segment_Limits (2 .. N_Corners);

      --  Corner_Blender
      Beziers                     : Block_Beziers (1 .. N_Corners);
      Shifted_Corners             : Block_Shifted_Corners (1 .. N_Corners);
      Shifted_Corner_Error_Limits : Block_Shifted_Corner_Error_Limits (1 .. N_Corners);

      --  Kinematic_Limiter
      Corner_Velocity_Limits : Block_Corner_Velocity_Limits (1 .. N_Corners);

      --  Feedrate_Profile_Generator
      Feedrate_Profiles : Block_Feedrate_Profiles (2 .. N_Corners);
   end record;

   procedure Enqueue (Comm : Command);
   procedure Dequeue (Block : out Execution_Block);

private

   package Command_Queues_Interface is new Ada.Containers.Synchronized_Queue_Interfaces (Command);
   package Command_Queues is new Ada.Containers.Bounded_Synchronized_Queues
     (Command_Queues_Interface, Input_Queue_Length);

   Command_Queue : Command_Queues.Queue;

   --  TODO: It might make sense to create a custom type here that can hold n bytes rather than n records so lots of
   --  small blocks may be queued if the planner is flushed often.
   package Execution_Block_Queues_Interface is new Ada.Containers.Synchronized_Queue_Interfaces (Execution_Block);
   package Execution_Block_Queues is new Ada.Containers.Bounded_Synchronized_Queues
     (Execution_Block_Queues_Interface, Output_Queue_Length);

   Execution_Block_Queue : Execution_Block_Queues.Queue;

   task Runner;

end Motion_Planner.Planner;
