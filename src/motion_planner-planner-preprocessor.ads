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
