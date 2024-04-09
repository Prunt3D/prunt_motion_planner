private generic
package Motion_Planner.Planner.Feedrate_Profile_Generator is

   procedure Run (Block : in out Execution_Block);

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
