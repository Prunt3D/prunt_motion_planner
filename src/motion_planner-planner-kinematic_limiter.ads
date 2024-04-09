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
