with Physical_Types; use Physical_Types;

package Motion_Planner is

   type Kinematic_Limits is record
      Velocity_Max     : Velocity;
      Acceleration_Max : Acceleration;
      Jerk_Max         : Jerk;
      Snap_Max         : Snap;
      Crackle_Max      : Crackle;
      Chord_Error_Max  : Length;
   end record;

   type Max_Corners_Type is range 2 .. 2**63 - 1;

   type Feedrate_Profile_Times_Index is range 1 .. 4;
   type Feedrate_Profile_Times is array (Feedrate_Profile_Times_Index) of Time;

   type Feedrate_Profile is tagged record
      Accel : Feedrate_Profile_Times;
      Coast : Time;
      Decel : Feedrate_Profile_Times;
   end record;

   --  A negative Start_Crackle value should be used in all of the below functions when working with a deceleration
   --  profile. This has the effect of negating the crackle graph shown in Feedrate_Profiles.ipynb.

   --  These two functions use a symbolically equivalent equation to X_At_Time where T is the time at the end of the
   --  feedrate profile. They may not be numerically identical to the functions that take T as an input but this does
   --  not cause issues with the current design of the motion planner.
   function Fast_Distance_At_Max_Time
     (Profile : Feedrate_Profile_Times; Start_Crackle : Crackle; Start_Vel : Velocity) return Length;
   function Fast_Velocity_At_Max_Time
     (Profile : Feedrate_Profile_Times; Start_Crackle : Crackle; Start_Vel : Velocity) return Velocity;

   function Total_Time (Times : Feedrate_Profile_Times) return Time;

   function Crackle_At_Time (Profile : Feedrate_Profile_Times; T : Time; Start_Crackle : Crackle) return Crackle;
   function Snap_At_Time (Profile : Feedrate_Profile_Times; T : Time; Start_Crackle : Crackle) return Snap;
   function Jerk_At_Time (Profile : Feedrate_Profile_Times; T : Time; Start_Crackle : Crackle) return Jerk;
   function Acceleration_At_Time
     (Profile : Feedrate_Profile_Times; T : Time; Start_Crackle : Crackle) return Acceleration;
   function Velocity_At_Time
     (Profile : Feedrate_Profile_Times; T : Time; Start_Crackle : Crackle; Start_Vel : Velocity) return Velocity;
   function Distance_At_Time
     (Profile : Feedrate_Profile_Times; T : Time; Start_Crackle : Crackle; Start_Vel : Velocity) return Length;

   function Total_Time (Profile : Feedrate_Profile) return Time;

   function Crackle_At_Time (Profile : Feedrate_Profile; T : Time; Start_Crackle : Crackle) return Crackle;
   function Snap_At_Time (Profile : Feedrate_Profile; T : Time; Start_Crackle : Crackle) return Snap;
   function Jerk_At_Time (Profile : Feedrate_Profile; T : Time; Start_Crackle : Crackle) return Jerk;
   function Acceleration_At_Time (Profile : Feedrate_Profile; T : Time; Start_Crackle : Crackle) return Acceleration;
   function Velocity_At_Time
     (Profile : Feedrate_Profile; T : Time; Start_Crackle : Crackle; Start_Vel : Velocity) return Velocity;
   function Distance_At_Time
     (Profile : Feedrate_Profile; T : Time; Start_Crackle : Crackle; Start_Vel : Velocity) return Length;

end Motion_Planner;
