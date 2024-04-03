with Physical_Types; use Physical_Types;

package Motion_Planner is

   --  type Scaled_Velocity_Limit is record
   --     Scale : Position_Scale := [others => 0.0];
   --     Vel   : Velocity := 0.0 * mm / s;
   --  end record;

   --  type Scaled_Velocities_Max_Array_Index is range 1 .. 8;
   --  type Scaled_Velocities_Max_Array is array (Scaled_Velocities_Max_Array_Index) of Scaled_Velocity_Limit;

   type Kinematic_Limits is record
      --  Scaled_Velocities_Max   : Scaled_Velocities_Max_Array;
      Acceleration_Max : Acceleration := 0.0 * mm / s**2;
      Jerk_Max         : Jerk         := 0.0 * mm / s**3;
      Snap_Max         : Snap         := 0.0 * mm / s**4;
      Crackle_Max      : Crackle      := 0.0 * mm / s**5;
      Chord_Error_Max  : Length       := 0.0 * mm;
   end record;

   type Max_Corners_Type is range 2 .. 2**63 - 1;

   type Feedrate_Profile_Times_Index is range 1 .. 4;
   type Feedrate_Profile_Times is array (Feedrate_Profile_Times_Index) of Time;

   type Feedrate_Profile is tagged record
      Accel : Feedrate_Profile_Times;
      Coast : Time;
      Decel : Feedrate_Profile_Times;
   end record;

   type Position_Offset_And_Scale is record
      Offset : Position_Offset;
      Scale  : Position_Scale;
   end record;

   function Fast_Distance_At_Max_Time
     (Profile : Feedrate_Profile_Times; Max_Crackle : Crackle; Start_Vel : Velocity) return Length;
   function Fast_Velocity_At_Max_Time
     (Profile : Feedrate_Profile_Times; Max_Crackle : Crackle; Start_Vel : Velocity) return Velocity;
   --  These two functions use a symbolically equivalent equation to X_At_Time where T is the time at the end of the
   --  feedrate profile. They may not be numerically identical to the functions that take T as an input but this does
   --  not cause issues with the current design of the motion planner.

   function Total_Time (Times : Feedrate_Profile_Times) return Time;

   function Crackle_At_Time (Profile : Feedrate_Profile_Times; T : Time; Max_Crackle : Crackle) return Crackle;
   function Snap_At_Time (Profile : Feedrate_Profile_Times; T : Time; Max_Crackle : Crackle) return Snap;
   function Jerk_At_Time (Profile : Feedrate_Profile_Times; T : Time; Max_Crackle : Crackle) return Jerk;
   function Acceleration_At_Time
     (Profile : Feedrate_Profile_Times; T : Time; Max_Crackle : Crackle) return Acceleration;
   function Velocity_At_Time
     (Profile : Feedrate_Profile_Times; T : Time; Max_Crackle : Crackle; Start_Vel : Velocity) return Velocity;
   function Distance_At_Time
     (Profile : Feedrate_Profile_Times; T : Time; Max_Crackle : Crackle; Start_Vel : Velocity) return Length;

   function Total_Time (Profile : Feedrate_Profile) return Time;

   function Crackle_At_Time (Profile : Feedrate_Profile; T : Time; Max_Crackle : Crackle) return Crackle;
   function Snap_At_Time (Profile : Feedrate_Profile; T : Time; Max_Crackle : Crackle) return Snap;
   function Jerk_At_Time (Profile : Feedrate_Profile; T : Time; Max_Crackle : Crackle) return Jerk;
   function Acceleration_At_Time (Profile : Feedrate_Profile; T : Time; Max_Crackle : Crackle) return Acceleration;
   function Velocity_At_Time
     (Profile : Feedrate_Profile; T : Time; Max_Crackle : Crackle; Start_Vel : Velocity) return Velocity;
   function Distance_At_Time
     (Profile : Feedrate_Profile; T : Time; Max_Crackle : Crackle; Start_Vel : Velocity) return Length;
   function Distance_At_Time
     (Profile            :     Feedrate_Profile;
      T                  :     Time;
      Max_Crackle        :     Crackle;
      Start_Vel          :     Velocity;
      Is_Past_Accel_Part : out Boolean)
      return Length;

   function Convert (Scaler : Position_Offset_And_Scale; Pos : Position) return Scaled_Position;
   function Convert (Scaler : Position_Offset_And_Scale; Pos : Scaled_Position) return Position;

end Motion_Planner;
