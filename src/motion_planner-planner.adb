with Ada.Numerics.Generic_Elementary_Functions;
with Ada.Unchecked_Conversion;
with Ada.Text_IO;
with Ada.Exceptions;

package body Motion_Planner.Planner is

   Working              : aliased Execution_Block;
   Scaler               : Position_Offset_And_Scale;
   Limits               : Kinematic_Limits;
   PP_Last_Pos          : Scaled_Position;
   PP_Corners           : Block_Plain_Corners (1 .. Corners_Index'Last);
   PP_Segment_Feedrates : Block_Segment_Feedrates (2 .. Corners_Index'Last);

   package Elementary_Functions is new Ada.Numerics.Generic_Elementary_Functions (Dimensioned_Float);
   use Elementary_Functions;

   function Curve_Corner_Distance (Start, Finish : Corners_Index) return Length is
   begin
      return
        Distance_At_T (Working.Beziers (Start), 0.5) + Distance_At_T (Working.Beziers (Finish), 1.0) -
        Distance_At_T (Working.Beziers (Finish), 0.5) +
        abs (Point_At_T (Working.Beziers (Start), 1.0) - Point_At_T (Working.Beziers (Finish), 0.0));
   end Curve_Corner_Distance;

   --  WARNING: This procedure is part of the public API and as such will be called from a different thread.
   --  Do not touch any of the package level variables.
   procedure Enqueue (Comm : Command) is
   begin
      Command_Queue.Enqueue (Comm);
   end Enqueue;

   --  WARNING: This procedure is part of the public API and as such will be called from a different thread.
   --  Do not touch any of the package level variables.
   procedure Dequeue (Block : out Execution_Block) is
   begin
      Execution_Block_Queue.Dequeue (Block);
   end Dequeue;

   procedure Preprocessor is
      Flush_Extra_Data  : Flush_Extra_Data_Type := Flush_Extra_Data_Default;
      N_Corners         : Corners_Index         := 1;
      Working_N_Corners : Corners_Index with
        Address => Working.N_Corners'Address;
   begin
      PP_Corners (1) := PP_Last_Pos;

      loop
         exit when N_Corners = Corners_Index'Last;
         N_Corners := N_Corners + 1;

         declare
            Next_Command : Command;
         begin
            Command_Queue.Dequeue (Next_Command);
            case Next_Command.Kind is
               when Flush_Kind =>
                  Flush_Extra_Data := Next_Command.Flush_Extra_Data;
                  N_Corners        := N_Corners - 1;
                  exit;
               when Flush_And_Reset_Position_Kind =>
                  Flush_Extra_Data := Next_Command.Flush_Extra_Data;
                  PP_Last_Pos      := Convert (Scaler, Next_Command.Reset_Pos);
                  N_Corners        := N_Corners - 1;
                  exit;
               when Move_Kind =>
                  if abs (Convert (Scaler, PP_Last_Pos) - Next_Command.Pos) < Preprocessor_Minimum_Move_Distance then
                     N_Corners := N_Corners - 1;
                  else
                     PP_Last_Pos                      := Convert (Scaler, Next_Command.Pos);
                     PP_Corners (N_Corners)           := Convert (Scaler, Next_Command.Pos);
                     PP_Segment_Feedrates (N_Corners) := Next_Command.Feedrate;
                  end if;
            end case;
         end;
      end loop;

      --  This is hacky and not portable, but if we try to assign to the entire record as you normally would then GCC
      --  insists on creating a whole Execution_Block on the stack.
      Working_N_Corners         := N_Corners;
      Working.Corners           := PP_Corners (1 .. N_Corners);
      Working.Segment_Feedrates := PP_Segment_Feedrates (2 .. N_Corners);
      Working.Flush_Extra_Data  := Flush_Extra_Data;
      Working.Next_Block_Pos    := PP_Last_Pos;
      --  TODO: In the future we can make these changeable per-block.
      Working.Scaler            := Scaler;
      Working.Limits            := Limits;
   end Preprocessor;

   procedure Corner_Blender is

      --  function Compute_Secondary_Angle (Start, Corner, Finish : Scaled_Position) return Angle is
      --     function Clamped_Arccos (X : Dimensionless) return Angle is
      --     begin
      --        if X < -1.0 then
      --           return Ada.Numerics.Pi;
      --        elsif X > 1.0 then
      --           return 0.0;
      --        else
      --           return Arccos (X);
      --        end if;
      --     end Clamped_Arccos;
      --
      --     V1              : constant Scaled_Position_Offset := Start - Corner;
      --     V2              : constant Scaled_Position_Offset := Finish - Corner;
      --     Dot_Product     : constant Dimensionless          := Dot (V1 / abs V1, V2 / abs V2);
      --     Primary_Angle   : constant Angle                  := Clamped_Arccos (Dot_Product);
      --     Secondary_Angle : constant Angle                  := (Ada.Numerics.Pi - Primary_Angle) / 2.0;
      --  begin
      --     return Secondary_Angle;
      --  end Compute_Secondary_Angle;

      function Compute_Sine_Secondary_Angle (Start, Corner, Finish : Scaled_Position) return Dimensionless is
         V1 : constant Scaled_Position_Offset := Start - Corner;
         V2 : constant Scaled_Position_Offset := Finish - Corner;
         A  : constant Area                   := Dot (V1, V2);
         B  : constant Area                   := 2.0 * (abs V1) * (abs V2);
      begin
         if 0.5 + A / B < 0.0 then
            return 0.0;
         elsif (0.5 + A / B)**(1 / 2) > 1.0 then
            return 1.0;
         else
            return (0.5 + A / B)**(1 / 2);
         end if;
      end Compute_Sine_Secondary_Angle;

      function Compute_Unit_Bisector (Start, Corner, Finish : Scaled_Position) return Position_Scale is
         A        : constant Scaled_Position_Offset := Start - Corner;
         B        : constant Scaled_Position_Offset := Finish - Corner;
         Bisector : constant Position_Scale         := A / abs A + B / abs B;
      begin
         if abs Bisector = 0.0 then
            return Bisector;
         else
            return Bisector / abs Bisector;
         end if;
      end Compute_Unit_Bisector;

      Last_Comp_Error : Length := 0.0 * mm;
   begin
      for I in Working.Corners'Range loop
         Working.Shifted_Corners (I) := Working.Corners (I);
      end loop;

      for I in Working.Shifted_Corner_Error_Limits'First + 1 .. Working.Shifted_Corner_Error_Limits'Last - 1 loop
         Working.Shifted_Corner_Error_Limits (I) := Working.Limits.Chord_Error_Max;
      end loop;
      Working.Shifted_Corner_Error_Limits (Working.Shifted_Corner_Error_Limits'First) := 0.0 * mm;
      Working.Shifted_Corner_Error_Limits (Working.Shifted_Corner_Error_Limits'Last)  := 0.0 * mm;

      Working.Beziers (Working.Beziers'First) :=
        Create_Bezier
          (Working.Corners (Working.Beziers'First),
           Working.Corners (Working.Beziers'First),
           Working.Corners (Working.Beziers'First),
           0.0 * mm);
      Working.Beziers (Working.Beziers'Last)  :=
        Create_Bezier
          (Working.Corners (Working.Beziers'Last),
           Working.Corners (Working.Beziers'Last),
           Working.Corners (Working.Beziers'Last),
           0.0 * mm);

      loop
         Last_Comp_Error := 0.0 * mm;

         for I in Working.Corners'First + 1 .. Working.Corners'Last - 1 loop
            if Sin (Corner_Blender_Max_Secondary_Angle_To_Blend) <
              Compute_Sine_Secondary_Angle (Working.Corners (I - 1), Working.Corners (I), Working.Corners (I + 1))
            then
               Working.Beziers (I) :=
                 Create_Bezier (Working.Corners (I), Working.Corners (I), Working.Corners (I), 0.0 * mm);
            else
               Working.Beziers (I) :=
                 Create_Bezier
                   (Working.Shifted_Corners (I - 1),
                    Working.Shifted_Corners (I),
                    Working.Shifted_Corners (I + 1),
                    Working.Shifted_Corner_Error_Limits (I));
               Last_Comp_Error     :=
                 Length'Max (Last_Comp_Error, abs (Midpoint (Working.Beziers (I)) - Working.Corners (I)));
            end if;
         end loop;

         exit when not Corner_Blender_Do_Shifting;
         exit when Last_Comp_Error <= Corner_Blender_Max_Computational_Error;

         for I in Working.Corners'First + 1 .. Working.Corners'Last - 1 loop
            Working.Shifted_Corners (I) := @ + (Working.Corners (I) - Midpoint (Working.Beziers (I)));
         end loop;

         for I in Working.Corners'First + 1 .. Working.Corners'Last - 1 loop
            declare
               Start  : constant Scaled_Position := Working.Shifted_Corners (I - 1);
               Corner : constant Scaled_Position := Working.Shifted_Corners (I);
               Finish : constant Scaled_Position := Working.Shifted_Corners (I + 1);
            begin
               Working.Shifted_Corner_Error_Limits (I) :=
                 abs Dot
                   (Working.Corners (I) - Working.Shifted_Corners (I), Compute_Unit_Bisector (Start, Corner, Finish));
            end;
         end loop;
      end loop;
   end Corner_Blender;

   procedure Kinematic_Limiter is

      function Optimal_Accel_For_Distance
        (Start_Vel        : Velocity;
         Distance         : Length;
         Acceleration_Max : Acceleration;
         Jerk_Max         : Jerk;
         Snap_Max         : Snap;
         Crackle_Max      : Crackle)
         return Feedrate_Profile_Times
      is
         D     : constant Length       := Distance;
         Vs    : constant Velocity     := Start_Vel;
         Am    : constant Acceleration := Acceleration_Max;
         Jm    : constant Jerk         := Jerk_Max;
         Sm    : constant Snap         := Snap_Max;
         Cm    : constant Crackle      := Crackle_Max;
         Cases : array (Feedrate_Profile_Times_Index) of Feedrate_Profile_Times;

         function Solve_Distance_At_Time
           (Profile : Feedrate_Profile_Times; Variable : Feedrate_Profile_Times_Index) return Feedrate_Profile_Times
         is
            Result : Feedrate_Profile_Times := Profile;

            Lower : Time := 0.0 * s;
            Upper : Time := 86_400.0 * s;
            --  A maximum of 24 hours should be more than enough unless you are using Prunt to control a space probe or
            --  a particle accelerator. It is not recommended to install Prunt on space probes or particle
            --  accelerators.

            type Casted_Time is mod 2**64;
            function Cast_Time is new Ada.Unchecked_Conversion (Time, Casted_Time);
            function Cast_Time is new Ada.Unchecked_Conversion (Casted_Time, Time);
         begin
            --  This probably breaks when not using IEEE 754 floats or on other weird systems, so try to check for
            --  that.
            pragma Assert (Time'Size = 64);
            pragma Assert (Casted_Time'Size = 64);
            pragma Assert (Cast_Time (86_400.0 * s) = 4_680_673_776_000_565_248);
            pragma Assert (Cast_Time (0.123_45 * s) = 4_593_559_930_647_147_132);

            loop
               Result (Variable) := Cast_Time (Cast_Time (Lower) + (Cast_Time (Upper) - Cast_Time (Lower)) / 2);
               exit when Lower = Result (Variable) or Upper = Result (Variable);
               if Fast_Distance_At_Max_Time (Result, Cm, Vs) <= D then
                  Lower := Result (Variable);
               else
                  Upper := Result (Variable);
               end if;
            end loop;

            return Result;
         end Solve_Distance_At_Time;

      begin
         --!pp off
         if Sm**2 < Jm * Cm then
            if Am >= Jm * (Jm / Sm + Sm / Cm) then
               Cases :=
               [
                  --  Reachable: Sm, Jm, Am
                  4 => [Sm / Cm, Jm / Sm - Sm / Cm, Am / Jm - Jm / Sm - Sm / Cm, 0.0 * s],
                  --  Reachable: Sm, Jm
                  3 => [Sm / Cm, Jm / Sm - Sm / Cm, 0.0 * s, 0.0 * s],
                  --  Reachable: Sm
                  2 => [Sm / Cm, 0.0 * s, 0.0 * s, 0.0 * s],
                  --  Reachable: None
                  1 => [0.0 * s, 0.0 * s, 0.0 * s, 0.0 * s]
               ];
            elsif Am >= 2.0 * Sm**3 / Cm**2 then
               Cases :=
               [
                  --  Reachable: Sm, Am
                  4 => [Sm / Cm, (0.25 * Sm**2 / Cm**2 + Am / Sm)**(1 / 2) - 1.5 * Sm / Cm, 0.0 * s, 0.0 * s],
                  --  Impossible case.
                  3 => [Sm / Cm, (0.25 * Sm**2 / Cm**2 + Am / Sm)**(1 / 2) - 1.5 * Sm / Cm, 0.0 * s, 0.0 * s],
                  --  Reachable: Sm
                  2 => [Sm / Cm, 0.0 * s, 0.0 * s, 0.0 * s],
                  --  Reachable: None
                  1 => [0.0 * s, 0.0 * s, 0.0 * s, 0.0 * s]
               ];
            else
               Cases :=
               [
                  --  Reachable: Am
                  4 => [(0.5 * Am / Cm)**(1 / 3), 0.0 * s, 0.0 * s, 0.0 * s],
                  --  Impossible case.
                  3 => [(0.5 * Am / Cm)**(1 / 3), 0.0 * s, 0.0 * s, 0.0 * s],
                  --  Impossible case.
                  2 => [(0.5 * Am / Cm)**(1 / 3), 0.0 * s, 0.0 * s, 0.0 * s],
                  --  Reachable: None
                  1 => [0.0 * s, 0.0 * s, 0.0 * s, 0.0 * s]
               ];
            end if;
         else
            if Am > 2.0 * Jm * (Jm / Cm)**(1 / 2) then
               Cases :=
               [
                  --  Reachable: Jm, Am
                  4 => [(Jm / Cm)**(1 / 2), 0.0 * s, Am / Jm - 2.0 * (Jm / Cm)**(1 / 2), 0.0 * s],
                  --  Reachable: Jm
                  3 => [(Jm / Cm)**(1 / 2), 0.0 * s, 0.0 * s, 0.0 * s],
                  --  Impossible case.
                  2 => [(Jm / Cm)**(1 / 2), 0.0 * s, 0.0 * s, 0.0 * s],
                  --  Reachable: None
                  1 => [0.0 * s, 0.0 * s, 0.0 * s, 0.0 * s]
               ];
            else
               Cases :=
               [
                  --  Reachable: Am
                  4 => [(Am / (2.0 * Cm))**(1 / 3), 0.0 * s, 0.0 * s, 0.0 * s],
                  --  Impossible case.
                  3 => [(Am / (2.0 * Cm))**(1 / 3), 0.0 * s, 0.0 * s, 0.0 * s],
                  --  Impossible case.
                  2 => [(Am / (2.0 * Cm))**(1 / 3), 0.0 * s, 0.0 * s, 0.0 * s],
                  --  Reachable: None
                  1 => [0.0 * s, 0.0 * s, 0.0 * s, 0.0 * s]
               ];
            end if;
         end if;
         --!pp on

         for I in reverse Cases'Range loop
            if I = Cases'First or D > Fast_Distance_At_Max_Time (Cases (I), Cm, Vs) then
               return Solve_Distance_At_Time (Cases (I), I);
               --  There are simple analytical solutions for a lot of these, but this is already fast so there is no
               --  reason to optimise it.
            end if;
         end loop;

         --  Unreachable.
         raise Program_Error;
      end Optimal_Accel_For_Distance;

   begin
      --  Adjust tangential velocity limit to account for scale.
      for I in Working.Segment_Feedrates'Range loop
         declare
            Unscaled_Distance : constant Length :=
              abs
              (Convert (Working.Scaler, Working.Shifted_Corners (I - 1)) -
               Convert (Working.Scaler, Working.Shifted_Corners (I)));
            Scaled_Distance   : constant Length := abs (Working.Shifted_Corners (I - 1) - Working.Shifted_Corners (I));
         begin
            if Unscaled_Distance /= 0.0 * mm then
               Working.Segment_Feedrates (I) := Working.Segment_Feedrates (I) * (Scaled_Distance / Unscaled_Distance);
            end if;
         end;
      end loop;

      Working.Corner_Velocity_Limits (Working.Corner_Velocity_Limits'First) := 0.0 * mm / s;
      Working.Corner_Velocity_Limits (Working.Corner_Velocity_Limits'Last)  := 0.0 * mm / s;

      for I in Working.Corner_Velocity_Limits'First + 1 .. Working.Corner_Velocity_Limits'Last - 1 loop
         declare
            Limit : Velocity := Velocity'Min (Working.Segment_Feedrates (I), Working.Segment_Feedrates (I + 1));
            Optimal_Profile : Feedrate_Profile_Times;

            Inverse_Curvature : constant Length := PH_Beziers.Inverse_Curvature (Working.Beziers (I));
         begin
            --  Inverse curvature range is 0..Length'Last, make sure to avoid overflow here.
            --  GCC with optimisation enabled may transform sqrt(x)*sqrt(y) to sqrt(x*y) etc., but that should be
            --  fine in optimised builds with Ada's checks disabled as the Velocity'Min call will immediately
            --  discard the resulting infinity.
            Limit := Velocity'Min (Limit, Working.Limits.Acceleration_Max**(1 / 2) * Inverse_Curvature**(1 / 2));
            Limit := Velocity'Min (Limit, Working.Limits.Jerk_Max**(1 / 3) * Inverse_Curvature**(2 / 3));
            Limit := Velocity'Min (Limit, Working.Limits.Snap_Max**(1 / 4) * Inverse_Curvature**(3 / 4));
            Limit := Velocity'Min (Limit, Working.Limits.Crackle_Max**(1 / 5) * Inverse_Curvature**(4 / 5));

            --  TODO: Add limit based on interpolation time.
            --  TODO: Snap and crackle limits currently do not match paper and are likely overly conservative.

            Optimal_Profile :=
              Optimal_Accel_For_Distance
                (Working.Corner_Velocity_Limits (I - 1),
                 Curve_Corner_Distance (I - 1, I),
                 Working.Limits.Acceleration_Max,
                 Working.Limits.Jerk_Max,
                 Working.Limits.Snap_Max,
                 Working.Limits.Crackle_Max);
            Limit           :=
              Velocity'Min
                (Limit,
                 Fast_Velocity_At_Max_Time
                   (Optimal_Profile, 0.97 * Working.Limits.Crackle_Max, Working.Corner_Velocity_Limits (I - 1)));
            --  The 0.97 here ensures that no feedrate profiles end up with a very small accel/decel part which can
            --  lead to numerical errors that cause kinematic limits to be greatly exceeded for a single interpolation
            --  period. If this is removed, then the sanity check in Feedrate_Profile_Generator also needs to be
            --  removed.
            --  TODO: Check whether this actually matters in practice.

            Working.Corner_Velocity_Limits (I) := Limit;
         end;
      end loop;

      for I in reverse Working.Corner_Velocity_Limits'First + 1 .. Working.Corner_Velocity_Limits'Last - 1 loop
         declare
            Optimal_Profile : Feedrate_Profile_Times;
         begin
            Optimal_Profile                    :=
              Optimal_Accel_For_Distance
                (Working.Corner_Velocity_Limits (I + 1),
                 Curve_Corner_Distance (I, I + 1),
                 Working.Limits.Acceleration_Max,
                 Working.Limits.Jerk_Max,
                 Working.Limits.Snap_Max,
                 Working.Limits.Crackle_Max);
            Working.Corner_Velocity_Limits (I) :=
              Velocity'Min
                (Working.Corner_Velocity_Limits (I),
                 Fast_Velocity_At_Max_Time
                   (Optimal_Profile, 0.97 * Working.Limits.Crackle_Max, Working.Corner_Velocity_Limits (I + 1)));
         end;
      end loop;
   end Kinematic_Limiter;

   procedure Feedrate_Profile_Generator is

      function Optimal_Accel_For_Delta_V
        (Delta_V          : Velocity;
         Acceleration_Max : Acceleration;
         Jerk_Max         : Jerk;
         Snap_Max         : Snap;
         Crackle_Max      : Crackle)
         return Feedrate_Profile_Times
      is
         Vd : constant Velocity     := abs Delta_V;
         Am : constant Acceleration := Acceleration_Max;
         Jm : constant Jerk         := Jerk_Max;
         Sm : constant Snap         := Snap_Max;
         Cm : constant Crackle      := Crackle_Max;

         function Solve_Velocity_At_Time
           (Profile  : Feedrate_Profile_Times;
            Variable : Feedrate_Profile_Times_Index;
            Target   : Velocity)
            return Feedrate_Profile_Times
         is
            Result : Feedrate_Profile_Times := Profile;

            Lower : Time := 0.0 * s;
            Upper : Time := 86_400.0 * s;
            --  A maximum of 24 hours should be more than enough unless you are using Prunt to control a space probe or
            --  a particle accelerator. It is not recommended to install Prunt on space probes or particle
            --  accelerators.

            type Casted_Time is mod 2**64;
            function Cast_Time is new Ada.Unchecked_Conversion (Time, Casted_Time);
            function Cast_Time is new Ada.Unchecked_Conversion (Casted_Time, Time);
         begin
            --  This probably breaks when not using IEEE 754 floats or on other weird systems, so try to check for
            --  that.
            pragma Assert (Time'Size = 64);
            pragma Assert (Casted_Time'Size = 64);
            pragma Assert (Cast_Time (86_400.0 * s) = 4_680_673_776_000_565_248);
            pragma Assert (Cast_Time (0.123_45 * s) = 4_593_559_930_647_147_132);

            loop
               Result (Variable) := Cast_Time (Cast_Time (Lower) + (Cast_Time (Upper) - Cast_Time (Lower)) / 2);
               exit when Lower = Result (Variable) or Upper = Result (Variable);
               if Fast_Velocity_At_Max_Time (Result, Cm, 0.0 * mm / s) <= Target then
                  Lower := Result (Variable);
               else
                  Upper := Result (Variable);
               end if;
            end loop;

            return Result;
         end Solve_Velocity_At_Time;

      begin
         --  This function is called a lot more than Optimal_Accel_For_Distance, so we use simple analytical solutions
         --  where they exist. In the one case where we resort to Solve_Velocity_At_Time, the analytical solution that
         --  Mathematica outputs involves a Cm**18, which is far outside the range of Dimensioned_Float for reasonable
         --  values of Cm.
         --
         --  For reference:
         --  ToRadicals[
         --    Solve[
         --      With[
         --        {T1 = Sm/Cm, T3 = 0, T4 = 0},
         --        v == Cm*T1*(T1 + T2)*(2*T1 + T2 + T3)*(4*T1 + 2*T2 + T3 + T4)
         --      ],
         --      T2,
         --      NonNegativeReals
         --    ]
         --  ]
         if Sm**2 < Jm * Cm then
            if Am >= Jm * (Jm / Sm + Sm / Cm) then
               if Vd > Am * (Am / Jm + Jm / Sm + Sm / Cm) then
                  --  Reachable: Sm, Jm, Am
                  return
                    [Sm / Cm, Jm / Sm - Sm / Cm, Am / Jm - Jm / Sm - Sm / Cm, Vd / Am - Am / Jm - Jm / Sm - Sm / Cm];
               elsif Vd > 2.0 * Jm * (Jm / Sm + Sm / Cm)**2 then
                  --  Reachable: Sm, Jm
                  return
                    [Sm / Cm,
                    Jm / Sm - Sm / Cm,
                    0.5 * ((Jm / Sm + Sm / Cm)**2 + 4.0 * Vd / Jm)**(1 / 2) - 1.5 * (Jm / Sm + Sm / Cm),
                    0.0 * s];
               elsif Vd > 8.0 * Sm**4 / Cm**3 then
                  --  Reachable: Sm
                  return Solve_Velocity_At_Time ([Sm / Cm, 0.0 * s, 0.0 * s, 0.0 * s], 2, Vd);
               else
                  --  Reachable: None
                  return [(0.125 * Vd / Cm)**(1 / 4), 0.0 * s, 0.0 * s, 0.0 * s];
               end if;
            elsif Am >= 2.0 * Sm**3 / Cm**2 then
               if Vd > Am * (2.0 * (0.25 * Sm**2 / Cm**2 + Am / Sm)**(1 / 2) + Sm / Cm) then
                  --  Reachable: Sm, Am
                  return
                    [Sm / Cm,
                    (0.25 * Sm**2 / Cm**2 + Am / Sm)**(1 / 2) - 1.5 * Sm / Cm,
                    0.0 * s,
                    Vd / Am - Sm / Cm - 2.0 * (0.25 * Sm**2 / Cm**2 + Am / Sm)**(1 / 2)];
               elsif Vd > 8.0 * Sm**4 / Cm**3 then
                  --  Reachable: Sm
                  return Solve_Velocity_At_Time ([Sm / Cm, 0.0 * s, 0.0 * s, 0.0 * s], 2, Vd);
               else
                  --  Reachable: None
                  return [(0.125 * Vd / Cm)**(1 / 4), 0.0 * s, 0.0 * s, 0.0 * s];
               end if;
            else
               if Vd > 8.0 * Cm * (0.5 * Am / Cm)**(4 / 3) then
                  --  Reachable: Am
                  return [(0.5 * Am / Cm)**(1 / 3), 0.0 * s, 0.0 * s, Vd / Am - 4.0 * (0.5 * Am / Cm)**(1 / 3)];
               else
                  --  Reachable: None
                  return [(0.125 * Vd / Cm)**(1 / 4), 0.0 * s, 0.0 * s, 0.0 * s];
               end if;
            end if;
         else
            if Am > 2.0 * Jm * (Jm / Cm)**(1 / 2) then
               if Vd > Am * (Am / Jm + 2.0 * (Jm / Cm)**(1 / 2)) then
                  --  Reachable: Jm, Am
                  return
                    [(Jm / Cm)**(1 / 2),
                    0.0 * s,
                    Am / Jm - 2.0 * (Jm / Cm)**(1 / 2),
                    Vd / Am - Am / Jm - 2.0 * (Jm / Cm)**(1 / 2)];
               elsif Vd > 8.0 * Jm**2 / Cm then
                  --  Reachable: Jm
                  return
                    [(Jm / Cm)**(1 / 2), 0.0 * s, (Jm / Cm + Vd / Jm)**(1 / 2) - 3.0 * (Jm / Cm)**(1 / 2), 0.0 * s];
               else
                  --  Reachable: None
                  return [(0.125 * Vd / Cm)**(1 / 4), 0.0 * s, 0.0 * s, 0.0 * s];
               end if;
            else
               if Vd > 8.0 * Cm * (0.5 * Am / Cm)**(4 / 3) then
                  --  Reachable: Am
                  return [(0.5 * Am / Cm)**(1 / 3), 0.0 * s, 0.0 * s, Vd / Am - 4.0 * (0.5 * Am / Cm)**(1 / 3)];
               else
                  --  Reachable: None
                  return [(0.125 * Vd / Cm)**(1 / 4), 0.0 * s, 0.0 * s, 0.0 * s];
               end if;
            end if;
         end if;
      end Optimal_Accel_For_Delta_V;

   begin
      for I in Working.Feedrate_Profiles'Range loop
         declare
            Profile                : constant Feedrate_Profile_Times :=
              Optimal_Accel_For_Delta_V
                (Working.Corner_Velocity_Limits (I - 1) - Working.Corner_Velocity_Limits (I),
                 Working.Limits.Acceleration_Max,
                 Working.Limits.Jerk_Max,
                 Working.Limits.Snap_Max,
                 Working.Limits.Crackle_Max);
            Accel_Profile_Distance : constant Length                 :=
              Fast_Distance_At_Max_Time (Profile, Working.Limits.Crackle_Max, Working.Corner_Velocity_Limits (I - 1));
            Decel_Profile_Distance : constant Length                 :=
              Fast_Distance_At_Max_Time (Profile, -Working.Limits.Crackle_Max, Working.Corner_Velocity_Limits (I - 1));
            Curve_Distance         : constant Length                 := Curve_Corner_Distance (I - 1, I);
         begin
            pragma Assert (Curve_Distance < Length'Min (Accel_Profile_Distance, Decel_Profile_Distance));
         end;

         Working.Feedrate_Profiles (I).Accel :=
           Optimal_Accel_For_Delta_V
             (Working.Corner_Velocity_Limits (I - 1) - Working.Segment_Feedrates (I),
              Working.Limits.Acceleration_Max,
              Working.Limits.Jerk_Max,
              Working.Limits.Snap_Max,
              Working.Limits.Crackle_Max);
         Working.Feedrate_Profiles (I).Decel :=
           Optimal_Accel_For_Delta_V
             (Working.Corner_Velocity_Limits (I) - Working.Segment_Feedrates (I),
              Working.Limits.Acceleration_Max,
              Working.Limits.Jerk_Max,
              Working.Limits.Snap_Max,
              Working.Limits.Crackle_Max);

         declare
            Accel_Distance : Length            :=
              Fast_Distance_At_Max_Time
                (Working.Feedrate_Profiles (I).Accel,
                 Working.Limits.Crackle_Max,
                 Working.Corner_Velocity_Limits (I - 1));
            Coast_Velocity : constant Velocity := Working.Segment_Feedrates (I);
            Decel_Distance : Length            :=
              Fast_Distance_At_Max_Time
                (Working.Feedrate_Profiles (I).Decel, -Working.Limits.Crackle_Max, Coast_Velocity);
            Curve_Distance : constant Length   := Curve_Corner_Distance (I - 1, I);
         begin
            if Accel_Distance + Decel_Distance <= Curve_Distance then
               Working.Feedrate_Profiles (I).Coast :=
                 (Curve_Distance - Accel_Distance - Decel_Distance) / Coast_Velocity;
            else
               Working.Feedrate_Profiles (I).Coast := 0.0 * s;
               declare
                  type Casted_Vel is mod 2**64;
                  function Cast_Vel is new Ada.Unchecked_Conversion (Velocity, Casted_Vel);
                  function Cast_Vel is new Ada.Unchecked_Conversion (Casted_Vel, Velocity);
                  Upper : Velocity := Working.Segment_Feedrates (I);
                  Lower : Velocity :=
                    Velocity'Max (Working.Corner_Velocity_Limits (I - 1), Working.Corner_Velocity_Limits (I));
                  Mid   : Velocity;
               begin
                  --  This probably breaks when not using IEEE 754 floats or on other weird systems, so try to check
                  --  for that.
                  pragma Assert (Velocity'Size = 64);
                  pragma Assert (Casted_Vel'Size = 64);
                  pragma Assert (Cast_Vel (86_400.0 * mm / s) = 4_680_673_776_000_565_248);
                  pragma Assert (Cast_Vel (0.123_45 * mm / s) = 4_593_559_930_647_147_132);

                  loop
                     Mid := Cast_Vel (Cast_Vel (Lower) + (Cast_Vel (Upper) - Cast_Vel (Lower)) / 2);
                     exit when Lower = Mid or Upper = Mid;

                     Working.Feedrate_Profiles (I).Accel :=
                       Optimal_Accel_For_Delta_V
                         (Working.Corner_Velocity_Limits (I - 1) - Mid,
                          Working.Limits.Acceleration_Max,
                          Working.Limits.Jerk_Max,
                          Working.Limits.Snap_Max,
                          Working.Limits.Crackle_Max);
                     Working.Feedrate_Profiles (I).Decel :=
                       Optimal_Accel_For_Delta_V
                         (Working.Corner_Velocity_Limits (I) - Mid,
                          Working.Limits.Acceleration_Max,
                          Working.Limits.Jerk_Max,
                          Working.Limits.Snap_Max,
                          Working.Limits.Crackle_Max);

                     Accel_Distance :=
                       Fast_Distance_At_Max_Time
                         (Working.Feedrate_Profiles (I).Accel,
                          Working.Limits.Crackle_Max,
                          Working.Corner_Velocity_Limits (I - 1));
                     Decel_Distance :=
                       Fast_Distance_At_Max_Time
                         (Working.Feedrate_Profiles (I).Decel,
                          Working.Limits.Crackle_Max,
                          Working.Corner_Velocity_Limits (I));

                     if Accel_Distance + Decel_Distance <= Curve_Distance then
                        Lower := Mid;
                     else
                        Upper := Mid;
                     end if;
                  end loop;
               end;
            end if;
         end;
      end loop;
   end Feedrate_Profile_Generator;

   task body Runner is
   begin
      accept Setup (In_Scaler : Position_Scale; In_Limits : Kinematic_Limits) do
         Scaler      := (Offset => [others => Length (0.0)], Scale => In_Scaler);
         Limits      := In_Limits;
         PP_Last_Pos := Convert (Scaler, Initial_Position);
      end Setup;

      loop
         Preprocessor;
         Corner_Blender;
         Kinematic_Limiter;
         Feedrate_Profile_Generator;
         Execution_Block_Queue.Enqueue (Working);
      end loop;
   exception
      when E : others =>
         Ada.Text_IO.Put_Line ("Exception in Motion_Planner.Planner:");
         Ada.Text_IO.Put_Line (Ada.Exceptions.Exception_Information (E));
   end Runner;

   --  WARNING: This procedure is part of the public API and as such will be called from a different thread.
   --  Do not touch any of the package level variables.
   function Segment_Time (Block : Execution_Block; Finishing_Corner : Corners_Index) return Time is
   begin
      return Total_Time (Block.Feedrate_Profiles (Finishing_Corner));
   end Segment_Time;

   --  WARNING: This procedure is part of the public API and as such will be called from a different thread.
   --  Do not touch any of the package level variables.
   function Segment_Corner_Distance (Block : Execution_Block; Finishing_Corner : Corners_Index) return Length is
   begin
      return
        abs
        (Convert (Block.Scaler, Block.Corners (Finishing_Corner)) -
         Convert (Block.Scaler, Block.Corners (Finishing_Corner - 1)));
   end Segment_Corner_Distance;

   --  WARNING: This procedure is part of the public API and as such will be called from a different thread.
   --  Do not touch any of the package level variables.
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
         Pos                :=
           Point_At_T (Block.Beziers (Finishing_Corner - 1), 1.0) +
           (Point_At_T (Block.Beziers (Finishing_Corner), 0.0) -
              Point_At_T (Block.Beziers (Finishing_Corner - 1), 1.0)) *
             ((Distance - Start_Curve_Half_Distance) / Mid_Distance);
      else
         Pos                :=
           Point_At_Distance (Block.Beziers (Finishing_Corner), Distance - Start_Curve_Half_Distance - Mid_Distance);
      end if;

      return Convert (Block.Scaler, Pos);
   end Segment_Pos_At_Time;

   --  WARNING: This procedure is part of the public API and as such will be called from a different thread.
   --  Do not touch any of the package level variables.
   function Next_Block_Pos (Block : Execution_Block) return Position is
   begin
      return Convert (Block.Scaler, Block.Next_Block_Pos);
   end Next_Block_Pos;

   --  WARNING: This procedure is part of the public API and as such will be called from a different thread.
   --  Do not touch any of the package level variables.
   function Flush_Extra_Data (Block : Execution_Block) return Flush_Extra_Data_Type is
   begin
      return Block.Flush_Extra_Data;
   end Flush_Extra_Data;

   --  WARNING: This procedure is part of the public API and as such will be called from a different thread.
   --  Do not touch any of the package level variables.
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
