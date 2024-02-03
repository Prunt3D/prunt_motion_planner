with Ada.Numerics.Generic_Elementary_Functions;
with Ada.Unchecked_Conversion;
with Ada.Text_IO;
with Ada.Exceptions;

package body Motion_Planner.Planner is

   Working           : aliased Execution_Block;
   PP_Last_Pos       : Scaled_Position;
   PP_Corners        : Block_Plain_Corners (1 .. Corners_Index'Last);
   PP_Segment_Limits : Block_Segment_Limits (2 .. Corners_Index'Last);

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
                  PP_Last_Pos      := Next_Command.Reset_Pos;
                  N_Corners        := N_Corners - 1;
                  exit;
               when Move_Kind =>
                  if abs (PP_Last_Pos - Next_Command.Pos) < Preprocessor_Minimum_Move_Distance then
                     N_Corners := N_Corners - 1;
                  else
                     PP_Last_Pos                   := Next_Command.Pos;
                     PP_Corners (N_Corners)        := Next_Command.Pos;
                     PP_Segment_Limits (N_Corners) := Next_Command.Limits;
                  end if;
            end case;
         end;
      end loop;

      --  This is hacky and not portable, but if we try to assign to the entire record as you normally would then GCC
      --  insists on creating a whole Execution_Block on the stack.
      Working_N_Corners        := N_Corners;
      Working.Corners          := PP_Corners (1 .. N_Corners);
      Working.Segment_Limits   := PP_Segment_Limits (2 .. N_Corners);
      Working.Flush_Extra_Data := Flush_Extra_Data;
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
         Working.Shifted_Corner_Error_Limits (I) :=
           Length'Min (Working.Segment_Limits (I).Chord_Error_Max, Working.Segment_Limits (I + 1).Chord_Error_Max);
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
            --  A maximum of 24 hours should be more than enough unless you are using Prunt to control a space probe or
            --  a particle accelerator. It is not recommended to install Prunt on space probes or particle
            --  accelerators.
            Upper : Time := 86_400.0 * s;

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
               --  There are simple analytical solutions for a lot of these, but this is already fast so there is no
               --  reason to optimise it.
               return Solve_Distance_At_Time (Cases (I), I);
            end if;
         end loop;

         --  Unreachable.
         raise Program_Error;
      end Optimal_Accel_For_Distance;

   begin
      Working.Corner_Velocity_Limits (Working.Corner_Velocity_Limits'First) := 0.0 * mm / s;
      Working.Corner_Velocity_Limits (Working.Corner_Velocity_Limits'Last)  := 0.0 * mm / s;

      for I in Working.Corner_Velocity_Limits'First + 1 .. Working.Corner_Velocity_Limits'Last - 1 loop
         declare
            Limit           : Velocity;
            Optimal_Profile : Feedrate_Profile_Times;

            Inverse_Curvature : constant Length       := PH_Beziers.Inverse_Curvature (Working.Beziers (I));
            Velocity_Max      : constant Velocity     :=
              Velocity'Min (Working.Segment_Limits (I).Velocity_Max, Working.Segment_Limits (I + 1).Velocity_Max);
            Acceleration_Max  : constant Acceleration :=
              Acceleration'Min
                (Working.Segment_Limits (I).Acceleration_Max, Working.Segment_Limits (I + 1).Acceleration_Max);
            Jerk_Max          : constant Jerk         :=
              Jerk'Min (Working.Segment_Limits (I).Jerk_Max, Working.Segment_Limits (I + 1).Jerk_Max);
            Snap_Max          : constant Snap         :=
              Snap'Min (Working.Segment_Limits (I).Snap_Max, Working.Segment_Limits (I + 1).Snap_Max);
            Crackle_Max       : constant Crackle      :=
              Crackle'Min (Working.Segment_Limits (I).Crackle_Max, Working.Segment_Limits (I + 1).Crackle_Max);
         begin
            Limit := Velocity_Max;
            --  Inverse curvature range is 0..Length'Last, make sure to avoid overflow here.
            --  GCC with optimisation enabled may transform sqrt(x)*sqrt(y) to sqrt(x*y) etc., but that should be
            --  fine in optimised builds with Ada's checks disabled as the Velocity'Min call will immediately
            --  discard the resulting infinity.
            Limit := Velocity'Min (Limit, Acceleration_Max**(1 / 2) * Inverse_Curvature**(1 / 2));
            Limit := Velocity'Min (Limit, Jerk_Max**(1 / 3) * Inverse_Curvature**(2 / 3));
            Limit := Velocity'Min (Limit, Snap_Max**(1 / 4) * Inverse_Curvature**(3 / 4));
            Limit := Velocity'Min (Limit, Crackle_Max**(1 / 5) * Inverse_Curvature**(4 / 5));

            --  TODO: Add limit based on interpolation time.
            --  TODO: Snap and crackle limits currently do not match paper and are likely overly conservative.

            --  The 0.97 here ensures that no feedrate profiles end up with a very small accel/decel part which can
            --  lead to numerical errors that cause kinematic limits to be greatly exceeded for a single interpolation
            --  period. If this is removed, then the sanity check in Feedrate_Profile_Generator also needs to be
            --  removed.
            --  TODO: Check whether this actually matters in practice.
            Optimal_Profile :=
              Optimal_Accel_For_Distance
                (Working.Corner_Velocity_Limits (I - 1),
                 Curve_Corner_Distance (I - 1, I),
                 Working.Segment_Limits (I).Acceleration_Max,
                 Working.Segment_Limits (I).Jerk_Max,
                 Working.Segment_Limits (I).Snap_Max,
                 Working.Segment_Limits (I).Crackle_Max);
            Limit           :=
              Velocity'Min
                (Limit,
                 Fast_Velocity_At_Max_Time
                   (Optimal_Profile,
                    0.97 * Working.Segment_Limits (I).Crackle_Max,
                    Working.Corner_Velocity_Limits (I - 1)));

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
                 Working.Segment_Limits (I).Acceleration_Max,
                 Working.Segment_Limits (I).Jerk_Max,
                 Working.Segment_Limits (I).Snap_Max,
                 Working.Segment_Limits (I).Crackle_Max);
            Working.Corner_Velocity_Limits (I) :=
              Velocity'Min
                (Working.Corner_Velocity_Limits (I),
                 Fast_Velocity_At_Max_Time
                   (Optimal_Profile,
                    0.97 * Working.Segment_Limits (I).Crackle_Max,
                    Working.Corner_Velocity_Limits (I + 1)));
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
            --  A maximum of 24 hours should be more than enough unless you are using Prunt to control a space probe or
            --  a particle accelerator. It is not recommended to install Prunt on space probes or particle
            --  accelerators.
            Upper : Time := 86_400.0 * s;

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
                 Working.Segment_Limits (I).Acceleration_Max,
                 Working.Segment_Limits (I).Jerk_Max,
                 Working.Segment_Limits (I).Snap_Max,
                 Working.Segment_Limits (I).Crackle_Max);
            Accel_Profile_Distance : constant Length                 :=
              Fast_Distance_At_Max_Time
                (Profile, Working.Segment_Limits (I).Crackle_Max, Working.Corner_Velocity_Limits (I - 1));
            Decel_Profile_Distance : constant Length                 :=
              Fast_Distance_At_Max_Time
                (Profile, -Working.Segment_Limits (I).Crackle_Max, Working.Corner_Velocity_Limits (I - 1));
            Curve_Distance         : constant Length                 := Curve_Corner_Distance (I - 1, I);
         begin
            pragma Assert (Curve_Distance < Length'Min (Accel_Profile_Distance, Decel_Profile_Distance));
         end;

         Working.Feedrate_Profiles (I).Accel :=
           Optimal_Accel_For_Delta_V
             (Working.Corner_Velocity_Limits (I - 1) - Working.Segment_Limits (I).Velocity_Max,
              Working.Segment_Limits (I).Acceleration_Max,
              Working.Segment_Limits (I).Jerk_Max,
              Working.Segment_Limits (I).Snap_Max,
              Working.Segment_Limits (I).Crackle_Max);
         Working.Feedrate_Profiles (I).Decel :=
           Optimal_Accel_For_Delta_V
             (Working.Corner_Velocity_Limits (I) - Working.Segment_Limits (I).Velocity_Max,
              Working.Segment_Limits (I).Acceleration_Max,
              Working.Segment_Limits (I).Jerk_Max,
              Working.Segment_Limits (I).Snap_Max,
              Working.Segment_Limits (I).Crackle_Max);

         declare
            Accel_Distance : Length            :=
              Fast_Distance_At_Max_Time
                (Working.Feedrate_Profiles (I).Accel,
                 Working.Segment_Limits (I).Crackle_Max,
                 Working.Corner_Velocity_Limits (I - 1));
            Coast_Velocity : constant Velocity := Working.Segment_Limits (I).Velocity_Max;
            Decel_Distance : Length            :=
              Fast_Distance_At_Max_Time
                (Working.Feedrate_Profiles (I).Decel, -Working.Segment_Limits (I).Crackle_Max, Coast_Velocity);
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
                  Upper : Velocity := Working.Segment_Limits (I).Velocity_Max;
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
                          Working.Segment_Limits (I).Acceleration_Max,
                          Working.Segment_Limits (I).Jerk_Max,
                          Working.Segment_Limits (I).Snap_Max,
                          Working.Segment_Limits (I).Crackle_Max);
                     Working.Feedrate_Profiles (I).Decel :=
                       Optimal_Accel_For_Delta_V
                         (Working.Corner_Velocity_Limits (I) - Mid,
                          Working.Segment_Limits (I).Acceleration_Max,
                          Working.Segment_Limits (I).Jerk_Max,
                          Working.Segment_Limits (I).Snap_Max,
                          Working.Segment_Limits (I).Crackle_Max);

                     Accel_Distance :=
                       Fast_Distance_At_Max_Time
                         (Working.Feedrate_Profiles (I).Accel,
                          Working.Segment_Limits (I).Crackle_Max,
                          Working.Corner_Velocity_Limits (I - 1));
                     Decel_Distance :=
                       Fast_Distance_At_Max_Time
                         (Working.Feedrate_Profiles (I).Decel,
                          Working.Segment_Limits (I).Crackle_Max,
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
      loop
         Preprocessor;
         Corner_Blender;
         Kinematic_Limiter;
         Feedrate_Profile_Generator;
         Execution_Block_Queue.Enqueue (Working);
      end loop;
   exception
      when E : others =>
         Ada.Text_IO.Put_Line ("Exception in Motion.Planner:");
         Ada.Text_IO.Put_Line (Ada.Exceptions.Exception_Information (E));
   end Runner;

end Motion_Planner.Planner;
