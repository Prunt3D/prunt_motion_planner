with Ada.Numerics.Generic_Elementary_Functions;

package body Motion_Planner.Planner.Corner_Blender is

   package Angle_Elementary_Functions is new Ada.Numerics.Generic_Elementary_Functions (Angle);

   procedure Run (Block : in out Execution_Block) is
      Last_Comp_Error : Length := 0.0 * mm;
   begin
      for I in Block.Corners'Range loop
         Shifted_Corners (I) := Block.Corners (I);
      end loop;

      for I in Block.Corners'First + 1 .. Block.Corners'Last - 1 loop
         Shifted_Corner_Error_Limits (I) := Block.Limits.Chord_Error_Max;
      end loop;
      Shifted_Corner_Error_Limits (Block.Corners'First) := 0.0 * mm;
      Shifted_Corner_Error_Limits (Block.Corners'Last)  := 0.0 * mm;

      Block.Beziers (Block.Beziers'First) :=
        Create_Bezier
          (Block.Corners (Block.Beziers'First),
           Block.Corners (Block.Beziers'First),
           Block.Corners (Block.Beziers'First),
           0.0 * mm);
      Block.Beziers (Block.Beziers'Last)  :=
        Create_Bezier
          (Block.Corners (Block.Beziers'Last),
           Block.Corners (Block.Beziers'Last),
           Block.Corners (Block.Beziers'Last),
           0.0 * mm);

      loop
         Last_Comp_Error := 0.0 * mm;

         for I in Block.Corners'First + 1 .. Block.Corners'Last - 1 loop
            if Angle_Elementary_Functions.Cos (Corner_Blender_Min_Corner_Angle_To_Blend) <
              Cosine_Corner_Angle (Block.Corners (I - 1), Block.Corners (I), Block.Corners (I + 1))
            then
               Block.Beziers (I) := Create_Bezier (Block.Corners (I), Block.Corners (I), Block.Corners (I), 0.0 * mm);
            else
               Block.Beziers (I) :=
                 Create_Bezier
                   (Shifted_Corners (I - 1),
                    Shifted_Corners (I),
                    Shifted_Corners (I + 1),
                    Shifted_Corner_Error_Limits (I));
               Last_Comp_Error := Length'Max (Last_Comp_Error, abs (Midpoint (Block.Beziers (I)) - Block.Corners (I)));
            end if;
         end loop;

         exit when not Corner_Blender_Do_Shifting;
         exit when Last_Comp_Error <= Corner_Blender_Max_Computational_Error;

         for I in Block.Corners'First + 1 .. Block.Corners'Last - 1 loop
            Shifted_Corners (I) := @ + (Block.Corners (I) - Midpoint (Block.Beziers (I)));
         end loop;

         for I in Block.Corners'First + 1 .. Block.Corners'Last - 1 loop
            declare
               Start  : constant Scaled_Position := Shifted_Corners (I - 1);
               Corner : constant Scaled_Position := Shifted_Corners (I);
               Finish : constant Scaled_Position := Shifted_Corners (I + 1);
            begin
               Shifted_Corner_Error_Limits (I) :=
                 abs Dot (Block.Corners (I) - Shifted_Corners (I), Unit_Bisector (Start, Corner, Finish));
            end;
         end loop;
      end loop;
   end Run;

   function Sine_Secondary_Angle (Start, Corner, Finish : Scaled_Position) return Dimensionless is
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
   end Sine_Secondary_Angle;

   function Cosine_Corner_Angle (Start, Corner, Finish : Scaled_Position) return Dimensionless is
      V1 : constant Scaled_Position_Offset := Start - Corner;
      V2 : constant Scaled_Position_Offset := Finish - Corner;
   begin
      return Dot (V1 / abs V1, V2 / abs V2);
   end Cosine_Corner_Angle;

   function Unit_Bisector (Start, Corner, Finish : Scaled_Position) return Position_Scale is
      A        : constant Scaled_Position_Offset := Start - Corner;
      B        : constant Scaled_Position_Offset := Finish - Corner;
      Bisector : constant Position_Scale         := A / abs A + B / abs B;
   begin
      if abs Bisector = 0.0 then
         return Bisector;
      else
         return Bisector / abs Bisector;
      end if;
   end Unit_Bisector;

end Motion_Planner.Planner.Corner_Blender;
