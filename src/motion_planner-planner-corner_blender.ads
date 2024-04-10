private generic
package Motion_Planner.Planner.Corner_Blender is

   procedure Run (Block : in out Execution_Block);

private

   type Shifted_Corner_Error_Limits_Type is array (Corners_Index) of Length;
   type Shifted_Corners_Type is array (Corners_Index) of Scaled_Position;

   Shifted_Corner_Error_Limits : access Shifted_Corner_Error_Limits_Type := new Shifted_Corner_Error_Limits_Type;
   Shifted_Corners             : access Shifted_Corners_Type             := new Shifted_Corners_Type;

   function Sine_Secondary_Angle (Start, Corner, Finish : Scaled_Position) return Dimensionless;
   --  Compute sin(x) where x is one of the two identical angles of the triangle formed by the normalised vectors
   --  (Start, Corner) and (Corner, Finish). The input vectors do not need to be normalised, this is done by the
   --  function.

   function Unit_Bisector (Start, Corner, Finish : Scaled_Position) return Position_Scale;
   --  Compute the unit vector that bisects the given corner.

end Motion_Planner.Planner.Corner_Blender;
