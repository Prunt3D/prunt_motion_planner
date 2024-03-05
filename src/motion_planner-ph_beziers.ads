private package Motion_Planner.PH_Beziers is

   type PH_Bezier is private;

   function Distance_At_T (Bez : PH_Bezier; T : Dimensionless) return Length;
   function T_At_Distance (Bez : PH_Bezier; Distance : Length) return Dimensionless;
   function Inverse_Curvature (Bez : PH_Bezier) return Length;
   function Midpoint (Bez : PH_Bezier) return Scaled_Position;
   function Point_At_T (Bez : PH_Bezier; T : Dimensionless) return Scaled_Position;
   function Point_At_Distance (Bez : PH_Bezier; Distance : Length) return Scaled_Position;
   function Create_Bezier (Start, Corner, Finish : Scaled_Position; Deviation_Limit : Length) return PH_Bezier;

private

   type Control_Points_Index is range 0 .. 15;
   type PH_Control_Points is array (Control_Points_Index) of Scaled_Position;

   type PH_Bezier is record
      Control_Points    : PH_Control_Points;
      Inverse_Curvature : Length;
   end record;

end Motion_Planner.PH_Beziers;
