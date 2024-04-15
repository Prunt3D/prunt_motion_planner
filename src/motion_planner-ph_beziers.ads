-----------------------------------------------------------------------------
--                                                                         --
--                   Part of the Prunt Motion Controller                   --
--                                                                         --
--            Copyright (C) 2024 Liam Powell (liam@prunt3d.com)            --
--                                                                         --
--  This program is free software: you can redistribute it and/or modify   --
--  it under the terms of the GNU General Public License as published by   --
--  the Free Software Foundation, either version 3 of the License, or      --
--  (at your option) any later version.                                    --
--                                                                         --
--  This program is distributed in the hope that it will be useful,        --
--  but WITHOUT ANY WARRANTY; without even the implied warranty of         --
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          --
--  GNU General Public License for more details.                           --
--                                                                         --
--  You should have received a copy of the GNU General Public License      --
--  along with this program.  If not, see <http://www.gnu.org/licenses/>.  --
--                                                                         --
-----------------------------------------------------------------------------

private package Motion_Planner.PH_Beziers is

   type PH_Bezier is private;

   function Distance_At_T (Bez : PH_Bezier; T : Dimensionless) return Length;
   function T_At_Distance (Bez : PH_Bezier; Distance : Length) return Dimensionless;
   function Inverse_Curvature (Bez : PH_Bezier) return Length;
   function Midpoint (Bez : PH_Bezier) return Scaled_Position;
   function Point_At_T (Bez : PH_Bezier; T : Dimensionless) return Scaled_Position;
   function Tangent_At_T (Bez : PH_Bezier; T : Dimensionless) return Scaled_Position_Offset;
   function Point_At_Distance (Bez : PH_Bezier; Distance : Length) return Scaled_Position;
   function Tangent_At_Distance (Bez : PH_Bezier; Distance : Length) return Scaled_Position_Offset;
   function Create_Bezier (Start, Corner, Finish : Scaled_Position; Deviation_Limit : Length) return PH_Bezier;

private

   type Control_Points_Index is range 0 .. 15;
   type PH_Control_Points is array (Control_Points_Index) of Scaled_Position;

   type PH_Bezier is record
      Control_Points    : PH_Control_Points;
      Inverse_Curvature : Length;
   end record;

end Motion_Planner.PH_Beziers;
