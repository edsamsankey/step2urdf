"""Generate a test STEP assembly: base + upper arm + forearm + gripper (named parts, colors, nested placement)."""
import sys
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCP.gp import gp_Pnt, gp_Ax2, gp_Dir, gp_Trsf, gp_Vec
from OCP.TopLoc import TopLoc_Location
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ColorSurf
from OCP.TDataStd import TDataStd_Name
from OCP.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs

def box(x, y, z, dx, dy, dz):
    return BRepPrimAPI_MakeBox(gp_Pnt(x, y, z), dx, dy, dz).Shape()

def cyl(x, y, z, r, h, d=(0, 0, 1)):
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x, y, z), gp_Dir(*d)), r, h).Shape()

def fuse(a, b):
    return BRepAlgoAPI_Fuse(a, b).Shape()

# All dimensions in mm. Parts are modelled at the origin and placed with locations (like real CAD).
base = fuse(box(-60, -60, 0, 120, 120, 40), cyl(0, 0, 40, 30, 20))              # base plate + turret boss, joint1 axis z at (0,0)
upper = fuse(box(-20, -20, 0, 40, 40, 200), cyl(0, -25, 200, 22, 50, (0, 1, 0)))  # vertical arm with a pin along Y at top
fore = fuse(box(-15, -15, 0, 150, 30, 30), cyl(0, -15, 15, 15, 30, (0, 1, 0)))    # forearm, pivot pin along Y at its root
grip = box(0, -25, -10, 20, 50, 20)
screw = cyl(50, 50, 40, 4, 10)

doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
ct = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())

def part(shape, name, rgb):
    lab = st.AddShape(shape, False)
    TDataStd_Name.Set_s(lab, TCollection_ExtendedString(name))
    ct.SetColor(lab, Quantity_Color(*rgb, Quantity_TOC_RGB), XCAFDoc_ColorSurf)
    return lab

def loc(x, y, z):
    t = gp_Trsf(); t.SetTranslation(gp_Vec(x, y, z)); return TopLoc_Location(t)

L = {n: part(s, n, c) for n, s, c in [
    ("Base", base, (0.3, 0.3, 0.35)), ("UpperArm", upper, (0.9, 0.5, 0.1)),
    ("Forearm", fore, (0.2, 0.5, 0.9)), ("Gripper", grip, (0.8, 0.1, 0.1)), ("M8_Screw", screw, (0.7, 0.7, 0.7))]}

assy = st.NewShape(); TDataStd_Name.Set_s(assy, TCollection_ExtendedString("TestArm"))
sub = st.NewShape(); TDataStd_Name.Set_s(sub, TCollection_ExtendedString("ArmSubAssembly"))
st.AddComponent(assy, L["Base"], loc(0, 0, 0))
st.AddComponent(assy, L["M8_Screw"], loc(0, 0, 0))
st.AddComponent(assy, L["M8_Screw"], loc(-100, -100, 0))     # second instance of same part
st.AddComponent(sub, L["UpperArm"], loc(0, 0, 0))
st.AddComponent(sub, L["Forearm"], loc(0, -10, 200))
st.AddComponent(sub, L["Gripper"], loc(135, 0, 215))
st.AddComponent(assy, sub, loc(0, 0, 60))                      # nested assembly offset
st.UpdateAssemblies()

w = STEPCAFControl_Writer(); w.SetColorMode(True); w.SetNameMode(True)
w.Transfer(doc, STEPControl_AsIs)
out = sys.argv[1] if len(sys.argv) > 1 else "test_arm.step"
w.Write(out); print("wrote", out)
