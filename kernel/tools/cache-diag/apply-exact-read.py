#!/usr/bin/env python3
"""Edits OCCT's BinTools_{CurveSet,Curve2dSet,SurfaceSet}.cxx so a read restores the written
directions and axis frames bit for bit (patch 0004). Usage: apply-exact-read.py DIR (edits in place).
Kept with the diagnostic so the experiment and the patch are the same edit."""
import re
import sys
from pathlib import Path

d = Path(sys.argv[1])

HELP3 = '''
// cadoc patch (kernel/build/patches, 0004): a read restores the directions that were
// written, bit for bit. Every direction BinTools writes is a gp_Dir's own components, already
// of unit length, and every axis placement is written with its X and Y directions. Reading
// used to normalise each direction again (SetCoord) and rebuild X and Y from the main and X
// directions, which can move their last bits: enough for exact HLR to flip an edge lying on a
// visibility boundary. The written components are kept only when they are already a unit
// vector, or what the rebuilt placement holds, to 1e-12; anything else is normalised as before.
static void restoreWrittenDir(gp_Dir& theD, const double theX, const double theY, const double theZ)
{
  if (std::abs(theX * theX + theY * theY + theZ * theZ - 1.0) <= 1.0e-12)
  {
    const_cast<gp_XYZ&>(theD.XYZ()).SetCoord(theX, theY, theZ);
  }
}

template <class Ax>
static void restoreWrittenFrame(Ax& theAx, const gp_Dir& theX, const gp_Dir& theY)
{
  if (theAx.XDirection().IsEqual(theX, 1.0e-12) && theAx.YDirection().IsEqual(theY, 1.0e-12))
  {
    const_cast<gp_Dir&>(theAx.XDirection()) = theX;
    const_cast<gp_Dir&>(theAx.YDirection()) = theY;
  }
}
'''

HELP2 = '''
// cadoc patch (kernel/build/patches, 0004): a read restores the directions that were
// written, bit for bit (see BinTools_CurveSet.cxx). The written components are kept only when
// they are already a unit vector, or what the rebuilt placement holds, to 1e-12; anything else
// is normalised as before.
static void restoreWrittenDir(gp_Dir2d& theD, const double theX, const double theY)
{
  if (std::abs(theX * theX + theY * theY - 1.0) <= 1.0e-12)
  {
    const_cast<gp_XY&>(theD.XY()).SetCoord(theX, theY);
  }
}

static gp_Ax22d exactAx22d(const gp_Pnt2d& theP, const gp_Dir2d& theX, const gp_Dir2d& theY)
{
  gp_Ax22d anAx(theP, theX, theY);
  if (anAx.YDirection().IsEqual(theY, 1.0e-12))
  {
    const_cast<gp_Dir2d&>(anAx.YDirection()) = theY;
  }
  return anAx;
}
'''


def insert_helpers(src: str, helpers: str) -> str:
    # after the last #include
    idx = [m.end() for m in re.finditer(r"^#include[^\n]*\n", src, re.M)][-1]
    return src[:idx] + "\n#include <cmath>\n" + helpers + src[idx:]


def sub1(src: str, old: str, new: str, count: int = 1) -> str:
    n = src.count(old)
    if n != count:
        raise SystemExit(f"expected {count} of {old!r}, found {n}")
    return src.replace(old, new)


# 3D curves
p = d / "BinTools_CurveSet.cxx"
s = p.read_text()
s = insert_helpers(s, HELP3)
s = sub1(s, "  D.SetCoord(X, Y, Z);\n  return IS;", "  D.SetCoord(X, Y, Z);\n  restoreWrittenDir(D, X, Y, Z);\n  return IS;")
for cls in ["Circle(gp_Ax2(P, A, AX), R)", "Ellipse(gp_Ax2(P, A, AX), R1, R2)", "Parabola(gp_Ax2(P, A, AX), R1)", "Hyperbola(gp_Ax2(P, A, AX), R1, R2)"]:
    name = cls.split("(")[0]
    var = {"Circle": "C", "Ellipse": "E", "Parabola": "C", "Hyperbola": "H"}[name]
    args = cls[len(name) + 1 + len("gp_Ax2(P, A, AX)"):-1]
    s = sub1(s, f"  {var} = new Geom_{cls};", f"  gp_Ax2 anAx(P, A, AX);\n  restoreWrittenFrame(anAx, AX, AY);\n  {var} = new Geom_{name}(anAx{args});")
p.write_text(s)

# surfaces
p = d / "BinTools_SurfaceSet.cxx"
s = p.read_text()
s = insert_helpers(s, HELP3)
s = sub1(s, "  D.SetCoord(X, Y, Z);\n  return IS;", "  D.SetCoord(X, Y, Z);\n  restoreWrittenDir(D, X, Y, Z);\n  return IS;")
s = sub1(s, "    ax3.YReverse();\n  }\n  A3 = ax3;", "    ax3.YReverse();\n  }\n  restoreWrittenFrame(ax3, AX, AY);\n  A3 = ax3;")
p.write_text(s)

# 2D curves
p = d / "BinTools_Curve2dSet.cxx"
s = p.read_text()
s = insert_helpers(s, HELP2)
s = sub1(s, "  D.SetCoord(X, Y);\n  return IS;", "  D.SetCoord(X, Y);\n  restoreWrittenDir(D, X, Y);\n  return IS;")
s = sub1(s, "gp_Ax22d(P, AX, AY)", "exactAx22d(P, AX, AY)", count=4)
p.write_text(s)
print("patched", d)

# ---- locations: format version 5 writes each elementary location exactly (form, scale, matrix,
# translation) as record type 3; reading restores the fields as written. Version 4 and earlier are
# read and written as before, and BinTools_FormatVersion_CURRENT stays 4.

p = d / "BinTools_FormatVersion.hxx"
s = p.read_text()
s = sub1(s, '''  BinTools_FormatVersion_VERSION_4 = 4, //!< Stores per-vertex normal information in case
                                        //!  of triangulation-only Faces, because
                                        //!  no analytical geometry to restore normals
  BinTools_FormatVersion_CURRENT = BinTools_FormatVersion_VERSION_4 //!< Current version''', '''  BinTools_FormatVersion_VERSION_4 = 4, //!< Stores per-vertex normal information in case
                                        //!  of triangulation-only Faces, because
                                        //!  no analytical geometry to restore normals
  BinTools_FormatVersion_VERSION_5 = 5, //!< (cadoc patch 0004) As version 4, and stores each
                                        //!  location's transformation exactly (form, scale
                                        //!  factor, matrix and translation), so a write-read
                                        //!  round trip restores it bit for bit
  BinTools_FormatVersion_CURRENT = BinTools_FormatVersion_VERSION_4 //!< Current version''')
s = sub1(s, "  BinTools_FormatVersion_UPPER = BinTools_FormatVersion_VERSION_4", "  BinTools_FormatVersion_UPPER = BinTools_FormatVersion_VERSION_5")
p.write_text(s)

p = d / "BinTools_ShapeSetBase.cxx"
s = p.read_text()
s = sub1(s, '''  "Open CASCADE Topology V4, (c) Open Cascade"};''', '''  "Open CASCADE Topology V4, (c) Open Cascade",
  "Open CASCADE Topology V5, (c) Open Cascade"};''')
p.write_text(s)

p = d / "BinTools_LocationSet.hxx"
s = p.read_text()
s = sub1(s, '''  //! Reads the content of me from the stream <IS>. me
  //! is first cleared.
  Standard_EXPORT void Read(Standard_IStream& IS);

private:
  NCollection_IndexedMap<TopLoc_Location> myMap;''', '''  //! Reads the content of me from the stream <IS>. me
  //! is first cleared.
  Standard_EXPORT void Read(Standard_IStream& IS);

  //! The format version written (BinTools_FormatVersion): from version 5 each elementary
  //! location is written exactly (cadoc patch 0004). Set by BinTools_ShapeSet.
  void SetFormatNb(const int theFormatNb) { myFormatNb = theFormatNb; }

private:
  NCollection_IndexedMap<TopLoc_Location> myMap;
  int                                     myFormatNb = 4;''')
p.write_text(s)

p = d / "BinTools_LocationSet.cxx"
s = p.read_text()
s = sub1(s, '''//=================================================================================================
// function : operator << (gp_Trsf& T)''', '''// cadoc patch (kernel/build/patches, 0004): a transformation written exactly (format version 5,
// location record type 3): its form, its scale factor, its matrix without the scale factor and
// its translation. Type 1 writes the matrix multiplied by the scale factor and no form, and its
// reading (gp_Trsf::SetValues) re-derives the scale from the determinant, re-orthogonalises the
// matrix and sets the form to gp_CompoundTrsf: a few ulps off, and a different code path in
// gp_Trsf::Multiply. Reading type 3 sets the fields as written.
static void writeExact(Standard_OStream& OS, const gp_Trsf& T)
{
  BinTools::PutInteger(OS, (int)T.Form());
  BinTools::PutReal(OS, T.ScaleFactor());
  const gp_Mat& M = T.HVectorialPart();
  for (int r = 1; r <= 3; ++r)
    for (int c = 1; c <= 3; ++c)
      BinTools::PutReal(OS, M(r, c));
  const gp_XYZ& V = T.TranslationPart();
  BinTools::PutReal(OS, V.X());
  BinTools::PutReal(OS, V.Y());
  BinTools::PutReal(OS, V.Z());
}

static void readExact(Standard_IStream& IS, gp_Trsf& T)
{
  int    aForm  = 0;
  double aScale = 1.0;
  BinTools::GetInteger(IS, aForm);
  BinTools::GetReal(IS, aScale);
  gp_Mat M;
  for (int r = 1; r <= 3; ++r)
    for (int c = 1; c <= 3; ++c)
    {
      double v = 0.0;
      BinTools::GetReal(IS, v);
      M(r, c) = v;
    }
  double X = 0.0, Y = 0.0, Z = 0.0;
  BinTools::GetReal(IS, X);
  BinTools::GetReal(IS, Y);
  BinTools::GetReal(IS, Z);
  if (aForm < (int)gp_Identity || aForm > (int)gp_Other)
  {
    throw Standard_Failure("BinTools_LocationSet::Read: unknown transformation form");
  }
  T = gp_Trsf();
  T.SetScaleFactor(aScale);
  const_cast<gp_Mat&>(T.HVectorialPart()) = M;
  T.SetTranslationPart(gp_Vec(X, Y, Z));
  T.SetForm((gp_TrsfForm)aForm);
}

//=================================================================================================
// function : operator << (gp_Trsf& T)''')
s = sub1(s, '''      if (elementary)
      {

        OS.put((uint8_t)1); // 1
        OS << L.Transformation();
      }''', '''      if (elementary && myFormatNb >= 5)
      {
        OS.put((uint8_t)3); // 3: exact (cadoc patch 0004)
        writeExact(OS, L.Transformation());
      }
      else if (elementary)
      {

        OS.put((uint8_t)1); // 1
        OS << L.Transformation();
      }''')
s = sub1(s, '''      if (aTypLoc == 1)
      {
        IS >> T;
        L = T;
      }
''', '''      if (aTypLoc == 1)
      {
        IS >> T;
        L = T;
      }
      else if (aTypLoc == 3)
      {
        readExact(IS, T);
        L = T;
      }
''')
p.write_text(s)

p = d / "BinTools_ShapeSet.cxx"
s = p.read_text()
s = sub1(s, '''  OS << "\\n" << THE_ASCII_VERSIONS[FormatNb()] << "\\n";

  //-----------------------------------------
  // write the locations
  //-----------------------------------------

  myLocations.Write(OS);''', '''  OS << "\\n" << THE_ASCII_VERSIONS[FormatNb()] << "\\n";

  //-----------------------------------------
  // write the locations
  //-----------------------------------------

  myLocations.SetFormatNb(FormatNb());
  myLocations.Write(OS);''')
s = sub1(s, '''  //-----------------------------------------
  // read the locations
  //-----------------------------------------
  myLocations.Read(IS);''', '''  //-----------------------------------------
  // read the locations
  //-----------------------------------------
  myLocations.SetFormatNb(FormatNb());
  myLocations.Read(IS);''')
p.write_text(s)
print("patched locations", d)
