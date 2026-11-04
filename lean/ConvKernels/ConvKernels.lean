import LeanSlang
/-!
# `ConvKernels`: 2D convolution as plain Slang functions, for training in PyTorch

Three pure functions over `TensorView<float>` (NCHW input, OIHW kernel), each computing one output element, so a thin
`[AutoPyBindCUDA]` wrapper kernel (hand-written, `conv_wrap.slang`) can launch one thread per element through `slangtorch`:

* `conv_fwd_at`      y[n,oc,oy,ox] = b[oc] + sum over ic,ky,kx of x[n,ic,iy,ix] * w[oc,ic,ky,kx],  iy = oy*s + ky - p
* `conv_bwd_input_at`  dx[n,ic,iy,ix] = sum over oc,ky,kx of dy[n,oc,oy,ox] * w[oc,ic,ky,kx],  where oy*s + ky - p = iy
* `conv_bwd_weight_at` dw[oc,ic,ky,kx] = sum over n,oy,ox of dy[n,oc,oy,ox] * x[n,ic,iy,ix]

The bias gradient is a plain sum of dy and stays in PyTorch. Sizes are `uint`, positions are computed in `int` so padding
never wraps. Gates (below): the emitted text names all three functions once each, and each is a plain function (no entry attribute).
-/
namespace ConvKernels
open LeanSlang

private def U : SlangType := .scalar .uint
private def I : SlangType := .scalar .int
private def F : SlangType := .scalar .float
private def TV : SlangType := .named "TensorView<float>"
private def v (n : String) : SlangExpr := .var n
private def u (n : Nat) : SlangExpr := .litUint n
private def i0 : SlangExpr := .litInt 0
private def bin (op : String) (a b : SlangExpr) : SlangExpr := .bin op a b
private def add := bin "+"
private def sub := bin "-"
private def mul := bin "*"
private def toI (e : SlangExpr) : SlangExpr := .cast I e
private def toU (e : SlangExpr) : SlangExpr := .cast U e
private def idx4 (t : String) (a b c d : SlangExpr) : SlangExpr := .index (v t) (.call "uint4" [a, b, c, d])
private def param (n : String) (t : SlangType) : SlangBinding := { name := n, type := t }
/-- `0 <= e < bound` for a signed position `e`. -/
private def inside (e bound : SlangExpr) : SlangExpr := bin "&&" (bin ">=" e i0) (bin "<" e (toI bound))
/-- The signed input position `o*stride + k - pad`. -/
private def pos (o k : String) : SlangExpr := sub (toI (add (mul (v o) (v "stride")) (v k))) (toI (v "pad"))

def convFwdAt : SlangFunctionDecl :=
  { retType := F, name := "conv_fwd_at"
    params := [param "x" TV, param "w" TV, param "b" TV, param "n" U, param "oc" U, param "oy" U, param "ox" U,
               param "cin" U, param "kh" U, param "kw" U, param "stride" U, param "pad" U, param "H" U, param "W" U]
    body :=
      [ .declInit F "acc" (.index (v "b") (v "oc"))
      , .forCount "ic" (u 0) (v "cin")
          [ .forCount "ky" (u 0) (v "kh")
              [ .declInit I "iy" (pos "oy" "ky")
              , .ifNoElse (inside (v "iy") (v "H"))
                  [ .forCount "kx" (u 0) (v "kw")
                      [ .declInit I "ix" (pos "ox" "kx")
                      , .ifNoElse (inside (v "ix") (v "W"))
                          [ .assign (v "acc") (add (v "acc")
                              (mul (idx4 "x" (v "n") (v "ic") (toU (v "iy")) (toU (v "ix")))
                                   (idx4 "w" (v "oc") (v "ic") (v "ky") (v "kx")))) ] ] ] ] ]
      , .retExpr (v "acc") ] }

/-- The output position that reads input position `i` through kernel offset `k`: `(i + pad - k) / stride`, valid when the
    division is exact and the result is inside `0 .. bound`. -/
private def outPos (i k : String) : SlangExpr := sub (toI (add (v i) (v "pad"))) (toI (v k))

def convBwdInputAt : SlangFunctionDecl :=
  { retType := F, name := "conv_bwd_input_at"
    params := [param "dy" TV, param "w" TV, param "n" U, param "ic" U, param "iy" U, param "ix" U,
               param "cout" U, param "kh" U, param "kw" U, param "stride" U, param "pad" U, param "OH" U, param "OW" U]
    body :=
      [ .declInit F "acc" (.litFloat 0.0)
      , .forCount "oc" (u 0) (v "cout")
          [ .forCount "ky" (u 0) (v "kh")
              [ .declInit I "ny" (outPos "iy" "ky")
              , .ifNoElse (bin "&&" (bin ">=" (v "ny") i0) (bin "==" (bin "%" (v "ny") (toI (v "stride"))) i0))
                  [ .declInit U "oy" (toU (bin "/" (v "ny") (toI (v "stride"))))
                  , .ifNoElse (bin "<" (v "oy") (v "OH"))
                      [ .forCount "kx" (u 0) (v "kw")
                          [ .declInit I "nx" (outPos "ix" "kx")
                          , .ifNoElse (bin "&&" (bin ">=" (v "nx") i0) (bin "==" (bin "%" (v "nx") (toI (v "stride"))) i0))
                              [ .declInit U "ox" (toU (bin "/" (v "nx") (toI (v "stride"))))
                              , .ifNoElse (bin "<" (v "ox") (v "OW"))
                                  [ .assign (v "acc") (add (v "acc")
                                      (mul (idx4 "dy" (v "n") (v "oc") (v "oy") (v "ox"))
                                           (idx4 "w" (v "oc") (v "ic") (v "ky") (v "kx")))) ] ] ] ] ] ] ]
      , .retExpr (v "acc") ] }

def convBwdWeightAt : SlangFunctionDecl :=
  { retType := F, name := "conv_bwd_weight_at"
    params := [param "dy" TV, param "x" TV, param "oc" U, param "ic" U, param "ky" U, param "kx" U,
               param "N" U, param "OH" U, param "OW" U, param "stride" U, param "pad" U, param "H" U, param "W" U]
    body :=
      [ .declInit F "acc" (.litFloat 0.0)
      , .forCount "n" (u 0) (v "N")
          [ .forCount "oy" (u 0) (v "OH")
              [ .declInit I "iy" (pos "oy" "ky")
              , .ifNoElse (inside (v "iy") (v "H"))
                  [ .forCount "ox" (u 0) (v "OW")
                      [ .declInit I "ix" (pos "ox" "kx")
                      , .ifNoElse (inside (v "ix") (v "W"))
                          [ .assign (v "acc") (add (v "acc")
                              (mul (idx4 "dy" (v "n") (v "oc") (v "oy") (v "ox"))
                                   (idx4 "x" (v "n") (v "ic") (toU (v "iy")) (toU (v "ix")))))  ] ] ] ] ]
      , .retExpr (v "acc") ] }

def convModule : SlangShaderModule := { functions := [convFwdAt, convBwdInputAt, convBwdWeightAt] }

/-- The text, for the gates and for `emit_conv`. -/
def text : String := LeanSlang.emit convModule

def count (hay needle : String) : Nat := (hay.splitOn needle).length - 1

-- Gates: each function is emitted once, none is an entry point, and the module has no entry point.
example : count text "conv_fwd_at(" = 1 ∧ count text "conv_bwd_input_at(" = 1 ∧ count text "conv_bwd_weight_at(" = 1 := by native_decide
example : convModule.entryPointName = "" := by native_decide
example : count text "[shader(" = 0 := by native_decide
end ConvKernels
