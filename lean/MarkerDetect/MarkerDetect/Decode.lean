import MarkerDetect.Dictionary

/-! Decoder core of the marker detector: a 4x4 grid of white/black cells (16 bits) to a marker id, with error correction.
    The dictionary is OpenCV's `DICT_4X4_1000`, exported by `tools/gen_dictionary.py`. -/
namespace MarkerDetect

/-- Number of differing bits between two 16-bit grids. -/
def hamming (a b : UInt16) : Nat :=
  let x := (a ^^^ b).toNat
  (List.range 16).foldl (fun acc k => acc + ((x >>> k) &&& 1)) 0

/-- A decode result: the marker id, the rotation of the grid (0 to 3), and the bit distance to the dictionary code. -/
structure Hit where
  id : Nat
  rot : Nat
  dist : Nat
  deriving Repr, BEq

/-- Nearest dictionary code over all 1000 markers and 4 rotations. `none` when the best distance exceeds `maxDist`,
    or when two different markers tie for the best distance (an ambiguous grid is rejected, never guessed). -/
def decode (bits : UInt16) (maxDist : Nat) : Option Hit := Id.run do
  let mut best : Option Hit := none
  let mut tie := false
  for k in [0:codes.size] do
    let dst := hamming bits codes[k]!
    let id := k / 4
    match best with
    | none => best := some { id, rot := k % 4, dist := dst }
    | some b =>
      if dst < b.dist then
        best := some { id, rot := k % 4, dist := dst }
        tie := false
      else if dst == b.dist && id != b.id then
        tie := true
  match best with
  | some b => if b.dist ≤ maxDist && !tie then return some b else return none
  | none => return none

/-- Markers that share a code with a different marker in some rotation (the same grid read as two ids): a single such marker cannot be
    named from its own cells, only from its neighbours. -/
def ambiguousIds : List Nat := Id.run do
  let mut out : List Nat := []
  for i in [0:markerCount] do
    let mut amb := false
    for j in [0:markerCount] do
      if j != i then
        for r in [0:4] do
          for t in [0:4] do
            if codes[4 * i + r]! == codes[4 * j + t]! then amb := true
    if amb then out := i :: out
  return out.reverse

/-- Smallest distance between codes of two different markers, ignoring the ambiguous ones, over every pair of rotations. A grid within
    `(dMin - 1) / 2` bits of a code cannot be nearer to another unambiguous marker. -/
def minDistance : Nat := Id.run do
  let amb := ambiguousIds
  let mut m := 16
  for i in [0:markerCount] do
    if !amb.contains i then
      for j in [i + 1:markerCount] do
        if !amb.contains j then
          for r in [0:4] do
            m := min m (hamming codes[4 * i]! codes[4 * j + r]!)
  return m

end MarkerDetect
