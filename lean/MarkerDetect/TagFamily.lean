/-!
# Tag family generator

Searches for a marker family: codes of `side x side` black/white data cells (bit k = row k / side, column k % side, 1 = white), with
a guaranteed minimum Hamming distance between any two tags in any of the four rotations, and between a tag and its own rotations
(so the orientation is never ambiguous), and a minimum pattern complexity (adjacent cell pairs that differ), so no tag is nearly
all one colour. The search is a seeded random lexicode (a code is kept when it is far enough from every code already kept), the
same family of method as the ArUco and AprilTag generators; the same seed always gives the same family.

Work is pass-fail, quality is a measure: the program checks its own gates first and prints PASS or FAIL for each; the measures
(false-accept rate, tries) are printed after, and the exit code is 0 only when every gate passes.
-/

def m1 : UInt64 := 0x5555555555555555
def m2 : UInt64 := 0x3333333333333333
def m4 : UInt64 := 0x0f0f0f0f0f0f0f0f
def h01 : UInt64 := 0x0101010101010101

def popcnt (x : UInt64) : Nat :=
  let x := x - ((x >>> 1) &&& m1)
  let x := (x &&& m2) + ((x >>> 2) &&& m2)
  let x := (x + (x >>> 4)) &&& m4
  ((x * h01) >>> 56).toNat

/-- Destination bit of every source bit under a 90 degree turn of a `side x side` grid. -/
def turnPerm (side : Nat) : Array Nat :=
  (Array.range (side * side)).map fun k => (k % side) * side + (side - 1 - k / side)

def turn (perm : Array Nat) (x : UInt64) : UInt64 := Id.run do
  let mut out : UInt64 := 0
  for k in [0:perm.size] do
    if ((x >>> k.toUInt64) &&& 1) == 1 then out := out ||| ((1 : UInt64) <<< perm[k]!.toUInt64)
  return out

/-- The four rotations of a tag. -/
def rots (perm : Array Nat) (x : UInt64) : Array UInt64 :=
  let r1 := turn perm x
  let r2 := turn perm r1
  #[x, r1, r2, turn perm r2]

/-- Number of adjacent cell pairs (left-right and up-down) with different colours. -/
def complexity (side : Nat) (x : UInt64) : Nat := Id.run do
  let bit := fun (r c : Nat) => ((x >>> (r * side + c).toUInt64) &&& 1) == 1
  let mut n := 0
  for r in [0:side] do
    for c in [0:side] do
      if c + 1 < side && bit r c != bit r (c + 1) then n := n + 1
      if r + 1 < side && bit r c != bit (r + 1) c then n := n + 1
  return n

def nextState (s : UInt64) : UInt64 :=
  let s := s ^^^ (s >>> 12)
  let s := s ^^^ (s <<< 25)
  s ^^^ (s >>> 27)

def mulK : UInt64 := 0x2545F4914F6CDD1D
def goldenK : UInt64 := 0x9E3779B97F4A7C15

def randWord (st : UInt64) : UInt64 := st * mulK

/-- The search: random words, kept when complex enough, distinguishable from their own rotations, and at least `d` bits from every kept tag in every rotation. -/
def generate (side d minC target maxTries : Nat) (seed : UInt64) : Array UInt64 × Nat := Id.run do
  let perm := turnPerm side
  let mask : UInt64 := ((1 : UInt64) <<< (side * side).toUInt64) - 1
  let mut st := seed
  let mut codes : Array UInt64 := #[]
  let mut tries := 0
  while codes.size < target && tries < maxTries do
    tries := tries + 1
    st := nextState st
    let c := randWord st &&& mask
    if complexity side c < minC then continue
    let r := rots perm c
    if popcnt (c ^^^ r[1]!) < d || popcnt (c ^^^ r[2]!) < d || popcnt (c ^^^ r[3]!) < d then continue
    let mut ok := true
    for a in codes do
      if popcnt (a ^^^ r[0]!) < d || popcnt (a ^^^ r[1]!) < d || popcnt (a ^^^ r[2]!) < d || popcnt (a ^^^ r[3]!) < d then
        ok := false
        break
    if ok then codes := codes.push c
  return (codes, tries)

/-- Nearest tag and rotation: `(tag, rotation, distance, tied)`. -/
def nearest (all : Array (Array UInt64)) (q : UInt64) : Nat × Nat × Nat × Bool := Id.run do
  let mut best := (0, 0, 64, false)
  for i in [0:all.size] do
    for r in [0:4] do
      let dst := popcnt (q ^^^ all[i]![r]!)
      if dst < best.2.2.1 then best := (i, r, dst, false)
      else if dst == best.2.2.1 && i != best.1 then best := (best.1, best.2.1, dst, true)
  return best

def hex (x : UInt64) : String := "0x" ++ String.ofList (Nat.toDigits 16 x.toNat)

def main (args : List String) : IO UInt32 := do
  let arg := fun (i : Nat) (dflt : Nat) => (args[i]?.bind String.toNat?).getD dflt
  let side := arg 0 6; let d := arg 1 9; let minC := arg 2 22; let target := arg 3 2600; let seed := arg 4 1; let maxTries := arg 5 400000000
  let out := args[6]?.getD "family.json"
  let t0 ← IO.monoMsNow
  let (codes, tries) := generate side d minC target maxTries (seed.toUInt64 * goldenK + 1)
  let t1 ← IO.monoMsNow
  IO.println s!"search: {side}x{side} cells, distance >= {d}, complexity >= {minC}, wanted {target}: found {codes.size} tags in {tries} tries, {(t1 - t0) / 1000} s"
  let perm := turnPerm side
  let all := codes.map (rots perm)
  let mut fail := 0
  let gate : String → Bool → String → IO Bool := fun name ok detail => do
    IO.println s!"{if ok then "PASS" else "FAIL"} {name} {detail}"
    return !ok
  -- gate 1: enough tags
  if (← gate "enough tags" (codes.size ≥ target) s!"({codes.size} of {target})") then fail := fail + 1
  -- gate 2: pairwise distance over every pair and every rotation, and each tag against its own rotations
  let mut pairMin := 64
  for i in [0:all.size] do
    for j in [i + 1:all.size] do
      for r in [0:4] do
        pairMin := min pairMin (popcnt (all[i]![0]! ^^^ all[j]![r]!))
  let mut selfMin := 64
  let mut cxMin := 999
  for i in [0:all.size] do
    for r in [1:4] do selfMin := min selfMin (popcnt (all[i]![0]! ^^^ all[i]![r]!))
    cxMin := min cxMin (complexity side codes[i]!)
  if (← gate "minimum distance between different tags, all rotations" (pairMin ≥ d) s!"({pairMin} bits, needed {d})") then fail := fail + 1
  if (← gate "no rotation twins: a tag differs from its own rotations" (selfMin ≥ d) s!"({selfMin} bits, needed {d})") then fail := fail + 1
  if (← gate "complexity" (cxMin ≥ minC) s!"({cxMin}, needed {minC})") then fail := fail + 1
  -- gate 3: round trip with up to t flipped bits, every rotation, a sample of tags
  let t := (d - 1) / 2
  let mut rtTried := 0; let mut rtOk := 0; let mut st : UInt64 := 12345
  let step := max 1 (codes.size / 300)
  let sample : List Nat := ((List.range (codes.size / step + 1)).map (· * step)).filter (· < codes.size)
  for i in sample do
    for r in [0:4] do
      let mut q := all[i]![r]!
      for _ in [0:t] do
        st := nextState st
        q := q ^^^ ((1 : UInt64) <<< (st % (side * side).toUInt64))
      rtTried := rtTried + 1
      let (bi, br, bd, tied) := nearest all q
      if bi == i && br == r && bd ≤ t && !tied then rtOk := rtOk + 1
  -- flipping the same bit twice can only help, so a failure here is a real failure
  if (← gate s!"round trip with {t} flipped bits" (rtOk == rtTried) s!"({rtOk} of {rtTried})") then fail := fail + 1
  -- gate 4: planted control: a tag with d bits flipped must NOT decode to that tag within t bits
  let mut planted := 0; let mut plantedOk := 0
  for i in sample do
    let mut q := codes[i]!
    for b in [0:d] do q := q ^^^ ((1 : UInt64) <<< ((b * 5 + i) % (side * side)).toUInt64)
    planted := planted + 1
    let (bi, _, bd, tied) := nearest all q
    if !(bi == i && bd ≤ t && !tied) then plantedOk := plantedOk + 1
  if (← gate s!"control: {d} flipped bits never decode back to the same tag" (plantedOk == planted) s!"({plantedOk} of {planted})") then fail := fail + 1
  IO.println (if fail == 0 then "ALL GATES PASS" else s!"{fail} GATE(S) FAILED")
  -- measures (only meaningful when the gates pass)
  let mut wrong := 0
  for _ in [0:20000] do
    st := nextState st
    let q := randWord st &&& (((1 : UInt64) <<< (side * side).toUInt64) - 1)
    let (_, _, bd, tied) := nearest all q
    if bd ≤ t && !tied then wrong := wrong + 1
  IO.println s!"measure: random words wrongly accepted within {t} bits: {wrong} of 20000; data bits {side * side}; correctable bits {t}"
  let body := String.intercalate "," (codes.toList.map fun c => "\"" ++ hex c ++ "\"")
  IO.FS.writeFile out s!"\{\"side\":{side},\"minDistance\":{d},\"minComplexity\":{minC},\"count\":{codes.size},\"seed\":{seed},\"codes\":[{body}]}\n"
  IO.println s!"wrote {out}"
  return (if fail == 0 then 0 else 1)
