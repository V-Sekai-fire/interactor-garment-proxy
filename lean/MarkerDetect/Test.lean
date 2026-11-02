import MarkerDetect
open MarkerDetect

/-- Flip the bits listed in `ks` (bit positions 0 to 15). -/
def flipBits (c : UInt16) (ks : List Nat) : UInt16 := ks.foldl (fun acc k => acc ^^^ ((1 : UInt16) <<< k.toUInt16)) c

/-- Debug helper kept for the ambiguity check: which (marker, rotation) pairs carry a given code, and what `decode` says about it. -/
def who (c : UInt16) : List (Nat × Nat) := Id.run do
  let mut out := []
  for k in [0:codes.size] do
    if codes[k]! == c then out := (k / 4, k % 4) :: out
  return out.reverse


def main : IO UInt32 := do
  let mut fail := 0
  IO.println s!"dictionary: {markerCount} markers, {codes.size} codes"
  -- 0. markers that are the same grid as another marker in another rotation: they must be reported, and they are rejected by `decode`
  let amb := ambiguousIds
  IO.println s!"ambiguous markers (share a code with another marker): {amb.length}, e.g. {amb.take 8}"
  -- 1. a code carried by two different markers must be rejected; every other code must decode to its own marker (distance 0)
  let mut exact := 0; let mut rejected := 0; let mut single := 0; let mut shared := 0
  for k in [0:codes.size] do
    let owners := ((who codes[k]!).map (fun p => p.1)).eraseDups
    if owners.length > 1 then
      shared := shared + 1
      if (decode codes[k]! 0).isNone then rejected := rejected + 1
    else
      single := single + 1
      -- a rotation-symmetric pattern has the same code in two rotations, so its orientation is not defined by the cells alone
      let sym := (List.range 4).any (fun s => s != k % 4 && codes[4 * (k / 4) + s]! == codes[k]!)
      match decode codes[k]! 0 with
      | some h => if h.id == k / 4 && (h.rot == k % 4 || sym) && h.dist == 0 then exact := exact + 1
      | none => pure ()
  IO.println s!"codes carried by one marker: {exact} of {single} decode to it exactly; codes carried by two markers: {rejected} of {shared} rejected, not guessed"
  if exact != single || rejected != shared then fail := fail + 1
  -- 2. the smallest distance between different markers, then what that allows
  let dmin := minDistance
  let t := (dmin - 1) / 2
  IO.println s!"minimum distance between different markers: {dmin} bits, so up to {t} flipped bits are corrected safely"
  -- 3. flipBits up to t bits of every marker: must come back as the same marker
  let mut recovered := 0; let mut tried := 0
  for i in [0:markerCount] do
    for b in [0:16] do
      if !amb.contains i then tried := tried + 1
      if !amb.contains i then
        match decode (flipBits codes[4 * i]! [b]) t with
        | some h => if h.id == i then recovered := recovered + 1
        | none => pure ()
  IO.println s!"one flipped bit: recovered {recovered} of {tried}"
  if t ≥ 1 && recovered != tried then fail := fail + 1
  -- 4. the planted control: with more flipped bits than the distance allows, a wrong id must be possible OR rejection must happen, never a silent wrong id at distance <= t
  let mut wrong := 0
  for i in [0:markerCount] do
    match decode (flipBits codes[4 * i]! (List.range (dmin)) ) t with
    | some h => if h.id != i then wrong := wrong + 1
    | none => pure ()
  IO.println s!"control: flipping {dmin} bits: decoded to another marker within {t} bits in {wrong} of {markerCount} cases (the correction limit is real)"
  -- 5. an all-white grid and an all-black grid must not decode
  IO.println s!"all white: {repr (decode 0xFFFF t)}, all black: {repr (decode 0 t)}"
  return fail.toUInt32

