# run_level.nu -- one cloth-fit run on a shell-variant garment; prints one TSV row.
#
#   nu run_level.nu <level> <gap> <defect: yes|no>
#
# Row: level gap defect outcome first_pair_id seconds
# outcome: succeeded | start_intersects | nan | timeout | other
# Needs: SV (shell_variants binary), PF (PolyFEM_bin), FOX (garment-data dir), WORK (scratch run dir).

def main [level: string, gap: string, defect: string] {
    let sv = $env.SV
    let pf = $env.PF
    let fox = $env.FOX
    let id = $"($level)_g($gap)_($defect)"
    let run = ($env.WORK | path join $id)
    rm -rf $run
    mkdir ($run | path join "foxgirl_skirt")
    ln -s ($fox | path join "assets") ($run | path join "assets")
    let setup = (open ($fox | path join "foxgirl_skirt" "setup.json"))
    let dhat = $setup.contact.dhat
    let src = ($fox | path join "assets" "garments" "LCL_Skirt_DressEvening_003" "garment.obj")
    let dir = ($run | path join "foxgirl_skirt")
    let args = [$src $level $gap ($dhat | into string) ($dir | path join "garment.obj")]
    if $defect == "yes" { ^$sv ...$args defect } else { ^$sv ...$args }
    $setup | upsert garment_mesh_path "garment.obj" | upsert solver.max_threads 4 | save ($dir | path join "setup.json")
    let t0 = (date now)
    let r = (do { cd $dir; ^timeout ($env.LIMIT? | default "600") $pf -j setup.json --max_threads 4 } | complete)
    let secs = ((date now) - $t0 | into int) / 1_000_000_000
    let log = ($r.stdout + $r.stderr)
    let outcome = if ($log | str contains "Garment retargeting succeeded") { "succeeded" } else if ($log | str contains "initial solution has intersections") { "start_intersects" } else if ($log | str contains "nan or inf") { "nan" } else if $r.exit_code == 124 { "timeout" } else { "other" }
    let pair = ($dir | path join "intersecting_pair.obj")
    let first = if ($pair | path exists) { (open $pair --raw | lines | where {|l| $l | str starts-with "f "} | first | split row " " | get 1) } else { "" }
    $log | save -f ($run | path join "run.log")
    print $"($level)\t($gap)\t($defect)\t($outcome)\t($first)\t($secs | math round)"
}
