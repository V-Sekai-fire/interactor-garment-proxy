# sweep.nu -- the shell-variant levels through PolyFEM_bin, three at a time; writes results_A.tsv.
# Env as run_level.nu. Run from this directory.
let gaps = ["0.5" "1" "2" "4" "8"]
let clean = ([L1 L2 L3] | each {|l| $gaps | each {|g| {level: $l, gap: $g, defect: "no"}}} | flatten)
let planted = ([L1 L2 L3] | each {|l| {level: $l, gap: "4", defect: "yes"}})
let repeat = [{level: "L2", gap: "2", defect: "no"}]
let cells = ($clean ++ $planted ++ $repeat)
let rows = ($cells | par-each -t 3 {|c| (nu run_level.nu $c.level $c.gap $c.defect | str trim) })
["level\tgap\tdefect\toutcome\tfirst_pair_id\tseconds"] ++ $rows | str join "\n" | save -f results_A.tsv
open results_A.tsv --raw | print
