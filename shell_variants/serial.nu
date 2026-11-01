# serial.nu -- the cells that timed out or disagreed, one at a time, no CPU contention.
let cells = [[l g]; [L2 "1"] [L2 "2"] [L2 "2"] [L2 "4"] [L2 "0.5"] [L3 "1"] [L3 "2"] [L3 "4"] [L3 "0.5"]]
"level\tgap\tdefect\toutcome\tfirst_pair_id\tseconds\n" | save -f results_A2.tsv
for c in $cells {
    let row = (nu run_level.nu $c.l $c.g no | str trim)
    $"($row)\n" | save -a results_A2.tsv
}
