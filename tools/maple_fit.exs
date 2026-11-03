#!/usr/bin/env elixir
# maple_fit.exs: fit an outfit piece onto the Miroir body with cloth-fit, then fit the outfit's own bones so the existing skinning
# reproduces the result. The driver only orchestrates: every stage is a small program (GDScript in the double engine, PolyFEM,
# remesh.elf, bone_fit.gd) that it starts in the background with a log and polls for its exit status.
#
#   elixir tools/maple_fit.exs --piece BKimono [--work DIR] [--config FILE] [--from STAGE] [--only STAGE] [--force] [--list]
#
# Stages: inputs, csg-clean, proxies, reduce, cloth-fit, map-back, bone-fit, report. See tools/README-maple-fit.md.

defmodule MapleFit do
  @stages ~w(inputs csg-clean proxies reduce cloth-fit map-back bone-fit report)

  defmodule Gate do
    defexception [:stage, :reason, :log]
    def message(e), do: "gate failed in #{e.stage}: #{e.reason} (log: #{e.log})"
  end

  # ------------------------------------------------------------------ entry
  def main(argv) do
    {opts, _, _} =
      OptionParser.parse(argv,
        strict: [piece: :string, work: :string, config: :string, from: :string, only: :string, force: :boolean, list: :boolean, help: :boolean, adopt: :string]
      )

    if opts[:help] || opts[:piece] == nil and not (opts[:list] || false) do
      IO.puts("usage: elixir tools/maple_fit.exs --piece <name> [--work DIR] [--config FILE] [--from STAGE] [--only STAGE] [--force] [--list]\nstages: #{Enum.join(@stages, " ")}")
      System.halt(if opts[:help], do: 0, else: 2)
    end

    root = __ENV__.file |> Path.dirname() |> Path.dirname() |> Path.expand()
    cfg_path = opts[:config] || Path.join(root, "tools/config/maple.exs")
    {cfg, _} = Code.eval_file(cfg_path)

    if opts[:list] do
      IO.puts("pieces: " <> Enum.join(Map.keys(cfg.pieces), " "))
      IO.puts("stages: " <> Enum.join(@stages, " "))
      System.halt(0)
    end

    piece = opts[:piece]
    pcfg = Map.get(cfg.pieces, piece) || (IO.puts("unknown piece #{piece}; known: #{Enum.join(Map.keys(cfg.pieces), " ")}"); System.halt(2))
    pcfg = if opts[:adopt], do: Map.put(pcfg, :adopt_cloth_fit, Path.expand(opts[:adopt])), else: pcfg
    work = Path.expand(opts[:work] || Path.join(root, "work"))
    ctx = %{root: root, cfg: cfg, p: pcfg, piece: piece, work: Path.join(work, piece), force: opts[:force] || false, cfg_path: cfg_path}

    selected =
      cond do
        opts[:only] -> check_stage!(opts[:only]); [opts[:only]]
        opts[:from] -> check_stage!(opts[:from]); Enum.drop_while(@stages, &(&1 != opts[:from]))
        true -> @stages
      end

    File.mkdir_p!(ctx.work)
    setup_project(ctx)
    t0 = System.monotonic_time(:second)

    {rows, failure} =
      Enum.reduce_while(selected, {[], nil}, fn stage, {rows, _} ->
        try do
          row = run_stage(stage, ctx)
          {:cont, {rows ++ [row], nil}}
        rescue
          e in Gate ->
            IO.puts("STOP #{e.stage}: #{e.reason}\n  log: #{e.log}")
            {:halt, {rows ++ [{stage, "FAILED", 0, "-", e.reason}], e}}
        end
      end)

    IO.puts("\n== #{piece}: per-stage table ==")
    IO.puts(String.pad_trailing("stage", 12) <> String.pad_trailing("status", 9) <> String.pad_trailing("seconds", 9) <> String.pad_trailing("source", 9) <> "note")
    Enum.each(rows, fn {s, st, sec, src, note} -> IO.puts(String.pad_trailing(s, 12) <> String.pad_trailing(st, 9) <> String.pad_trailing("#{sec}", 9) <> String.pad_trailing(src, 9) <> note) end)
    IO.puts("total #{System.monotonic_time(:second) - t0} s; work dir #{ctx.work}")
    if failure, do: System.halt(1), else: System.halt(0)
  end

  defp check_stage!(s), do: if(s in @stages, do: :ok, else: (IO.puts("unknown stage #{s}; stages: #{Enum.join(@stages, " ")}"); System.halt(2)))

  # ------------------------------------------------------------------ project, paths, hashing
  defp rel(ctx, p), do: if(String.starts_with?(p, "./"), do: Path.join(ctx.root, p), else: p)
  defp proj(ctx), do: Path.join(ctx.work, "godot_project")
  defp stage_dir(ctx, stage), do: Path.join(ctx.work, stage)
  defp engine(ctx), do: rel(ctx, ctx.cfg.engine)
  defp elf(ctx, k), do: rel(ctx, Map.fetch!(ctx.cfg, k))

  defp setup_project(ctx) do
    File.mkdir_p!(proj(ctx))
    File.write!(Path.join(proj(ctx), "project.godot"), "config_version=5\n[application]\nconfig/name=\"maple_fit\"\n")

    Enum.each(ctx.cfg.scripts, fn s ->
      src = Path.join([ctx.root, "godot", s <> ".gd"])
      unless File.exists?(src), do: raise(Gate, stage: "inputs", reason: "stage script missing: #{src}", log: "-")
      File.cp!(src, Path.join(proj(ctx), s <> ".gd"))
    end)
  end

  defp hash_file(p) do
    if File.exists?(p) do
      ctx = File.stream!(p, 4_194_304) |> Enum.reduce(:crypto.hash_init(:sha256), fn chunk, acc -> :crypto.hash_update(acc, chunk) end)
      ctx |> :crypto.hash_final() |> Base.encode16(case: :lower) |> binary_part(0, 16)
    else
      "missing"
    end
  end

  defp script_hash(ctx, name), do: hash_file(Path.join([ctx.root, "godot", name <> ".gd"]))

  # ------------------------------------------------------------------ process running (background, log, short polls)
  # Starts `cmd args` detached from this process with stdout+stderr in `log`; writes `<log>.pid` and `<log>.status` (exit code).
  # Returns the exit code once the status file exists. Resumable: if a pid file exists with no status and the pid is alive, it
  # keeps polling instead of starting a second copy.
  defp run_bg(cmd, args, log, opts \\ []) do
    env = Keyword.get(opts, :env, [])
    cwd = Keyword.get(opts, :cwd, File.cwd!())
    timeout = Keyword.get(opts, :timeout_s, 1800)
    poll_ms = Keyword.get(opts, :poll_ms, 1000)
    resume = Keyword.get(opts, :resume, false)
    status = log <> ".status"
    pidf = log <> ".pid"
    File.mkdir_p!(Path.dirname(log))

    started =
      if resume and File.exists?(pidf) and not File.exists?(status) and pid_alive?(File.read!(pidf)) do
        IO.puts("  resuming wait on running job (pid #{String.trim(File.read!(pidf))})")
        false
      else
        File.rm(status)
        File.rm(pidf)

        script =
          ~s|cd "$1"; shift; ( sh -c 'echo $$ > "$0"; exec "$@"' "#{pidf}" "$@" > "#{log}" 2>&1 < /dev/null; echo $? > "#{status}.tmp"; mv "#{status}.tmp" "#{status}" ) < /dev/null > /dev/null 2>&1 &|

        {_, 0} = System.cmd("sh", ["-c", script, "sh", cwd, cmd | args], env: env)
        true
      end

    _ = started
    deadline = System.monotonic_time(:second) + timeout
    wait_status(status, log, deadline, poll_ms, System.monotonic_time(:second), Keyword.get(opts, :watch, false), pidf)
  end

  defp wait_status(status, log, deadline, poll_ms, last_note, watch, pidf) do
    cond do
      File.exists?(status) ->
        status |> File.read!() |> String.trim() |> String.to_integer()

      watch and File.exists?(log) and String.contains?(File.read!(log), "SCRIPT ERROR") ->
        # a GDScript error leaves the headless engine running forever: kill it and stop
        if File.exists?(pidf), do: System.cmd("kill", [String.trim(File.read!(pidf))], stderr_to_stdout: true)
        first = File.read!(log) |> String.split("\n") |> Enum.find(&String.contains?(&1, "SCRIPT ERROR"))
        raise Gate, stage: "script", reason: "GDScript error: " <> String.trim(first), log: log

      System.monotonic_time(:second) > deadline ->
        raise Gate, stage: "timeout", reason: "no exit status before the deadline", log: log

      true ->
        Process.sleep(poll_ms)
        now = System.monotonic_time(:second)
        if now - last_note >= 60, do: IO.puts("  ... running (#{Path.basename(log)})")
        wait_status(status, log, deadline, poll_ms, if(now - last_note >= 60, do: now, else: last_note), watch, pidf)
    end
  end

  defp pid_alive?(s), do: match?({_, 0}, System.cmd("kill", ["-0", String.trim(s)], stderr_to_stdout: true))

  # Runs a GDScript stage program headless in the double engine; returns {exit_code, log_text}.
  defp godot(ctx, script, args, log, opts \\ []) do
    full = ["--headless", "--path", proj(ctx), "-s", "res://#{script}.gd", "--" | args]
    code = run_bg(engine(ctx), full, log, Keyword.put(opts, :watch, true))
    {code, File.read!(log)}
  end

  defp need!(stage, log, text, regex, what) do
    case Regex.run(regex, text) do
      nil -> raise Gate, stage: stage, reason: "expected output missing: #{what}", log: log
      m -> m
    end
  end

  # ------------------------------------------------------------------ manifests / caching
  defp run_stage(stage, ctx) do
    dir = stage_dir(ctx, stage)
    File.mkdir_p!(dir)
    {inputs, params} = stage_key(stage, ctx)
    key = %{inputs: Map.new(inputs, fn p -> {p, hash_file(p)} end), params: params}
    mpath = Path.join(dir, "manifest.term")
    prev = if File.exists?(mpath), do: :erlang.binary_to_term(File.read!(mpath)), else: nil

    if not ctx.force and prev != nil and prev.status == :pass and prev.key == key and Enum.all?(prev.outputs, &File.exists?/1) do
      IO.puts("[#{stage}] cache hit (#{prev.seconds} s when it ran)")
      Enum.each(prev.numbers, &IO.puts("    " <> &1))
      {stage, "pass", 0, "cached", "ran #{prev.seconds} s earlier"}
    else
      IO.puts("[#{stage}] running")
      t0 = System.monotonic_time(:second)
      {numbers, outputs} = do_stage(stage, ctx, dir)
      sec = System.monotonic_time(:second) - t0
      Enum.each(numbers, &IO.puts("    " <> &1))
      manifest = %{status: :pass, key: key, outputs: outputs, numbers: numbers, seconds: sec, tools: tool_versions(ctx), at: DateTime.utc_now() |> DateTime.to_string()}
      File.write!(mpath, :erlang.term_to_binary(manifest))
      File.write!(Path.join(dir, "manifest.txt"), format_manifest(stage, manifest))
      {stage, "pass", sec, "run", "gates passed"}
    end
  end

  defp tool_versions(ctx) do
    %{engine: hash_file(engine(ctx)), usd_elf: hash_file(elf(ctx, :usd_elf)), remesh_elf: hash_file(elf(ctx, :remesh_elf)), polyfem: hash_file(rel(ctx, ctx.cfg.polyfem)), bone_fit: hash_file(rel(ctx, ctx.cfg.bone_fit))}
  end

  defp format_manifest(stage, m) do
    ins = Enum.map_join(m.key.inputs, "\n", fn {p, h} -> "  #{h}  #{p}" end)
    "stage #{stage}\nstatus #{m.status}\nseconds #{m.seconds}\nat #{m.at}\ntools #{inspect(m.tools)}\nparams #{inspect(m.key.params, limit: :infinity, pretty: true)}\ninputs\n#{ins}\noutputs\n" <> Enum.map_join(m.outputs, "\n", &("  " <> &1)) <> "\nnumbers\n" <> Enum.map_join(m.numbers, "\n", &("  " <> &1)) <> "\n"
  end

  # The inputs (files whose hash decides a cache hit) and the parameters of each stage.
  defp stage_key(stage, ctx) do
    p = ctx.p
    ex = ctx.cfg.export_dir
    piece = ctx.piece
    own = Path.join(ex, "garment_own_#{piece}.bin")
    scripts = fn names -> Enum.map(names, &Path.join([ctx.root, "godot", &1 <> ".gd"])) end
    tools = [engine(ctx)]

    case stage do
      "inputs" ->
        {[Path.join(ex, "garment_#{piece}.obj"), own, Path.join(ex, "avatar.obj"), Path.join(ex, "source_skeleton.obj"), Path.join(ex, "target_skeleton.obj"), ctx.cfg.fbx, ctx.cfg.prefab], %{piece: piece}}

      "csg-clean" ->
        {[Path.join(ex, "garment_#{piece}.obj"), elf(ctx, :usd_elf)] ++ scripts.(["csg_clean_usd"]) ++ tools, %{csg_clean: ctx.cfg.csg_clean}}

      "proxies" ->
        {[Path.join(stage_dir(ctx, "csg-clean"), "out/#{piece}.usdz"), Path.join(ex, "avatar.obj"), elf(ctx, :usd_elf)] ++ scripts.(["union_proxy"]) ++ tools,
         %{proxy: ctx.cfg.proxy, garment_thickness_m: p.garment_thickness_m, avatar_thickness_m: p.avatar_thickness_m}}

      "reduce" ->
        {[Path.join(stage_dir(ctx, "proxies"), "garment_union.obj"), Path.join(stage_dir(ctx, "proxies"), "avatar_union.obj"), Path.join(ex, "source_skeleton.obj"), Path.join(ex, "target_skeleton.obj"), elf(ctx, :remesh_elf)] ++
           scripts.(["run_remesh", "to_solver", "fix_smooth", "jitter", "shell_pick", "nofit", "mesh_check", "crosslib"]) ++ tools,
         %{reduce: ctx.cfg.reduce, scale: ctx.cfg.scale, p: Map.take(p, [:garment_res, :avatar_res, :target_triangles, :smooth_max_iter, :jitter_seeds, :drop_shell_ranks, :nofit])}}

      "cloth-fit" ->
        case Map.get(p, :external_cloth_fit) do
          nil ->
            a = Path.join(stage_dir(ctx, "reduce"), "assets")
            {Enum.map(~w(avatar_red.obj garment_red.obj no-fit_red.txt source_skeleton.obj target_skeleton.obj), &Path.join(a, &1)) ++ [rel(ctx, ctx.cfg.polyfem)], %{setup: ctx.cfg.polyfem_setup, adopt: Map.get(p, :adopt_cloth_fit)}}

          ext ->
            {Enum.map(~w(avatar_red.obj garment_red.obj), &Path.join(ext.assets, &1)) ++ [Path.join(ext.run, "setup.json")], %{external: ext.run}}
        end

      "map-back" ->
        {a, run, steps} = cloth_fit_paths(ctx)
        {[Path.join(a, "garment_red.obj"), Path.join(a, "avatar_red.obj"), Path.join(run, "target_avatar.obj"), steps.garment, own, elf(ctx, :remesh_elf), elf(ctx, :usd_elf)] ++ scripts.(["map_back", "obj_to_usdz"]) ++ tools,
         %{scale: ctx.cfg.scale, map_back: ctx.cfg.map_back, last_step: steps.n}}

      "bone-fit" ->
        mb = stage_dir(ctx, "map-back")
        {[Path.join(mb, "target.bin"), own, ctx.cfg.fbx, ctx.cfg.prefab, rel(ctx, ctx.cfg.bone_fit), elf(ctx, :usd_elf)] ++ tools,
         %{mesh: p.mesh, same_piece: p.same_piece, bone_fit: ctx.cfg.bone_fit_cfg, expect: Map.get(p, :expect)}}

      "report" ->
        {Enum.map(~w(inputs csg-clean proxies reduce cloth-fit map-back bone-fit), &Path.join(stage_dir(ctx, &1), "manifest.term")), %{}}
    end
  end

  # Where the cloth-fit result lives: this driver's run, an adopted run, or an external one (skirt).
  defp cloth_fit_paths(ctx) do
    {assets, run} =
      case Map.get(ctx.p, :external_cloth_fit) do
        nil ->
          adopt = Map.get(ctx.p, :adopt_cloth_fit)
          {Path.join(stage_dir(ctx, "reduce"), "assets"), if(adopt, do: adopt, else: Path.join(stage_dir(ctx, "cloth-fit"), "fit"))}

        ext ->
          {ext.assets, ext.run}
      end

    n =
      if File.dir?(run) do
        run |> File.ls!() |> Enum.flat_map(fn f -> case Regex.run(~r/^step_garment_(\d+)\.obj$/, f) do [_, d] -> [String.to_integer(d)]; _ -> [] end end) |> Enum.max(fn -> -1 end)
      else
        -1
      end

    {assets, run, %{n: n, garment: Path.join(run, "step_garment_#{n}.obj")}}
  end

  # ------------------------------------------------------------------ the stages
  defp do_stage("inputs", ctx, dir) do
    {files, _} = stage_key("inputs", ctx)
    missing = Enum.filter(files, &(not File.exists?(&1)))
    if missing != [], do: raise(Gate, stage: "inputs", reason: "missing input: #{Enum.join(missing, ", ")}", log: "-")
    tools = [engine(ctx), elf(ctx, :usd_elf), elf(ctx, :remesh_elf), rel(ctx, ctx.cfg.bone_fit)] ++ if(Map.get(ctx.p, :external_cloth_fit), do: [], else: [rel(ctx, ctx.cfg.polyfem)])
    gone = Enum.filter(tools, &(not File.exists?(&1)))
    if gone != [], do: raise(Gate, stage: "inputs", reason: "missing tool: #{Enum.join(gone, ", ")} (see tools/README-maple-fit.md to build or copy them)", log: "-")
    listing = Path.join(dir, "inputs.txt")
    File.write!(listing, Enum.map_join(files ++ tools, "\n", fn f -> "#{hash_file(f)}  #{f}" end) <> "\n")
    {["#{length(files)} input files and #{length(tools)} tools present"], [listing]}
  end

  defp do_stage("csg-clean", ctx, dir) do
    out = Path.join(dir, "out")
    log = Path.join(dir, "csg_clean.log")
    c = ctx.cfg.csg_clean
    {code, txt} = godot(ctx, "csg_clean_usd", [elf(ctx, :usd_elf), out, "#{c.thickness_m}", Path.join(ctx.cfg.export_dir, "garment_#{ctx.piece}.obj")], log, timeout_s: c.timeout_s)
    if code != 0, do: raise(Gate, stage: "csg-clean", reason: "csg_clean_usd.gd exit #{code} (readback or planted control failed)", log: log)
    m = need!("csg-clean", log, txt, ~r/#{ctx.piece}\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\S+)\s+(identical|DIFFERS)\s+(caught|MISSED)/, "result row for #{ctx.piece}")
    [_, shells, enclosed, tin, tout, bytes, union, rb, ctl] = m
    if rb != "identical" or ctl != "caught", do: raise(Gate, stage: "csg-clean", reason: "usdz readback #{rb}, planted control #{ctl}", log: log)
    if union == "refused", do: raise(Gate, stage: "csg-clean", reason: "Manifold refused the union", log: log)
    {["shells #{shells}, enclosed (dropped) #{enclosed}, triangles #{tin} -> #{tout}, #{bytes} bytes, union #{union}, usdz readback #{rb}, planted control #{ctl}"], [Path.join(out, "#{ctx.piece}.usdz")]}
  end

  defp do_stage("proxies", ctx, dir) do
    p = ctx.p
    t = ctx.cfg.proxy.timeout_s

    res =
      for {kind, src, thick} <- [{"garment", Path.join(stage_dir(ctx, "csg-clean"), "out/#{ctx.piece}.usdz"), p.garment_thickness_m}, {"avatar", Path.join(ctx.cfg.export_dir, "avatar.obj"), p.avatar_thickness_m}] do
        log = Path.join(dir, "#{kind}_union.log")
        out = Path.join(dir, "#{kind}_union.obj")
        {code, txt} = godot(ctx, "union_proxy", [elf(ctx, :usd_elf), src, out, "#{thick}"], log, timeout_s: t)
        if code != 0 or not String.contains?(txt, "RESULT: PASS"), do: raise(Gate, stage: "proxies", reason: "#{kind} union failed", log: log)
        [_, welded] = need!("proxies", log, txt, ~r/welded: (.*)/, "welded line")
        [_, comps] = need!("proxies", log, txt, ~r/union components: (\d+)/, "components")
        [_, kept, dropped, tris] = need!("proxies", log, txt, ~r/components kept (\d+), dropped (\d+), kept triangles (\d+)/, "kept line")
        if String.to_integer(tris) == 0, do: raise(Gate, stage: "proxies", reason: "#{kind} union has no triangles", log: log)
        {"#{kind}: thickness #{thick} m; input #{welded}; union components #{comps}, kept #{kept} (enclosed dropped #{dropped}), #{tris} triangles", out}
      end

    {Enum.map(res, &elem(&1, 0)), Enum.map(res, &elem(&1, 1))}
  end

  defp do_stage("reduce", ctx, dir) do
    p = ctx.p
    cfgr = ctx.cfg.reduce
    sc = "#{ctx.cfg.scale}"
    a = Path.join(dir, "assets")
    File.mkdir_p!(a)
    pdir = stage_dir(ctx, "proxies")
    t = cfgr.timeout_s

    one = fn kind, res, seed ->
      src = Path.join(pdir, "#{kind}_union.obj")
      un = Path.join(dir, "#{kind}_unity.obj")
      log = Path.join(dir, "#{kind}_remesh.log")
      {code, txt} = godot(ctx, "run_remesh", [elf(ctx, :remesh_elf), src, un, "#{p.target_triangles}", "1.0", "#{cfgr.options}"], log, env: [{"REMESH_MODE", "remesh"}, {"REMESH_RES", "#{res}"}], timeout_s: t)
      if code != 0 or not String.contains?(txt, "RESULT: PASS"), do: raise(Gate, stage: "reduce", reason: "#{kind} remesh failed", log: log)
      [_, v, tr] = need!("reduce", log, txt, ~r/out: (\d+) vertices, (\d+) triangles/, "remesh out line")
      s0 = Path.join(dir, "#{kind}_s0.obj")
      {c2, _} = godot(ctx, "to_solver", [un, s0, sc], Path.join(dir, "#{kind}_to_solver.log"), timeout_s: t)
      if c2 != 0, do: raise(Gate, stage: "reduce", reason: "#{kind} to_solver failed", log: Path.join(dir, "#{kind}_to_solver.log"))
      {s0, v, tr, seed}
    end

    {gs0, gv, gt, _} = one.("garment", p.garment_res, p.jitter_seeds.garment)
    {as0, av, at, _} = one.("avatar", p.avatar_res, p.jitter_seeds.avatar)

    # garment: smooth the pinch crossings, jitter, drop the shells the solver cannot start with, no-fit list
    sm = Path.join(dir, "garment_s1.obj")
    smlog = Path.join(dir, "garment_smooth.log")
    {c3, _} = godot(ctx, "fix_smooth", [gs0, sm, "#{p.smooth_max_iter}", Path.join(dir, "garment_smooth.txt")], smlog, timeout_s: t)
    smtxt = File.read!(Path.join(dir, "garment_smooth.txt"))
    [_, c0] = need!("reduce", smlog, smtxt, ~r/iteration 0: (\d+) crossing pairs/, "iteration 0")
    if c3 != 0 or not Regex.match?(~r/iteration \d+: 0 crossing pairs/, smtxt), do: raise(Gate, stage: "reduce", reason: "pinch crossings not resolved by smoothing (started with #{c0})", log: Path.join(dir, "garment_smooth.txt"))
    [_, moved] = need!("reduce", smlog, smtxt, ~r/final: (.*)/, "final line")
    gj = Path.join(dir, "garment_s2.obj")
    {c4, _} = godot(ctx, "jitter", [sm, gj, "#{cfgr.jitter_amp}", "#{p.jitter_seeds.garment}"], Path.join(dir, "garment_jitter.log"), timeout_s: t)
    if c4 != 0, do: raise(Gate, stage: "reduce", reason: "garment jitter failed", log: Path.join(dir, "garment_jitter.log"))

    gfinal = Path.join(a, "garment_red.obj")
    droplog = Path.join(dir, "garment_drop.log")

    shells_note =
      if p.drop_shell_ranks == [] do
        File.cp!(gj, gfinal)
        "no shells dropped"
      else
        {c5, dtxt} = godot(ctx, "shell_pick", [gj, gfinal, "--drop=" <> Enum.join(p.drop_shell_ranks, ",")], droplog, timeout_s: t)
        if c5 != 0, do: raise(Gate, stage: "reduce", reason: "shell_pick failed", log: droplog)
        [_, sin, kept, trs] = need!("reduce", droplog, dtxt, ~r/shells in: (\d+), kept: (\d+), triangles out: (\d+)/, "shell_pick line")
        if String.to_integer(sin) - length(p.drop_shell_ranks) != String.to_integer(kept), do: raise(Gate, stage: "reduce", reason: "drop list #{inspect(p.drop_shell_ranks)} does not match the #{sin} shells of this proxy", log: droplog)
        "shells #{sin} -> #{kept} (dropped ranks #{inspect(p.drop_shell_ranks)}), #{trs} triangles"
      end

    avatar_final = Path.join(a, "avatar_red.obj")
    {c6, _} = godot(ctx, "jitter", [as0, avatar_final, "#{cfgr.jitter_amp}", "#{p.jitter_seeds.avatar}"], Path.join(dir, "avatar_jitter.log"), timeout_s: t)
    if c6 != 0, do: raise(Gate, stage: "reduce", reason: "avatar jitter failed", log: Path.join(dir, "avatar_jitter.log"))

    # skeletons into the solver frame, no-fit list
    for k <- ["source", "target"] do
      lg = Path.join(dir, "#{k}_skeleton.log")
      {c7, _} = godot(ctx, "to_solver", [Path.join(ctx.cfg.export_dir, "#{k}_skeleton.obj"), Path.join(a, "#{k}_skeleton.obj"), sc], lg, timeout_s: t)
      if c7 != 0, do: raise(Gate, stage: "reduce", reason: "#{k} skeleton conversion failed", log: lg)
    end

    nf_arg =
      case p.nofit do
        {:below_y, y} -> "--below-z=#{y * ctx.cfg.scale}"
        {:height_fraction, f} -> "--height-fraction=#{f}"
      end

    nflog = Path.join(dir, "nofit.log")
    {c8, nftxt} = godot(ctx, "nofit", [gfinal, Path.join(a, "no-fit_red.txt"), nf_arg], nflog, timeout_s: t)
    if c8 != 0, do: raise(Gate, stage: "reduce", reason: "nofit failed", log: nflog)
    [_, nfline] = need!("reduce", nflog, nftxt, ~r/(vertices \d+, no-fit \d+, threshold [-\d.]+)/, "nofit line")

    # gates: closed, crossing-free, cross-file clear, planted shift must be caught
    chk = fn label, args ->
      lg = Path.join(dir, "check_#{label}.log")
      {c, tx} = godot(ctx, "mesh_check", ["--noweld" | args], lg, timeout_s: t)
      if c != 0, do: raise(Gate, stage: "reduce", reason: "mesh_check #{label} failed to run", log: lg)
      files = Regex.scan(~r/(\S+\.obj): (\d+) verts, (\d+) tris, (\d+) shells, (\d+) open edges, (\d+) non-manifold edges, signed volume (\S+)/, tx)
      [_, pairs, cross_file] = need!("reduce", lg, tx, ~r/CROSSINGS: (\d+) pairs total; (\d+) between files/, "CROSSINGS line")
      {files, String.to_integer(pairs), String.to_integer(cross_file), lg}
    end

    {af, ap, _, alog} = chk.("avatar", [avatar_final])
    {gf, gp, _, glog} = chk.("garment", [gfinal])
    {_, pp, pc, plog} = chk.("pair", [avatar_final, gfinal])
    {_, _, ctl, _} = chk.("pair_planted_shift", ["--shift=0,0,1.0", avatar_final, gfinal])

    Enum.each(af ++ gf, fn [_, name, _v, _t, _s, open, _nm, vol] ->
      if String.to_integer(open) != 0, do: raise(Gate, stage: "reduce", reason: "#{name} has #{open} open edges", log: glog)
      if String.starts_with?(vol, "-"), do: raise(Gate, stage: "reduce", reason: "#{name} has negative signed volume (inward winding)", log: glog)
    end)

    if ap != 0,
 do: raise(Gate, stage: "reduce", reason: "avatar proxy has #{ap} crossing pairs", log: alog)
    if gp != 0, do: raise(Gate, stage: "reduce", reason: "garment proxy has #{gp} crossing pairs", log: glog)
    if pp != 0 or pc != 0, do: raise(Gate, stage: "reduce", reason: "avatar and garment proxies cross (#{pc} pairs)", log: plog)
    if ctl == 0, do: raise(Gate, stage: "reduce", reason: "planted shift was not detected: the crossing checker cannot fail", log: Path.join(dir, "check_pair_planted_shift.log"))

    desc = fn [[_, _n, v, t, s, o, nm, vol]] -> "#{v} verts, #{t} tris, #{s} shells, #{o} open edges, #{nm} non-manifold edges, signed volume #{vol}" end

    {[
       "garment remesh res #{p.garment_res}: #{gv} vertices, #{gt} triangles; avatar remesh res #{p.avatar_res}: #{av} vertices, #{at} triangles",
       "garment pinch crossings #{c0} -> 0 by local smoothing (#{moved}); #{shells_note}",
       "no-fit: #{nfline}",
       "avatar proxy: " <> desc.(af) <> "; crossings 0",
       "garment proxy: " <> desc.(gf) <> "; crossings 0",
       "avatar vs garment: 0 crossing pairs; planted shift (0,0,1) detected #{ctl} pairs"
     ], [avatar_final, gfinal, Path.join(a, "no-fit_red.txt"), Path.join(a, "source_skeleton.obj"), Path.join(a, "target_skeleton.obj")]}
  end

  defp do_stage("cloth-fit", ctx, dir) do
    p = ctx.p

    case Map.get(p, :external_cloth_fit) do
      nil -> cloth_fit_run(ctx, dir)
      ext ->
        {_, run, steps} = cloth_fit_paths(ctx)
        if steps.n < 0, do: raise(Gate, stage: "cloth-fit", reason: "external run has no step_garment files: #{ext.run}", log: run)
        {["external cloth-fit run (earlier session, not rerun): #{run}, last step #{steps.n}"], [Path.join(run, "setup.json")]}
    end
  end

  defp cloth_fit_run(ctx, dir) do
    cf = ctx.cfg.cloth_fit
    fit = Path.join(dir, "fit")
    assets = Path.join(stage_dir(ctx, "reduce"), "assets")
    File.mkdir_p!(fit)
    setup = ctx.cfg.polyfem_setup |> Map.merge(%{"avatar_mesh_path" => "../assets/avatar_red.obj", "target_skeleton_path" => "../assets/target_skeleton.obj", "avatar_skin_weights_path" => "", "garment_mesh_path" => "../assets/garment_red.obj", "source_skeleton_path" => "../assets/source_skeleton.obj", "no_fit_spec_path" => "../assets/no-fit_red.txt"})
    File.write!(Path.join(fit, "setup.json"), JSON.encode!(setup))
    unless File.exists?(Path.join(dir, "assets")), do: File.ln_s!(assets, Path.join(dir, "assets"))

    case Map.get(ctx.p, :adopt_cloth_fit) do
      nil ->
        log = Path.join(fit, "log")
        code = run_bg(rel(ctx, ctx.cfg.polyfem), ["-j", "setup.json", "-o", "."], log, cwd: fit, timeout_s: cf.timeout_s, poll_ms: cf.poll_s * 1000, resume: true)
        txt = File.read!(log)
        if code != 0 or not String.contains?(txt, cf.success_line), do: raise(Gate, stage: "cloth-fit", reason: "PolyFEM exit #{code}, success line #{inspect(cf.success_line)} #{if String.contains?(txt, cf.success_line), do: "present", else: "absent"}" <> if(String.contains?(txt, "initial solution has intersections"), do: " (initial solution has intersections)", else: ""), log: log)
        {_, _, steps} = cloth_fit_paths(ctx)
        {["PolyFEM exit 0, #{inspect(cf.success_line)}, last step #{steps.n}"], [steps.garment]}

      adopt ->
        # An existing run counts only if its inputs are byte-identical to the assets this driver made and its setup.json is the same.
        theirs = Path.expand(Path.join([adopt, "..", "assets"]))
        ids = fn p -> p |> File.read!() |> String.split() end

        same =
          Enum.all?(~w(avatar_red.obj garment_red.obj source_skeleton.obj target_skeleton.obj), fn f -> hash_file(Path.join(assets, f)) == hash_file(Path.join(theirs, f)) end) and
            ids.(Path.join(assets, "no-fit_red.txt")) == ids.(Path.join(theirs, "no-fit_red.txt"))

        same_setup = hash_file(Path.join(adopt, "setup.json")) == hash_file(Path.join(fit, "setup.json")) or JSON.decode!(File.read!(Path.join(adopt, "setup.json"))) == JSON.decode!(File.read!(Path.join(fit, "setup.json")))
        unless same and same_setup, do: raise(Gate, stage: "cloth-fit", reason: "adopted run #{adopt} was not made from these assets/setup (inputs differ): not adopting", log: adopt)
        txt = File.read!(Path.join(adopt, "log"))
        unless String.contains?(txt, cf.success_line), do: raise(Gate, stage: "cloth-fit", reason: "adopted run did not finish with #{inspect(cf.success_line)}", log: Path.join(adopt, "log"))
        {_, _, steps} = cloth_fit_paths(ctx)
        {["adopted finished PolyFEM run #{adopt} (assets and setup.json identical to this driver's), last step #{steps.n}"], [steps.garment]}
    end
  end

  defp do_stage("map-back", ctx, dir) do
    mb = ctx.cfg.map_back
    {assets, run, steps} = cloth_fit_paths(ctx)
    log = Path.join(dir, "map_back.log")
    deltas = Path.join(dir, "deltas.bin")
    own = Path.join(ctx.cfg.export_dir, "garment_own_#{ctx.piece}.bin")
    args = [elf(ctx, :remesh_elf), Path.join(assets, "garment_red.obj"), steps.garment, Path.join(assets, "avatar_red.obj"), Path.join(run, "target_avatar.obj"), own, deltas, "#{ctx.cfg.scale}"]
    {code, txt} = godot(ctx, "map_back", args, log, timeout_s: mb.timeout_s)
    if code != 0, do: raise(Gate, stage: "map-back", reason: "map_back.gd exit #{code}", log: log)
    [_, resid] = need!("map-back", log, txt, ~r/frame: .* max residual (\S+) \(solver units\)/, "frame line")
    [_, pm, dmiss] = need!("map-back", log, txt, ~r/control \(proxy vertices as queries\): max position miss (\S+) m, max delta miss (\S+) m/, "control line")
    if String.to_float(resid) > mb.max_frame_residual, do: raise(Gate, stage: "map-back", reason: "similarity frame fit residual #{resid} > #{mb.max_frame_residual}", log: log)
    if String.to_float(pm) > mb.max_control_miss_m or String.to_float(dmiss) > mb.max_control_miss_m, do: raise(Gate, stage: "map-back", reason: "planted control: position miss #{pm}, delta miss #{dmiss}", log: log)
    keep = Regex.scan(~r/^(proxy displacement.*|query->proxy.*|garment delta.*|symmetry:.*|original garment.*)$/m, txt) |> Enum.map(fn [_, l] -> l end)
    tgt = Path.join(dir, "deltas_target.bin")
    File.rename!(tgt, Path.join(dir, "target.bin"))
    extra = reference_compare(ctx, deltas)
    {["last cloth-fit step #{steps.n}", "frame fit residual #{resid}; control: position miss #{pm} m, delta miss #{dmiss} m"] ++ keep ++ extra, [deltas, Path.join(dir, "target.bin")]}
  end

  # Cross-check against an earlier result (skirt): per-vertex difference between this map-back and the reference deltas.
  defp reference_compare(ctx, deltas) do
    case get_in(ctx.p, [:external_cloth_fit, :reference_deltas]) do
      nil -> []
      path ->
        ref = JSON.decode!(File.read!(path))["deltas_unity_m"]
        bin = File.read!(deltas)
        n = length(ref)
        if byte_size(bin) != n * 12, do: raise(Gate, stage: "map-back", reason: "reference has #{n} deltas, this run #{div(byte_size(bin), 12)}", log: path)

        {sum, mx, sum_ref} =
          ref
          |> Enum.with_index()
          |> Enum.reduce({0.0, 0.0, 0.0}, fn {[rx, ry, rz], i}, {s, m, sr} ->
            off = i * 12
            <<_::binary-size(^off), x::float-32-little, y::float-32-little, z::float-32-little, _::binary>> = bin
            d = :math.sqrt((x - rx) * (x - rx) + (y - ry) * (y - ry) + (z - rz) * (z - rz))
            {s + d, max(m, d), sr + :math.sqrt(rx * rx + ry * ry + rz * rz)}
          end)

        ["vs the earlier session's deltas (#{Path.basename(path)}, #{n} vertices): mean |difference| #{fmt(sum / n)} m, max #{fmt(mx)} m; their mean |delta| #{fmt(sum_ref / n)} m"]
    end
  end

  defp fmt(x), do: :erlang.float_to_binary(x * 1.0, decimals: 6)

  defp do_stage("bone-fit", ctx, dir) do
    bf = ctx.cfg.bone_fit_cfg
    mb = stage_dir(ctx, "map-back")

    results =
      for mode <- bf.modes do
        out = Path.join(dir, Atom.to_string(mode))
        File.mkdir_p!(out)
        log = Path.join(dir, "bone_fit_#{mode}.log")

        base = ["--fbx", ctx.cfg.fbx, "--mesh", ctx.p.mesh, "--orig", Path.join(ctx.cfg.export_dir, "garment_own_#{ctx.piece}.bin"), "--target", Path.join(mb, "target.bin"), "--out", out, "--prefab", ctx.cfg.prefab, "--usd-elf", elf(ctx, :usd_elf), "--name", ctx.piece]
        same = if ctx.p.same_piece == [], do: [], else: ["--same-piece", Enum.join(ctx.p.same_piece, ",")]
        lam = if bf.lambda, do: ["--lambda", "#{bf.lambda}"], else: []

        modeargs =
          case mode do
            :exclusive -> []
            :clothing_free -> ["--clothing-free", "1"]
            :analysis -> ["--exclusive", "0", "--analysis-not-applied", "1"]
          end

        full = ["--headless", "--path", proj(ctx), "-s", rel(ctx, ctx.cfg.bone_fit), "--"] ++ base ++ same ++ lam ++ modeargs
        code = run_bg(engine(ctx), full, log, timeout_s: bf.timeout_s)
        txt = File.read!(log)
        fails = Regex.scan(~r/^FAIL .*$/m, txt) |> Enum.map(&hd/1)
        if code != 0 or fails != [], do: raise(Gate, stage: "bone-fit", reason: "bone_fit.gd (#{mode}) exit #{code}; #{length(fails)} failed checks: #{Enum.join(Enum.take(fails, 3), " | ")}", log: log)
        passes = length(Regex.scan(~r/^PASS /m, txt))
        key = Regex.scan(~r/^((?:full affine|TRS from the affine|RT, fitted|END TO END|mode:|frozen \(kept|ASSERT|PASS ASSERT).*)$/m, txt) |> Enum.map(fn [_, l] -> l end) |> Enum.uniq() |> Enum.take(14)
        check_expect(ctx, mode, txt, log)
        outs = out |> File.ls!() |> Enum.map(&Path.join(out, &1)) |> Enum.filter(&String.ends_with?(&1, [".usdz", ".usda", ".txt"]))
        {["mode #{mode}: exit #{code}, #{passes} PASS checks, 0 FAIL"] ++ Enum.map(key, &("  " <> &1)), outs}
      end

    {Enum.flat_map(results, &elem(&1, 0)), Enum.flat_map(results, &elem(&1, 1))}
  end

  # Optional reproduction gate (config piece key :expect): the rigid-TRS fit must match an earlier result within a tolerance.
  defp check_expect(ctx, :exclusive, txt, log) do
    case Map.get(ctx.p, :expect) do
      nil ->
        :ok

      e ->
        [_, mean, expl] = need!("bone-fit", log, txt, ~r/RT, fitted rigid directly.*?mean (\S+)\s+p99.*?explained (\S+)%/, "rigid TRS result line")
        dm = abs(String.to_float(mean) * 1000 - e.rt_mean_mm)
        de = abs(String.to_float(expl) - e.rt_explained_pct)

        if dm > e.tol_mm or de > e.tol_pct,
          do: raise(Gate, stage: "bone-fit", reason: "does not reproduce the earlier result: mean #{fmt(String.to_float(mean) * 1000)} mm (expected #{e.rt_mean_mm} +- #{e.tol_mm}), explained #{expl}% (expected #{e.rt_explained_pct} +- #{e.tol_pct})", log: log)
    end
  end

  defp check_expect(_, _, _, _), do: :ok

  defp do_stage("report", ctx, dir) do
    rows =
      for s <- ~w(inputs csg-clean proxies reduce cloth-fit map-back bone-fit), File.exists?(Path.join(stage_dir(ctx, s), "manifest.term")) do
        m = :erlang.binary_to_term(File.read!(Path.join(stage_dir(ctx, s), "manifest.term")))
        "## #{s}  (#{m.status}, #{m.seconds} s, at #{m.at})\n\n" <> Enum.map_join(m.numbers, "\n", &("- " <> &1)) <> "\n\nartifacts:\n" <> Enum.map_join(m.outputs, "\n", &("- " <> &1)) <> "\n"
      end

    body = "# #{ctx.piece}: automated fit report\n\nConfig: #{ctx.cfg_path}. Every gate listed below passed (the driver stops at the first failure).\n\n" <> Enum.join(rows, "\n")
    path = Path.join(dir, "REPORT-auto.md")
    File.write!(path, body)
    {["wrote #{path}"], [path]}
  end
end

MapleFit.main(System.argv())
