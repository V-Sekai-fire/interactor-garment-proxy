# Parameters for tools/maple_fit.exs. One map; every number a stage uses is here (no magic numbers in the stages).
# Paths that start with "./" are relative to the repository root (the directory that holds tools/).
# Evaluated with Code.eval_file/1; the last expression is the config.
%{
  # ---- tools --------------------------------------------------------------------------------------------------------
  # Double-precision engine with the Sandbox class and CSG vertex attributes (headless).
  engine: "/Users/ernest.lee/Desktop/entities-persona/engine-r4-party/bin/godot.macos.editor.double.arm64",
  usd_elf: "./tools/bin/usd.elf",
  remesh_elf: "./tools/bin/remesh.elf",
  polyfem: "./tools/bin/PolyFEM_bin",
  # The bone fit tool (read, never edited by the driver). Default: the committed one in this repository.
  bone_fit: "./godot/bone_fit.gd",
  # Scripts the stages run (copied into <work>/godot_project so the engine has a project to open).
  scripts: ~w(union_proxy mesh_check crosslib fix_smooth jitter to_solver shell_pick nofit map_back obj_to_usdz run_remesh csg_clean_usd),

  # ---- Unity inputs (the exporter ChibifireFitExport.cs output, a manual menu action) ------------------------------
  export_dir: "/Users/ernest.lee/.hermes/cache/scratch/garment-proxy/fit-work/export",
  fbx: "/Users/ernest.lee/Desktop/entities-fox/character-fox/Assets/PLUMARIUM/Nutty Mapley_SHINANO/FBX/Nutty Mapley_SHINANO.fbx",
  prefab: "/Users/ernest.lee/Desktop/entities-fox/character-fox/Assets/PLUMARIUM/Nutty Mapley_SHINANO/Nutty Mapley_SHINANO.prefab",

  # ---- cloth-fit frame -----------------------------------------------------------------------------------------------
  # Unity metres, Y up -> solver frame (X, Y, Z) = scale * (x, -z, y). The example avatar is about 6 units tall; dhat and
  # voxel_size below assume that.
  scale: 5,

  # ---- CSG clean (stage 1) -------------------------------------------------------------------------------------------
  csg_clean: %{thickness_m: 0.001, timeout_s: 1800},

  # ---- proxies (stage 2): CSG union of shells thickened inward -------------------------------------------------------
  proxy: %{timeout_s: 1800},

  # ---- reduction (stage 3) -------------------------------------------------------------------------------------------
  reduce: %{
    # remesh.elf options bitmask: 0 = plain solid voxel remesh (1 = shell, 2 = solve were worse here, see README).
    options: 0,
    jitter_amp: 1.0e-4,
    timeout_s: 1800
  },

  # ---- cloth-fit (stage 4): setup.json, one-to-one with the run that produced the Maple results -----------------------
  polyfem_setup: %{
    "incremental_steps" => 2,
    "similarity_penalty_weight" => 1,
    "curvature_penalty_weight" => 0.01,
    "twist_penalty_weight" => 0.01,
    "curve_center_target_weight" => 1,
    "fit_weight" => 2,
    "symmetry_weight" => 0,
    "curve_size_weight" => 0,
    "voxel_size" => 0.01,
    "is_skirt" => false,
    "curve_center_target_automatic_bone_generation" => true,
    "contact" => %{"enabled" => true, "dhat" => 0.002},
    "solver" => %{
      "max_threads" => 10,
      "linear" => %{"solver" => ["Eigen::PardisoLDLT", "Eigen::AccelerateLDLT", "Eigen::SimplicialLDLT"]},
      "augmented_lagrangian" => %{
        "initial_weight" => 1,
        "max_weight" => 1_000_000.0,
        "eta" => 1,
        "nonlinear" => %{"grad_norm" => 1, "max_iterations" => 50}
      },
      "nonlinear" => %{
        "Newton" => %{
          "use_psd_projection" => true,
          "use_psd_projection_in_regularized" => true,
          "reg_weight_max" => 1.0e16,
          "reg_weight_min" => 1,
          "reg_weight_inc" => 10_000.0
        },
        "grad_norm" => 0.01,
        "line_search" => %{
          "max_step_size_limiter" => 0.5,
          "use_grad_norm_tol" => 0.0001,
          "method" => "Backtracking",
          "min_step_size" => 1.0e-8
        },
        "max_iterations" => 5000
      },
      "contact" => %{
        "CCD" => %{"broad_phase" => "BVH", "max_iterations" => 200, "tolerance" => 0.001},
        "barrier_stiffness" => 100_000_000.0
      }
    },
    "output" => %{"skip_frame" => 2, "log" => %{"level" => "debug"}}
  },
  cloth_fit: %{poll_s: 5, timeout_s: 14_400, success_line: "Garment retargeting succeeded!"},

  # ---- map back (stage 5) --------------------------------------------------------------------------------------------
  map_back: %{
    timeout_s: 1800,
    # gates: frame fit residual (solver units) and the planted control (proxy vertices as queries must come back exactly)
    max_frame_residual: 1.0e-9,
    max_control_miss_m: 1.0e-6
  },

  # ---- bone fit (stage 6) --------------------------------------------------------------------------------------------
  # Modes: :exclusive (only clothing bones no other piece weights move; the deliverable) and :clothing_free (every clothing
  # bone except the Miroir-merged ones moves; analysis only). The multi-piece form is run when bone_fit.gd has --piece.
  bone_fit_cfg: %{modes: [:exclusive], timeout_s: 3600, lambda: nil},

  # ---- pieces ---------------------------------------------------------------------------------------------------------
  pieces: %{
    "BKimono" => %{
      mesh: "(B)Kimono",
      same_piece: ["(B)Kimono_Big"],
      garment_thickness_m: 0.012,
      avatar_thickness_m: 0.012,
      garment_res: 192,
      avatar_res: 160,
      target_triangles: 25_000,
      smooth_max_iter: 60,
      jitter_seeds: %{garment: 7, avatar: 11},
      # Shell ranks (0 = largest) of the garment proxy dropped before the solve. PolyFEM reported "initial solution has
      # intersections" with them; found by running the solver's initial-solution check on one shell at a time. Dropped
      # shells keep the displacement of the nearest proxy surface at map-back.
      drop_shell_ranks: [6, 18, 22],
      # Loose (no-fit) parts: Unity y below the source hip joint (m).
      nofit: {:below_y, 0.7935},
      # Optional: adopt an existing cloth-fit run if its inputs are byte-identical to this driver's stage-3 assets.
      adopt_cloth_fit: nil
    },
    "ASkirt" => %{
      mesh: "(A)Skirt",
      same_piece: [],
      # The skirt cloth-fit was run in an earlier session with proxies this driver did not make (df_proxy). Stages 1 to 4 are
      # not rerun for it: the driver starts at map-back from that run (assets + run directory below).
      external_cloth_fit: %{
        assets: "/private/tmp/claude-502/-Users-ernest-lee-Desktop-entities-fabric/a12cd50d-7401-45f9-bb49-2b5152fd12b0/scratchpad/maple_skirt/assets",
        run: "/private/tmp/claude-502/-Users-ernest-lee-Desktop-entities-fabric/a12cd50d-7401-45f9-bb49-2b5152fd12b0/scratchpad/maple_skirt/run3",
        # Earlier result to cross-check the map-back against (JSON {"deltas_unity_m": [[dx,dy,dz], ...]}).
        reference_deltas: "/private/tmp/claude-502/-Users-ernest-lee-Desktop-entities-fabric/a12cd50d-7401-45f9-bb49-2b5152fd12b0/scratchpad/maple_skirt/clothfit_deltas_unity.json"
      },
      # Reproduction gate: the skirt agent's exclusive rigid-TRS fit (mean 36.650 mm, 83.9544 % explained) within a tolerance.
      expect: %{rt_mean_mm: 36.65, rt_explained_pct: 83.9544, tol_mm: 0.05, tol_pct: 0.01},
      garment_thickness_m: 0.005,
      avatar_thickness_m: 0.012,
      garment_res: 96,
      avatar_res: 160,
      target_triangles: 20_000,
      smooth_max_iter: 60,
      jitter_seeds: %{garment: 7, avatar: 11},
      drop_shell_ranks: [],
      nofit: {:height_fraction, 0.84},
      adopt_cloth_fit: nil
    }
  }
}
