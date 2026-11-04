import ConvKernels
/-- `emit_conv <dir>` writes `<dir>/conv_lean.slang`. -/
def main (args : List String) : IO UInt32 := do
  let dir := args.headD "."
  IO.FS.createDirAll dir
  IO.FS.writeFile s!"{dir}/conv_lean.slang" ConvKernels.text
  IO.println s!"wrote {dir}/conv_lean.slang ({ConvKernels.text.length} chars)"
  return 0
