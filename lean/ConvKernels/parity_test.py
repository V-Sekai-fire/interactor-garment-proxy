# Gate: the Lean-emitted Slang conv equals PyTorch's conv in forward and in both gradients, for stride 1 and 2; a shifted padding must be caught.
import torch, torch.nn.functional as F, conv_torch
torch.manual_seed(0); dev = "cuda"; ok = True
def rel(a, b): return float((a - b).abs().max() / b.abs().max().clamp(min=1e-6))
for stride in (1, 2):
    for pad in (0, 1):
        x = torch.randn(2, 3, 37, 41, device=dev, requires_grad=True); w = torch.randn(5, 3, 3, 3, device=dev, requires_grad=True); b = torch.randn(5, device=dev, requires_grad=True)
        y = conv_torch.conv2d(x, w, b, stride, pad); g = torch.randn_like(y); y.backward(g); mine = [y.detach(), x.grad.clone(), w.grad.clone(), b.grad.clone()]
        x2, w2, b2 = [t.detach().clone().requires_grad_() for t in (x, w, b)]; y2 = F.conv2d(x2, w2, b2, stride, pad); y2.backward(g); ref = [y2.detach(), x2.grad, w2.grad, b2.grad]
        errs = [rel(a, r) for a, r in zip(mine, ref)]; good = max(errs) < 1e-4; ok &= good
        print(f"stride {stride} pad {pad}: forward {errs[0]:.1e} dx {errs[1]:.1e} dw {errs[2]:.1e} db {errs[3]:.1e}", "PASS" if good else "FAIL")
# negative control: one extra pixel of padding in the kernel must disagree with PyTorch
x = torch.randn(2, 3, 37, 41, device=dev); w = torch.randn(5, 3, 3, 3, device=dev); b = torch.randn(5, device=dev)
bad = conv_torch.LeanConv2d.apply(x, w, b, 1, 1, 1); ref = F.conv2d(x, w, b, 1, 1)
d = rel(bad[:, :, :ref.shape[2], :ref.shape[3]], ref) if bad.shape[2] >= ref.shape[2] else 1.0
print("control (padding shifted by one) differs:", f"{d:.2e}", "PASS" if d > 1e-2 else "FAIL (control did not fail)"); ok &= d > 1e-2
print("ALL GATES PASS" if ok else "SOME GATES FAILED")
