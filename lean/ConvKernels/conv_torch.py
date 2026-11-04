# PyTorch autograd function whose forward and backward are the Lean-emitted Slang kernels (conv_lean.slang via conv_wrap.slang, slangtorch).
import os, torch, slangtorch
_here = os.path.dirname(os.path.abspath(__file__)); _m = None
def module():
    global _m
    if _m is None: _m = slangtorch.loadModule(os.path.join(_here, "conv_wrap.slang"))
    return _m
def _launch(k, total): k.launchRaw(blockSize=(256, 1, 1), gridSize=((total + 255) // 256, 1, 1))
class LeanConv2d(torch.autograd.Function):
    @staticmethod
    def forward(ctx, x, w, b, stride, pad, flip=0):
        x, w, b = x.contiguous(), w.contiguous(), b.contiguous()
        oh = (x.shape[2] + 2 * pad - w.shape[2]) // stride + 1; ow = (x.shape[3] + 2 * pad - w.shape[3]) // stride + 1
        y = torch.empty(x.shape[0], w.shape[0], oh, ow, device=x.device, dtype=torch.float32)
        _launch(module().conv_fwd(x=x, w=w, b=b, y=y, stride=stride, pad=pad + flip), y.numel()); ctx.save_for_backward(x, w); ctx.sp = (stride, pad, flip); return y
    @staticmethod
    def backward(ctx, dy):
        x, w = ctx.saved_tensors; stride, pad, flip = ctx.sp; dy = dy.contiguous()
        dx = torch.empty_like(x); dw = torch.empty_like(w); m = module()
        _launch(m.conv_bwd_input(dy=dy, w=w, dx=dx, stride=stride, pad=pad + flip), dx.numel())
        _launch(m.conv_bwd_weight(dy=dy, x=x, dw=dw, stride=stride, pad=pad + flip), dw.numel())
        return dx, dw, dy.sum((0, 2, 3)), None, None, None
def conv2d(x, w, b, stride=1, pad=0): return LeanConv2d.apply(x, w, b, stride, pad)
