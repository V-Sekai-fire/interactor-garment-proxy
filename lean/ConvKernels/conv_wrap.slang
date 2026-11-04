// Thin CUDA kernels over the Lean-emitted functions in conv_lean.slang (one thread per output element).
import conv_lean;

[AutoPyBindCUDA]
[CUDAKernel]
void conv_fwd(TensorView<float> x, TensorView<float> w, TensorView<float> b, TensorView<float> y, uint stride, uint pad)
{
    uint t = cudaBlockIdx().x * cudaBlockDim().x + cudaThreadIdx().x;
    uint OC = y.size(1), OH = y.size(2), OW = y.size(3);
    if (t >= y.size(0) * OC * OH * OW) return;
    uint ox = t % OW, oy = (t / OW) % OH, oc = (t / (OW * OH)) % OC, n = t / (OW * OH * OC);
    y[uint4(n, oc, oy, ox)] = conv_fwd_at(x, w, b, n, oc, oy, ox, w.size(1), w.size(2), w.size(3), stride, pad, x.size(2), x.size(3));
}

[AutoPyBindCUDA]
[CUDAKernel]
void conv_bwd_input(TensorView<float> dy, TensorView<float> w, TensorView<float> dx, uint stride, uint pad)
{
    uint t = cudaBlockIdx().x * cudaBlockDim().x + cudaThreadIdx().x;
    uint C = dx.size(1), H = dx.size(2), W = dx.size(3);
    if (t >= dx.size(0) * C * H * W) return;
    uint ix = t % W, iy = (t / W) % H, ic = (t / (W * H)) % C, n = t / (W * H * C);
    dx[uint4(n, ic, iy, ix)] = conv_bwd_input_at(dy, w, n, ic, iy, ix, w.size(0), w.size(2), w.size(3), stride, pad, dy.size(2), dy.size(3));
}

[AutoPyBindCUDA]
[CUDAKernel]
void conv_bwd_weight(TensorView<float> dy, TensorView<float> x, TensorView<float> dw, uint stride, uint pad)
{
    uint t = cudaBlockIdx().x * cudaBlockDim().x + cudaThreadIdx().x;
    uint IC = dw.size(1), KH = dw.size(2), KW = dw.size(3);
    if (t >= dw.size(0) * IC * KH * KW) return;
    uint kx = t % KW, ky = (t / KW) % KH, ic = (t / (KW * KH)) % IC, oc = t / (KW * KH * IC);
    dw[uint4(oc, ic, ky, kx)] = conv_bwd_weight_at(dy, x, oc, ic, ky, kx, x.size(0), dy.size(2), dy.size(3), stride, pad, x.size(2), x.size(3));
}
