"""Export DeepLabV3+ to ONNX (input 1x3xHxW RGB 0..255, output class map 1x(H/4)x(W/4) int32)
and write calibration raws (NHWC float32). Usage: python3 scripts/export.py H W"""
import os, sys, glob, numpy as np, torch
from PIL import Image
sys.path.insert(0, '.')
from model import load
H, W = int(sys.argv[1]), int(sys.argv[2])
tag = f'{H}x{W}'


class WithArgmax(torch.nn.Module):
    def __init__(s, m): super().__init__(); s.m = m
    def forward(s, x):
        return s.m(x).argmax(1).to(torch.int32)


torch.onnx.export(WithArgmax(load()), torch.zeros(1, 3, H, W), f'deeplab_ade_{tag}.onnx',
                  input_names=['images'], output_names=['classes'], opset_version=17, dynamo=False)
cal = f'calib_{tag}'
os.makedirs(cal, exist_ok=True)
lst = []
for f in sorted(glob.glob('ade_val/*.jpg'))[::2][:50]:  # even = calibration, odd = test
    a = np.array(Image.open(f).convert('RGB').resize((W, H), Image.BILINEAR), np.float32)[None]
    p = os.path.abspath(f'{cal}/{os.path.basename(f)[:-4]}.raw')
    a.tofile(p)
    lst.append(p)
open(f'{cal}/list.txt', 'w').write('\n'.join(lst) + '\n')
print('exported', tag, len(lst), 'calibration images')
