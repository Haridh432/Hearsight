"""Quick check: (1) INT8 phone output vs fp32 on one image; (2) fp32 mIoU on 20 held-out images at several sizes."""
import sys, glob, numpy as np, torch, torch.nn.functional as F
from PIL import Image
sys.path.insert(0, '.')
from model import load
m = load()
if len(sys.argv) > 1:
    q = np.fromfile(sys.argv[1], np.float32).astype(np.int64).reshape(96, 128)
    a = np.fromfile('calib/ADE_val_00000001.raw', np.float32).reshape(1, 384, 512, 3)
    with torch.no_grad(): f = m(torch.from_numpy(a).permute(0, 3, 1, 2)).argmax(1)[0].numpy()
    print('INT8 vs fp32 pixel agreement: %.3f' % (q == f).mean())
if len(sys.argv) > 2: sys.exit()
files = sorted(glob.glob('ade_val/*.jpg'))[1::2][:20]
for size in [(384, 512), (288, 384), (240, 320)]:
    I, U = np.zeros(150), np.zeros(150); acc = []
    for fn in files:
        img = Image.open(fn).convert('RGB')
        gt = np.array(Image.open(fn[:-4] + '.png')).astype(np.int64) - 1
        x = torch.from_numpy(np.array(img.resize(size[::-1], Image.BILINEAR), np.float32)).permute(2, 0, 1)[None]
        with torch.no_grad(): y = m(x)
        p = F.interpolate(y, size=gt.shape, mode='bilinear', align_corners=False).argmax(1)[0].numpy()
        v = gt >= 0; acc.append((p[v] == gt[v]).mean())
        for c in np.unique(gt[v]):
            I[c] += ((p == c) & (gt == c) & v).sum(); U[c] += (((p == c) | (gt == c)) & v).sum()
    ok = U > 0
    print(size, 'pixel acc %.3f  mIoU(present classes) %.3f' % (np.mean(acc), (I[ok] / U[ok]).mean()), flush=True)
