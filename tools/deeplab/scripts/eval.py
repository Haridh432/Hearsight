"""mIoU of the fp32 model on the fetched ADE20K val pairs at given input sizes."""
import sys, glob, json, numpy as np, torch, torch.nn.functional as F
from PIL import Image
sys.path.insert(0, '.')
from model import load

def predict(m, img, size):
    x = torch.from_numpy(np.array(img.resize((size[1], size[0]), Image.BILINEAR), np.float32)).permute(2, 0, 1)[None]
    with torch.no_grad():
        y = m(x)
    return F.interpolate(y, size=(img.height, img.width), mode='bilinear', align_corners=False).argmax(1)[0].numpy()

def miou(m, files, size):
    inter, union = np.zeros(150), np.zeros(150)
    for f in files:
        img = Image.open(f).convert('RGB')
        gt = np.array(Image.open(f[:-4] + '.png')).astype(np.int64) - 1   # reduce_zero_label
        s = size or (512, round(512 * img.width / img.height))
        p = predict(m, img, s)
        v = gt >= 0
        for c in np.unique(np.concatenate([gt[v], p[v]])):
            a, b = gt[v] == c, p[v] == c
            inter[c] += (a & b).sum(); union[c] += (a | b).sum()
    ok = union > 0
    return (inter[ok] / union[ok]).mean(), inter.sum() / max(1, 1)

if __name__ == '__main__':
    files = sorted(glob.glob('ade_val/*.jpg'))
    m = load()
    for size in [None, (480, 640), (384, 512), (240, 320)]:
        print(size or 'short side 512 (reference)', 'mIoU %.3f' % miou(m, files, size)[0], flush=True)
