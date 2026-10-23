"""Fetch N ADE20K validation image/annotation pairs from the remote zip via HTTP range requests."""
import io, sys, zipfile, urllib.request
URL = 'https://data.csail.mit.edu/places/ADEchallenge/ADEChallengeData2016.zip'
N, out = int(sys.argv[1]), sys.argv[2]

class RangeFile(io.RawIOBase):
    def __init__(s, url):
        s.url, s.pos = url, 0
        r = urllib.request.urlopen(urllib.request.Request(url, method='HEAD')); s.size = int(r.headers['Content-Length'])
    def seekable(s): return True
    def readable(s): return True
    def tell(s): return s.pos
    def seek(s, off, wh=0):
        s.pos = off if wh == 0 else s.pos + off if wh == 1 else s.size + off; return s.pos
    def read(s, n=-1):
        if n < 0: n = s.size - s.pos
        if n == 0 or s.pos >= s.size: return b''
        end = min(s.size, s.pos + n) - 1
        d = urllib.request.urlopen(urllib.request.Request(s.url, headers={'Range': f'bytes={s.pos}-{end}'})).read()
        s.pos += len(d); return d
    def readinto(s, b):
        d = s.read(len(b)); b[:len(d)] = d; return len(d)

z = zipfile.ZipFile(io.BufferedReader(RangeFile(URL), buffer_size=1 << 20))
imgs = sorted(n for n in z.namelist() if '/images/validation/' in n and n.endswith('.jpg'))
step = max(1, len(imgs) // N)
import os
for n in imgs[::step][:N]:
    a = n.replace('/images/', '/annotations/').replace('.jpg', '.png')
    for m in (n, a):
        open(os.path.join(out, os.path.basename(m)), 'wb').write(z.read(m))
print(len(imgs), 'val images; fetched', len(os.listdir(out)) // 2)
