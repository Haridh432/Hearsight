"""Pure-PyTorch DeepLabV3+ (MobileNetV2, output stride 8) matching the
MMSegmentation v0.x checkpoint deeplabv3plus_m-v2-d8_512x512_160k_ade20k.
Only the inference path (decode head); the auxiliary head is dropped."""
import torch
import torch.nn as nn
import torch.nn.functional as F

MEAN = [123.675, 116.28, 103.53]
STD = [58.395, 57.12, 57.375]


class ConvModule(nn.Module):
    def __init__(self, cin, cout, k, stride=1, padding=0, dilation=1, groups=1, act='relu'):
        super().__init__()
        self.conv = nn.Conv2d(cin, cout, k, stride, padding, dilation, groups, bias=False)
        self.bn = nn.BatchNorm2d(cout)
        self.act = {'relu': nn.ReLU(), 'relu6': nn.ReLU6(), None: nn.Identity()}[act]

    def forward(self, x):
        return self.act(self.bn(self.conv(x)))


class InvertedResidual(nn.Module):
    def __init__(self, cin, cout, stride, expand, dilation):
        super().__init__()
        hidden = int(round(cin * expand))
        self.use_res = stride == 1 and cin == cout
        layers = []
        if expand != 1:
            layers.append(ConvModule(cin, hidden, 1, act='relu6'))
        layers += [
            ConvModule(hidden, hidden, 3, stride, dilation, dilation, hidden, act='relu6'),
            ConvModule(hidden, cout, 1, act=None),
        ]
        self.conv = nn.Sequential(*layers)

    def forward(self, x):
        return x + self.conv(x) if self.use_res else self.conv(x)


class MobileNetV2(nn.Module):
    arch = [[1, 16, 1], [6, 24, 2], [6, 32, 3], [6, 64, 4], [6, 96, 3], [6, 160, 3], [6, 320, 1]]
    strides = (1, 2, 2, 1, 1, 1, 1)
    dilations = (1, 1, 1, 2, 2, 4, 4)
    out_indices = (1, 2, 4, 6)

    def __init__(self, dilate_all_blocks):
        super().__init__()
        self.conv1 = ConvModule(3, 32, 3, 2, 1, act='relu6')
        cin = 32
        self.layer_names = []
        for i, (e, c, n) in enumerate(self.arch):
            blocks = []
            for j in range(n):
                d = self.dilations[i] if (j == 0 or dilate_all_blocks) else 1
                blocks.append(InvertedResidual(cin, c, self.strides[i] if j == 0 else 1, e, d))
                cin = c
            name = f'layer{i + 1}'
            self.add_module(name, nn.Sequential(*blocks))
            self.layer_names.append(name)

    def forward(self, x):
        x = self.conv1(x)
        outs = []
        for i, name in enumerate(self.layer_names):
            x = getattr(self, name)(x)
            if i in self.out_indices:
                outs.append(x)
        return outs


class DSConv(nn.Module):
    def __init__(self, cin, cout, dilation=1):
        super().__init__()
        self.depthwise_conv = ConvModule(cin, cin, 3, 1, dilation, dilation, cin)
        self.pointwise_conv = ConvModule(cin, cout, 1)

    def forward(self, x):
        return self.pointwise_conv(self.depthwise_conv(x))


class Head(nn.Module):
    def __init__(self, num_classes=150):
        super().__init__()
        self.image_pool = nn.Sequential(nn.AdaptiveAvgPool2d(1), ConvModule(320, 512, 1))
        self.aspp_modules = nn.ModuleList([ConvModule(320, 512, 1)] +
                                          [DSConv(320, 512, d) for d in (12, 24, 36)])
        self.bottleneck = ConvModule(2560, 512, 3, padding=1)
        self.c1_bottleneck = ConvModule(24, 48, 1)
        self.sep_bottleneck = nn.Sequential(DSConv(560, 512), DSConv(512, 512))
        self.conv_seg = nn.Conv2d(512, num_classes, 1)

    def forward(self, feats):
        c1, x = feats[0], feats[3]
        h, w = x.shape[2:]
        # Global pooling: mean over H,W (exports as ReduceMean) and broadcast.
        pool = self.image_pool[1](x.mean(dim=(2, 3), keepdim=True)).expand(-1, -1, h, w)
        y = torch.cat([pool] + [m(x) for m in self.aspp_modules], 1)
        y = self.bottleneck(y)
        c1 = self.c1_bottleneck(c1)
        y = F.interpolate(y, size=c1.shape[2:], mode='bilinear', align_corners=False)
        y = self.sep_bottleneck(torch.cat([y, c1], 1))
        return self.conv_seg(y)


class DeepLabV3Plus(nn.Module):
    """Input: RGB float 0..255 NCHW. Output: logits at 1/4 resolution."""

    def __init__(self, dilate_all_blocks=False):
        super().__init__()
        self.backbone = MobileNetV2(dilate_all_blocks)
        self.decode_head = Head()
        self.register_buffer('mean', torch.tensor(MEAN).view(1, 3, 1, 1))
        self.register_buffer('std', torch.tensor(STD).view(1, 3, 1, 1))

    def forward(self, x):
        x = (x - self.mean) / self.std
        return self.decode_head(self.backbone(x))


def load(path='dl/sd.pt', dilate_all_blocks=False):
    m = DeepLabV3Plus(dilate_all_blocks)
    sd = {k: v for k, v in torch.load(path).items() if not k.startswith('auxiliary_head')}
    missing, unexpected = m.load_state_dict(sd, strict=False)
    missing = [k for k in missing if k not in ('mean', 'std')]
    assert not missing and not unexpected, (missing, unexpected)
    return m.eval()
