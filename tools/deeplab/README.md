# DeepLab (ADE20K) → QNN conversion

How `android/app/src/main/assets/qnn/deeplab_ade_int8_v{73,81}.bin` were made: DeepLabV3+
(MobileNetV2, output stride 8) trained on ADE20K (150 classes), exported to ONNX with the
class arg-max inside the graph, quantized to INT8 and compiled to QNN HTP context binaries
for the Hexagon v81 (SM8850) and v73 (SM6650) NPUs.

Shipped models: input **288 × 384** RGB (0–255), output a **72 × 96** int32 class map.
They are byte-identical to `qnn/out/deeplab_int8_288x384_v{81,73}.bin` from step 4.

## Files

| File | Purpose |
|---|---|
| `model.py` | Pure-PyTorch DeepLabV3+ matching the MMSegmentation v0.x checkpoint; `load()` reads `dl/sd.pt` |
| `ade_meta.json` | ADE20K class names (the app maps them to navigation classes in `nav_classes.h`) |
| `scripts/fetch_ade.py` | Fetch N ADE20K validation image / annotation pairs (HTTP range requests, no full download) |
| `scripts/export.py` | Export ONNX at a given size + write INT8 calibration inputs |
| `scripts/build_qnn.sh` | ONNX → QNN INT8 model → context binaries for v81 and v73 |
| `scripts/eval.py` | fp32 mIoU at several input sizes |
| `scripts/quick_eval.py` | INT8 phone output vs fp32 on one image; fp32 mIoU on held-out images |

## Steps

Run everything from this directory (`tools/deeplab`). Large files (checkpoint, ONNX, images,
outputs) are not in git.

1. **Checkpoint** — download the MMSegmentation checkpoint
   `deeplabv3plus_m-v2-d8_512x512_160k_ade20k_20200825_223255-465a01d4.pth` (from the
   MMSegmentation model zoo, DeepLabV3+ MobileNetV2 ADE20K) into `dl/`, and save its weights as
   `dl/sd.pt`:

   ```bash
   python3 -c "import torch; c=torch.load('dl/deeplabv3plus_m-v2-d8_512x512_160k_ade20k_20200825_223255-465a01d4.pth', map_location='cpu'); torch.save(c.get('state_dict', c), 'dl/sd.pt')"
   ```

2. **Calibration / test images** — 100 ADE20K validation pairs (even = calibration, odd = test):

   ```bash
   mkdir -p ade_val && python3 scripts/fetch_ade.py 100 ade_val
   ```

3. **(optional) accuracy at different sizes**: `python3 scripts/eval.py`

4. **Export + convert + compile** (QNN / QAIRT SDK 2.47; set `QNN_SDK_ROOT` if it is not at
   the default path):

   ```bash
   QNN_SDK_ROOT=/path/to/qairt/2.47.0.260601 scripts/build_qnn.sh 288 384
   ```

   → `qnn/out/deeplab_int8_288x384_v81.bin` and `..._v73.bin`

5. **Check INT8 vs fp32** on the phone output of one calibration image (optional):
   `python3 scripts/quick_eval.py <phone_output.raw>`

6. **Install in the app**:

   ```bash
   cp qnn/out/deeplab_int8_288x384_v81.bin ../../android/app/src/main/assets/qnn/deeplab_ade_int8_v81.bin
   cp qnn/out/deeplab_int8_288x384_v73.bin ../../android/app/src/main/assets/qnn/deeplab_ade_int8_v73.bin
   ```

   (stored in Git LFS — see the main README). `semseg.cpp` expects the 288 × 384 input and the
   72 × 96 class output.

## Requirements

Python 3 with `torch`, `numpy`, `Pillow`, `onnx`; Qualcomm QNN / QAIRT SDK 2.47
(`qnn-onnx-converter`, `qnn-model-lib-generator`, `qnn-context-binary-generator`).

## Licences

The checkpoint (MMSegmentation, Apache-2.0) and the ADE20K dataset (MIT CSAIL, its own terms of
use) carry their own licences; check them before distributing models built from them.
