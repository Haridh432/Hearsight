#!/bin/bash
# Usage: scripts/build_qnn.sh H W   -> qnn/out/deeplab_int8_HxW_{v81,v73}.bin
set -e
mkdir -p qnn/out
cd "$(dirname "$0")/.."
T=${1}x${2}; N=deeplab_int8_$T
export QNN_SDK_ROOT=${QNN_SDK_ROOT:-/home/naveen/Downloads/qairt/2.47.0.260601}
source $QNN_SDK_ROOT/bin/envsetup.sh >/dev/null 2>&1
python3 scripts/export.py $1 $2 2>/dev/null
qnn-onnx-converter --input_network deeplab_ade_$T.onnx --input_list calib_$T/list.txt --act_bitwidth 8 \
  --weights_bitwidth 8 --bias_bitwidth 32 --out_node classes -o qnn/$N.cpp > qnn/convert_$T.log 2>&1
qnn-model-lib-generator -c qnn/$N.cpp -b qnn/$N.bin -o qnn/libs -t x86_64-linux-clang > qnn/libgen_$T.log 2>&1
for arch in v81 v73; do
  soc=""; [ $arch = v73 ] && soc=', "soc_model": 74'
  echo "{ \"graphs\": [ { \"graph_names\": [\"$N\"], \"vtcm_mb\": 0, \"O\": 3 } ], \"devices\": [ { \"dsp_arch\": \"$arch\"$soc } ] }" > qnn/htp_ext_${T}_$arch.json
  echo "{ \"backend_extensions\": { \"shared_library_path\": \"$QNN_SDK_ROOT/lib/x86_64-linux-clang/libQnnHtpNetRunExtensions.so\", \"config_file_path\": \"$PWD/qnn/htp_ext_${T}_$arch.json\" } }" > qnn/htp_cfg_${T}_$arch.json
  qnn-context-binary-generator --backend $QNN_SDK_ROOT/lib/x86_64-linux-clang/libQnnHtp.so \
    --model qnn/libs/x86_64-linux-clang/lib$N.so --config_file qnn/htp_cfg_${T}_$arch.json \
    --binary_file ${N}_$arch --output_dir qnn/out > qnn/ctx_${T}_$arch.log 2>&1
done
grep -m1 "Total MACs" qnn/convert_$T.log
ls -la qnn/out/${N}_*
