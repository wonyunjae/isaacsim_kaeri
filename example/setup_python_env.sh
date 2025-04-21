#!/bin/bash

ISAAC_DIR="/isaac-sim"
PACKAGE_DIR="/isaac-sim/isaac_sim"

# built-in
# 1) 파이썬 참조 세팅
export PYTHONPATH=$PYTHONPATH:$ISAAC_DIR/../../../$PYTHONPATH:$ISAAC_DIR/exts/omni.isaac.kit:$ISAAC_DIR/exts/omni.isaac.gym:$ISAAC_DIR/kit/kernel/py:$ISAAC_DIR/kit/plugins/bindings-python:$ISAAC_DIR/exts/omni.isaac.lula/pip_prebundle:$ISAAC_DIR/exts/omni.exporter.urdf/pip_prebundle:$ISAAC_DIR/kit/exts/omni.kit.pip_archive/pip_prebundle:$ISAAC_DIR/exts/omni.isaac.core_archive/pip_prebundle:$ISAAC_DIR/exts/omni.isaac.ml_archive/pip_prebundle:$ISAAC_DIR/exts/omni.pip.compute/pip_prebundle:$ISAAC_DIR/exts/omni.pip.cloud/pip_prebundle:$ISAAC_DIR/extscache/omni.pip.torch-2_0_1-2.0.2+105.1.lx64/torch-2-0-1:$ISAAC_DIR/extsPhysics/omni.physics.tensors-105.1.9-5.1
# 2) 라이브러리 참조 세팅
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ISAAC_DIR/../../../$LD_LIBRARY_PATH:$ISAAC_DIR/.:$ISAAC_DIR/exts/omni.usd.schema.isaac/plugins/IsaacSensorSchema/lib:$ISAAC_DIR/exts/omni.usd.schema.isaac/plugins/RangeSensorSchema/lib:$ISAAC_DIR/exts/omni.isaac.lula/pip_prebundle:$ISAAC_DIR/exts/omni.exporter.urdf/pip_prebundle:$ISAAC_DIR/kit:$ISAAC_DIR/kit/kernel/plugins:$ISAAC_DIR/kit/libs/iray:$ISAAC_DIR/kit/plugins:$ISAAC_DIR/kit/plugins/bindings-python:$ISAAC_DIR/kit/plugins/carb_gfx:$ISAAC_DIR/kit/plugins/rtx:$ISAAC_DIR/kit/plugins/gpu.foundation:$ISAAC_DIR/extsPhysics/omni.physics.tensors-105.1.9-5.1/bin
