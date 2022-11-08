#!/usr/bin/bash

git submodule update --init --recursive 

ln -sf Detic/configs detic_configs 
ln -sf Detic/datasets datasets

cd node_script
ln -sf ../Detic/detic
ln -sf ../Detic/third_party
cd ..

if [ ! -d "models" ]; then
    mkdir models
    # All real-time models
    # https://github.com/facebookresearch/Detic/blob/main/docs/MODEL_ZOO.md#real-time-models

    # Swin Transformer model
    wget -q https://dl.fbaipublicfiles.com/detic/Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.pth -O models/Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.pth
    # ConvNet model
    wget -q https://dl.fbaipublicfiles.com/detic/Detic_LCOCOI21k_CLIP_CXT21k_640b32_4x_ft4x_max-size.pth -O models/Detic_LCOCOI21k_CLIP_CXT21k_640b32_4x_ft4x_max-size.pth
    # Res50 model
    wget -q https://dl.fbaipublicfiles.com/detic/Detic_LCOCOI21k_CLIP_R5021k_640b32_4x_ft4x_max-size.pth -O models/Detic_LCOCOI21k_CLIP_R5021k_640b32_4x_ft4x_max-size.pth
    # Res18 model
    wget -q https://dl.fbaipublicfiles.com/detic/Detic_LCOCOI21k_CLIP_R18_640b32_4x_ft4x_max-size.pth -O models/Detic_LCOCOI21k_CLIP_R18_640b32_4x_ft4x_max-size.pth
fi

pip3 install -r requirements.txt
