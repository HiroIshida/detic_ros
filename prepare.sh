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
    # SwinB model
    wget https://dl.fbaipublicfiles.com/detic/Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.pth -O models/Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.pth
    # ConvNet model
    wget https://dl.fbaipublicfiles.com/detic/Detic_LCOCOI21k_CLIP_CXT21k_640b32_4x_ft4x_max-size.pth -O models/Detic_LCOCOI21k_CLIP_CXT21k_640b32_4x_ft4x_max-size.pth
    # Res18 model
    wget https://dl.fbaipublicfiles.com/detic/Detic_LCOCOI21k_CLIP_R18_640b32_4x_ft4x_max-size.pth -O models/Detic_LCOCOI21k_CLIP_R18_640b32_4x_ft4x_max-size.pth
fi

pip3 install -r requirements.txt
