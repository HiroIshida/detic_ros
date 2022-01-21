#!/usr/bin/bash

ln -sf Detic/configs detic_configs 

cd node_script
ln -sf ../Detic/detic
ln -sf ../Detic/third_party
cd ..

if [ ! -d "models" ]; then
    mkdir models
    wget https://dl.fbaipublicfiles.com/detic/Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.pth -O models/Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.pth
fi
