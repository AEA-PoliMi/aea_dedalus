# Lane · U-Net

**Owner:** Jamileth

`dedalus_unet_corsia_v2.ipynb` (Kaggle) trains a U-Net that says, pixel by pixel, "lane marking" or not.
Labels come for free from the CARLA segmentation (`RoadLine` = 24).

- Based on AN2DL **lab 9** (Semantic Segmentation) and **lab 7** (Transfer Learning).
- Three models to compare with `MODELLO`: `"lab"` (lab 9 U-Net), `"res_unet"` (residual U-Net, 4 levels),
  `"pre_allenato"` (ResNet-18 ImageNet encoder).
- Loss: weighted cross-entropy + Dice (optional clDice for thin structures).
- Output: a 0/1 mask at 960×540, i.e. the same output as the colour-filter step of `../baseline/`.
  The rest of the baseline pipeline (BEV → sliding window → RANSAC) stays the same.
- Exports the model to ONNX and measures ms per frame.

Status: notebook ready and tested on synthetic data; first runs on CARLA sessions to do.
