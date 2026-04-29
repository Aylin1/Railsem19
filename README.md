# RailSem19 — Semantic Segmentation & Exploration of Railway Scenes

Exploratory analysis and semantic segmentation of the [RailSem19 dataset](https://www.wilddash.cc/railsem19) — 8,500 ego-perspective railway images with pixel-wise annotations across 19 classes. The project combines a pre-trained HRNet model from TensorFlow Hub with LangSAM zero-shot segmentation to explore and evaluate railway scene understanding.

## Project Overview

This project investigates how well existing segmentation models generalize to railway-specific scenes. It performs inference on RailSem19 validation images and compares predictions against the dataset's ground-truth masks. It also explores prompt-based segmentation using LangSAM (Language Segment Anything Model) for targeted class detection such as vegetation.

The pipeline covers data exploration, class distribution analysis, model inference, mask post-processing, IoU evaluation, and visual comparison of predictions versus ground truth.

## Key Features

- **Dataset Exploration** — Visualization of RailSem19 images, semantic label maps, and color-coded ground-truth overlays across all 19 classes.
- **Class Distribution Analysis** — Statistical analysis of class frequencies, image resolutions, and per-class pixel coverage across all 8,500 images.
- **HRNet Inference** — Semantic segmentation using a pre-trained HRNet-W48 (CamVid) model loaded from TensorFlow Hub, with full post-processing (argmax decoding, color mapping, overlay blending).
- **LangSAM Zero-Shot Segmentation** — Text-prompted segmentation (e.g., "vegetation") using the Segment Anything Model with language grounding, enabling flexible class-specific mask generation.
- **IoU Evaluation** — Intersection over Union computation to quantitatively compare predicted masks against ground-truth annotations.
- **Multi-Dataset Testing** — LangSAM inference tested on both the RailSem19 subset and a secondary vegetation dataset to assess generalization.

## RailSem19 Classes

The dataset contains 19 semantic classes: road, sidewalk, construction, tram-track, fence, pole, traffic-light, traffic-sign, vegetation, terrain, sky, human, rail-track, car, truck, trackbed, on-rails, rail-raised, and rail-embedded.

## Repository Structure

```
Railsem19/
│
├── explore4.ipynb                        # Main notebook: EDA, HRNet inference,
│                                         # LangSAM segmentation, IoU evaluation
│
├── rs19-config.json                      # Class-to-color mapping configuration
├── requirements.txt                      # Python dependencies
└── README.md
```

### Notebooks Overview

The project originally consisted of several notebooks during development. These have been consolidated into a single clean notebook:

| Original Notebook | Content | Status |
|---|---|---|
| `explore4.ipynb` | Full EDA + HRNet inference + LangSAM + class stats | **Kept** (main notebook) |
| `langSAM_modeling.ipynb` | LangSAM batch inference on RailSem19 subset with IoU | Merged into `explore4` |
| `langSAM_modeling__1_.ipynb` | Near-duplicate of `langSAM_modeling.ipynb` | Removed (redundant) |
| `langSAM_modeling_second_dataset.ipynb` | LangSAM on secondary vegetation dataset | Merged into `explore4` |

## Getting Started

### Prerequisites

- Python 3.9+
- TensorFlow 2.x
- Internet connection (for downloading the HRNet model from TensorFlow Hub)

### Installation

```bash
git clone https://github.com/Aylin1/Railsem19.git
cd Railsem19
pip install -r requirements.txt
```

### Dependencies

```
tensorflow
tensorflow-hub
torch
torchvision
segment-geospatial
groundingdino-py
leafmap
localtileserver
opencv-python
matplotlib
numpy
pandas
Pillow
```

### Dataset

The RailSem19 dataset is not included in this repository. You can request access from the official source: [wilddash.cc/railsem19](https://www.wilddash.cc/railsem19)

Once downloaded, place the data so the folder structure looks like:

```
jpgs/rs19_val/         # 8,500 JPEG images
uint8/rs19_val/        # Corresponding semantic label maps (grayscale)
rs19-config.json       # Class configuration (included in repo)
```

## Models Used

### HRNet-W48 (TensorFlow Hub)

A pre-trained high-resolution network loaded from TensorFlow Hub (`google/HRNet/camvid-hrnetv2-w48/1`), originally trained on the CamVid dataset. Used here for general semantic segmentation inference on railway scenes. The model outputs per-pixel class probabilities which are decoded via argmax and mapped to color labels.

### LangSAM (Language Segment Anything)

An open-source project combining Meta's Segment Anything Model (SAM) with GroundingDINO for text-prompted instance segmentation. In this project, the text prompt `"vegetation"` is used to generate vegetation masks, which are then compared against the RailSem19 ground-truth annotations (class 8) using IoU.

## Tech Stack

- **Framework:** TensorFlow 2.x, TensorFlow Hub, PyTorch (for SAM)
- **Segmentation Models:** HRNet-W48, LangSAM (SAM + GroundingDINO)
- **Libraries:** NumPy, OpenCV, Matplotlib, Pandas, PIL
- **Evaluation:** IoU (Intersection over Union)

## License

This project is for academic and research purposes. The RailSem19 dataset is subject to its own licensing terms.
