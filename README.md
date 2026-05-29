# RailSem19: Semantic Segmentation and Digital Twin for Railway Infrastructure

Exploratory analysis and semantic segmentation of the [RailSem19 dataset](https://www.wilddash.cc/railsem19), containing 8,500 ego-perspective railway images with pixel-wise annotations across 19 classes. The project was developed as part of a larger initiative to create a **digital twin of rail infrastructure** for vegetation monitoring, obstacle detection, and maintenance planning. It applies a pre-trained HRNet model from TensorFlow Hub alongside LangSAM zero-shot segmentation to explore and evaluate railway scene understanding.

## Project Overview

This project investigates how well existing segmentation models generalize to railway-specific scenes, with a focus on **vegetation segmentation** as part of a digital twin concept for a fictional railroad operator ("United Railroad"). The pipeline performs inference on RailSem19 validation images and compares predictions against the dataset's ground-truth masks using IoU evaluation.

LangSAM (Language Segment Anything Model) was selected over HRNet after comparative evaluation, due to its text-image integration capability and superior precision for vegetation-specific segmentation on this dataset.

The project also designed a conceptual **OD (Object Detection) Signal Creation pipeline** that processes 10-frame batches from train-mounted cameras, performs vegetation pixel threshold analysis, integrates GPS data for geolocation, and generates structured alert signals with object type and severity classification.

## Key Results

| Metric | Value |
|---|---|
| Average IoU (vegetation segmentation) | 0.62 |
| Mean pixel accuracy | 54.42% |
| Median pixel accuracy | 51.70% |
| Avg. processing time per image | 1.69 min |
| Image resizing strategy | 1/2 original size |
| Train/test split | 80% / 20% |

LangSAM was chosen over HRNet for its streamlined approach offering more precision in the RailSem19 context, particularly for targeted class detection via text prompts.

## Key Features

- **Dataset Exploration:** Visualization of RailSem19 images, semantic label maps, and color-coded ground-truth overlays across all 19 classes.
- **Class Distribution Analysis:** Statistical analysis of class frequencies, image resolutions, and per-class pixel coverage across all 8,500 images.
- **HRNet Inference:** Semantic segmentation using a pre-trained HRNet-W48 (CamVid) model loaded from TensorFlow Hub, with full post-processing (argmax decoding, color mapping, overlay blending).
- **LangSAM Zero-Shot Segmentation:** Text-prompted segmentation (e.g., "vegetation") using the Segment Anything Model with language grounding, enabling flexible class-specific mask generation without retraining.
- **IoU Evaluation:** Intersection over Union computation to quantitatively compare predicted masks against ground-truth annotations.
- **Multi-Dataset Testing:** LangSAM inference tested on both the RailSem19 subset and a secondary vegetation dataset to assess generalization.
- **OD Signal Pipeline (concept):** Designed a deployment architecture for real-time obstacle detection using 10-frame batches, GPS data integration, and severity-based alert generation.

## RailSem19 Classes

The dataset contains 19 semantic classes: road, sidewalk, construction, tram-track, fence, pole, traffic-light, traffic-sign, vegetation, terrain, sky, human, rail-track, car, truck, trackbed, on-rails, rail-raised, and rail-embedded.

## Repository Structure

```
Railsem19/
|
├── EDA_images.ipynb                          # Main notebook: EDA, HRNet inference, visualization
├── langSAM_modeling.ipynb                    # LangSAM batch inference on RailSem19 subset with IoU
├── langSAM_modeling_second_dataset.ipynb     # LangSAM on secondary vegetation dataset
|
├── example-vis.py                            # Dataset visualization utility script
├── rs19-config.json                          # Class-to-color mapping configuration
├── calibration.txt                           # Camera calibration parameters
|
├── license.md                                # License information
├── license.txt                               # License (text format)
└── README.md
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

A pre-trained high-resolution network loaded from TensorFlow Hub (`google/HRNet/camvid-hrnetv2-w48/1`), originally trained on the CamVid dataset. Used here for general semantic segmentation inference on railway scenes. The model outputs per-pixel class probabilities which are decoded via argmax and mapped to color labels. Evaluated as a baseline but ultimately not selected for the final pipeline.

### LangSAM (Language Segment Anything)

An open-source project combining Meta's Segment Anything Model (SAM) with GroundingDINO for text-prompted instance segmentation. Selected as the primary model for vegetation segmentation due to its text-image synergy and adaptability to the RailSem19 dataset. The text prompt `"vegetation"` is used to generate vegetation masks, which are then compared against the RailSem19 ground-truth annotations (class 8) using IoU. Model parameters were customized to suit unique dataset characteristics including large image sizes and diverse scene conditions.

## Tech Stack

- **Frameworks:** TensorFlow 2.x, TensorFlow Hub, PyTorch (for SAM)
- **Segmentation Models:** HRNet-W48, LangSAM (SAM + GroundingDINO)
- **Libraries:** NumPy, OpenCV, Matplotlib, Pandas, PIL
- **Evaluation:** IoU (Intersection over Union), pixel accuracy
- **Project Management:** CRISP-DM, Agile/Scrum sprints, Jira

## License

This project is for academic and research purposes. The RailSem19 dataset is subject to its own licensing terms. See the [official dataset page](https://www.wilddash.cc/railsem19) for details.
