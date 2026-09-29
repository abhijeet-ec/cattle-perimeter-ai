# Cattle Detection Model — Development and Final V3 Result

The cattle detection model was developed progressively by testing it against increasingly difficult real-world conditions and modifying the dataset based on the problems observed at each stage.

## 1. Initial Cattle Model — Non-Greenery Dataset

The initial model was developed using cattle images without specifically accounting for complex greenery and farm-background conditions.

The first stage established the basic ability of the model to detect cattle using the FOMO object-detection approach.

Testing then showed that the model needed to be exposed to more realistic farm environments.

## 2. Greenery and Background Data Added

Since the actual deployment environment contains grass, plants, leaves, trees, and other vegetation, greenery and background images were added to the dataset.

A total of **350 greenery/background negative images** were added.

The model was then retrained so that normal farm vegetation and background patterns were represented during training.

This stage was intended to improve the separation between actual cattle and the surrounding farm environment.

## 3. White Cattle Added

The next major problem identified was the detection of white or light-coloured cattle.

White cattle can be difficult to distinguish from bright or low-contrast backgrounds, so additional white-cattle examples were added to the dataset.

The model was retrained with these additional examples.

The documented V2 evaluation of the improved model was recorded in the independent test report that was added to the project repository.

### Test Configuration

- **Model:** `one with white cow.eim`
- **Test images:** 50
- **Cattle images:** 30
- **Non-cattle images:** 20
- **Input:** 160 × 160 RGB
- **Model:** FOMO MobileNetV2 0.35
- **Training cycles:** 60
- **Learning rate:** 0.001
- **Training:** CPU
- **Data augmentation:** Enabled
- **Quantization:** INT8
- **Evaluation threshold:** 0.50

### Test Results

| Metric | Result |
|---|---:|
| Accuracy | **94.00%** |
| Precision | **96.55%** |
| Recall | **93.33%** |
| F1 Score | **94.92%** |
| False Positive Rate | **5.00%** |
| Average Confidence | **0.4775** |
| Average Latency | **2.061 ms** |

### Confusion Matrix

| Actual | Predicted Cattle | Predicted Non-Cattle |
|---|---:|---:|
| Cattle | **28** | **2** |
| Non-cattle | **1** | **19** |

The test therefore produced **28 true positives, 19 true negatives, 1 false positive, and 2 false negatives**.

The detailed test report also recorded individual sample results. For example, `cattle_001.jpg` was detected with **94.92% confidence**, while `cattle_020.jpg` was recorded as a false negative with no detected cattle box.

This stage established that the model could detect cattle across a more varied visual dataset, including the previously difficult white-cattle cases. The complete test report was included with the project documentation on GitHub.

## 4. Cat and Dog Testing

After the cattle and background improvements, cats and dogs were introduced into the testing process.

The purpose was to observe how the model behaved when presented with animals other than cattle.

This revealed another important dataset consideration: the cat and dog samples were subsequently incorporated into the dataset under the **cattle label** for the next experiment.

## 5. Dataset Modified With Cat and Dog Samples

The dataset was then changed so that the cat and dog images were also labelled as **cattle**.

The modified dataset was used to retrain the model, followed by another accuracy test.

The important result of this stage was that the model was evaluated again **after the cat and dog data-sheet/label modification**, rather than using the earlier white-cattle result as the final result.

The exact numerical accuracy, precision, recall, and F1 values from this final cat-and-dog-modified dataset test are not present in the available project record, so they are not reproduced here.

## 6. Final V3 Model — 160 × 160

The final model was deployed on the **AI-Thinker ESP32-CAM** using Edge Impulse FOMO.

The model uses a **160 × 160 input resolution** for on-device inference.

The final embedded configuration included:

- AI-Thinker ESP32-CAM
- Edge Impulse FOMO
- 160 × 160 model input
- Cattle confidence threshold of **90%**
- On-device inference
- Bounding-box detection
- Serial detection telemetry

A verified final V3 runtime detection produced:

```text
Cattle confidence: 92.97%

Bounding box:
x = 40
y = 88
w = 32
h = 32

Capture + preprocessing: 186 ms
FOMO inference: 2428 ms
Total detection latency: 2648 ms

Free PSRAM: 3,923,252 bytes
```


## 7. Independent V3 Evaluation — Final Animal Intrusion Model

The final V3 model was evaluated separately after the dataset was modified to include cattle, cats, and dogs as the positive **animal intrusion** class.

This evaluation was documented in the **Independent V3 Evaluation Report**.

### V3 Test Configuration

| Parameter | Result |
|---|---|
| Model | `with cats and cow.eim` |
| Test images | **50** |
| Test set | **30 positive + 20 negative** |
| Input resolution | **160 × 160 RGB** |
| Positive class | **Cattle + dogs + cats = animal intrusion** |
| Decision threshold | **0.50** |

### V3 Evaluation Results

| Metric | Result |
|---|---:|
| True Positive | **29** |
| True Negative | **19** |
| False Positive | **1** |
| False Negative | **1** |
| Accuracy | **96.00%** |
| Precision | **96.67%** |
| Recall | **96.67%** |
| F1 Score | **96.67%** |
| False Positive Rate | **5.00%** |
| Average Confidence | **0.8305** |
| Average Latency | **6.10 ms** |

### Confusion Matrix

| Actual | Predicted Intrusion | Predicted Non-Intrusion |
|---|---:|---:|
| Actual Intrusion | **29 (TP)** | **1 (FN)** |
| Actual Non-Intrusion | **1 (FP)** | **19 (TN)** |

This final V3 evaluation showed **96.00% accuracy** on the 50-image independent test set, with 29 correctly detected animal-intrusion samples and 19 correctly rejected non-intrusion samples.

The evaluation represents the final dataset configuration in which **cattle, dogs, and cats were treated as the positive animal-intrusion class**.


## Final Development Progression

```text
Initial cattle dataset
        ↓
Greenery/background data added
        ↓
White-cattle examples added
        ↓
Cat and dog testing
        ↓
Cat + dog samples included in the animal-intrusion dataset
        ↓
Final dataset retraining
        ↓
Independent V3 evaluation
        ↓
96.00% accuracy
        ↓
V3 embedded deployment
        ↓
160 × 160 on-device FOMO detection
        ↓
92.97% cattle confidence
```

The final V3 stage represents the transition from dataset experimentation to an actual **160 × 160 embedded cattle-detection system running on the ESP32-CAM**.
