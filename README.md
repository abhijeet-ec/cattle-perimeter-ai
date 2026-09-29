# 🐄 Cattle Perimeter AI

An ESP32-CAM based perimeter monitoring and cattle detection project.

## ⚡ Overview

This project explores real-time detection using an **AI-Thinker ESP32-CAM** and an Edge Impulse FOMO model.

The system is designed around a camera-based monitoring setup for detecting cattle around a defined perimeter.

## 🔧 Hardware

- AI-Thinker ESP32-CAM
- OV2640 camera
- Status LED
- Wi-Fi

## 💻 Software

- Arduino IDE
- Embedded C/C++
- Edge Impulse
- ESP32 Arduino framework

## 🧠 Detection

The project uses **FOMO (Faster Objects, More Objects)** for lightweight object detection on the ESP32-CAM.

The inference pipeline is designed for embedded hardware with limited resources.

## 📡 System Concept

```text
          ┌──────────────┐
          │  OV2640      │
          │   Camera     │
          └──────┬───────┘
                 │
                 ▼
        ┌─────────────────┐
        │    ESP32-CAM     │
        │                  │
        │  Image Capture   │
        │       ↓          │
        │  FOMO Inference  │
        │       ↓          │
        │ Detection Result │
        └────────┬─────────┘
                 │
                 ▼
          ┌─────────────┐
          │   Indicator │
          │     LED     │
          └─────────────┘
