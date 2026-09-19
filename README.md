# TILT

> Two-legged Iterative Locomotion Testbed

A compact 6-DoF biped robot platform for iterative locomotion experiments.

<p align="center">
  <img src="docs/image/TILT%20cad%20and%20real.png" alt="TILT MK1 CAD model" width="720">
</p>

## Overview

TILT는 소형 이족보행 로봇의 기구 설계, 서보 제어, 보행 알고리즘을 반복적으로 실험하기 위한 로봇 프로젝트입니다.

Apple Developer Academy Challenge 6의 Physical AI Pet 프로젝트에서 보행 가능성을 검증하기 위해 시작했지만, 특정 캐릭터에 종속되지 않는 독립적인 로봇 플랫폼으로 발전시키는 것을 목표로 합니다.

## Goals

- ESP32-S3 기반 실시간 로봇 제어
- STS3215 버스 서보를 이용한 6자유도 하체 구현
- 모델 기반 관절 궤적과 보행 시퀀스 개발
- 평평한 실내 바닥에서 전진, 정지, 재출발 및 완만한 회전
- CAD와 펌웨어를 함께 관리하는 재현 가능한 로봇 프로젝트 구축
- 향후 센서 피드백과 균형 제어를 추가할 수 있는 구조 마련

## Hardware

| Component | Specification |
| --- | --- |
| Controller | ESP32-S3 |
| Actuator | 6 × Feetech STS3215-C001 |
| Servo interface | Waveshare Bus Servo Adapter (A) |
| Degrees of freedom | 6 DoF, 3 per leg |
| Communication | UART, BLE |
| Current prototype | TILT-P0 |

## Repository Structure

```text
TILT/
├── CAD/          # Mechanical source files and manufacturing exports
├── firmware/     # ESP32-S3 firmware
├── hardware/     # BOM, wiring, pinout and power documentation
├── tools/        # Servo setup and gait development utilities
└── docs/         # Architecture, assembly, calibration and experiment notes
```
