# SAGE — Edge AI Biomedical System for Real-Time Physiological Risk Monitoring

**SIH26181 — Qualcomm Inc. | MedTech / BioTech / HealthTech | Hardware**

A secure, AI-powered personal health companion that delivers real-time, privacy-preserving health monitoring and early warning — entirely on-device, with no cloud dependency.

---

## Table of Contents
- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Sensor Interfacing and AFE](#sensor-interfacing-and-afe)
- [VLSI Design](#vlsi-design)
- [Power Electronics](#power-electronics)
- [Embedded Systems + ML](#embedded-systems--ml)
- [Setup & Build](#setup--build)
- [Limitations](#limitations)

---

## Overview

Existing wearable health monitors share three problems: they run at constant computational effort regardless of urgency, they depend on cloud connectivity that fails exactly when disasters make it most unreliable, and they treat every reading with equal weight — missing early, subtle risk trends.

---

## System Architecture

| Stage | What happens |
|---|---|
| Sensing | Four physiological/environmental channels sampled continuously |
| FPGA triage | Three-regime classification (Stable / Risk / Critical) computed in dedicated hardware |
| Wake decision | FPGA raises `wake_flag` only for Risk/Critical — ESP32-S3 ML sleeps otherwise |
| Classification | 12-tree Random Forest model runs only when woken, fusing FPGA output with additional sensor context |
| Power delivery | Gating, dual-rail isolation, and voltage scaling respond to the same regime signal |

---

## Sensor Interfacing and AFE

**Role:** Sensor front-end — acquiring every physiological and environmental signal the rest of the system depends on.

- **MAX30100** — heart rate and SpO2 via photoplethysmography
- **Dual DHT22** — one on-skin, one ambient, providing a stress-relevant differential (temperature + humidity swing between body and environment)
- **Thermistor** — body temperature, analog front end into the ESP32-S3's onboard ADC
- **MQ2** — air quality / toxic gas sensing for pollution-event resilience

Every downstream computation — FPGA triage, ML classification, power scaling — depends entirely on the signal quality this layer delivers. Noisy or miscalibrated sensing here silently corrupts every regime decision made afterward.

**Analog Front-End Design**

Only the thermistor and MQ2 are true analog signals; MAX30100 and dual DHT22 digitize internally and connect over I²C/GPIO directly.

Both analog channels share an active 2nd-order Sallen-Key low-pass filter (LM358, R = 10kΩ, C_feedback = 2µF, C_ground = 1µF), cutoff ~11.25Hz — passing real sensor signal while rejecting ~95% of 50Hz mains noise. Verified in LTspice (simulated cutoff 10.75Hz, clean Butterworth roll-off, no overshoot). Both channels digitize via the ESP32-S3's onboard ADC1, 12-bit shifted to 8-bit for the FPGA's RTL.

MQ2's heater — the stack's most power-hungry sensor — is regime-gated via an N-channel MOSFET: duty-cycled in Stable regime, fully powered in Risk/Critical. A 1N4007 snubber diode clamps the switch-off spike to ~5.7V (LTspice-verified), extending the system's power-adaptive philosophy down into the sensing layer.

---

## VLSI Design

**Role:** A dedicated, always-on hardware watchdog — the reason this system doesn't need its MCU awake constantly.

- **Target device:** ForgeFPGA SLG47910V, ~30% LUT utilization, well within its 1120-LUT capacity
- **Three-regime triage:**
  - *Stable* — successive BPM difference fused with stress/temp/gas via add + shift only
  - *Risk* — 4-sample windowed BPM average, smoother estimate, still no multiplication
  - *Critical* — reduced-iteration CORDIC magnitude estimate, chosen specifically because this FPGA has no dedicated multiplier hardware, making shift-add the most LUT-efficient way to combine two signals
- **`wake_flag` output** — the FPGA never classifies anything itself; it only decides whether the situation is worth waking the ML model for
- Internal power-on reset generator and internal-oscillator clocking — no external reset/clock pins wasted

The FPGA's entire justification: continuous computation is cheaper in dedicated hardware than in a general-purpose CPU, and its negligible standby cost is what lets the ML classifier sleep by default.

---

## Power Electronics

**Role:** Physiologically-driven dynamic power routing — power delivery that scales with the same urgency signal driving the VLSI layer.

- **Hardware power-gating** — PMOS load switches cut power to the FPGA's CORDIC block entirely during Stable regime, dropping standby leakage to zero
- **Dual-rail noise isolation** — a dedicated ultra-low-noise LDO powers the bioelectronic sensors on a rail isolated from digital switching noise, protecting signal fidelity
- **Dynamic voltage scaling** — an I²C/SPI-programmable buck regulator drops FPGA supply to ~0.9V when stable, raises it to ~1.2V the instant Critical regime is entered

This is what makes the system's power adaptivity compound rather than sit at a single layer — computation, power delivery, and ML wake state all scale together.

---

## Embedded Systems + ML

**Role:** On-device classification and system orchestration — sleeps by default, wakes only when the FPGA says so.

- **ESP32-S3** — dual-core compute, onboard ADC, boots the FPGA bitstream over SPI at startup
- **Serial link to FPGA** — 32-bit packetized sensor data sent over a 2-wire interface (data + clock)
- **12-tree Random Forest classifier** — takes 5 fused inputs (HR, SpO2, temperature, gas, humidity), runs *only* when `wake_flag` is high
- **Personal baseline calibration** — a short on-boot learning phase establishes resting HR/temperature statistics per user
- **5-rule predictive heuristics** — cheap, always-running checks (heatstroke onset, pollution exposure, hypoxia, cardiac stress surge, dehydration risk) that fire independently of the wake-gated ML model
- **Local alerting** — LED/buzzer feedback per regime; no raw physiological data ever leaves the device

---

## Setup & Build

**FPGA side:**
1. Open the `.v` sources in Renesas Go Configure Software Hub, target **SLG47910V**
2. Assign I/O per the pin table in `docs/` (2 serial input pins, 9 output pins, internal oscillator for clock)
3. Synthesize, run the Rule Checker, generate bitstream

**Firmware side:**
1. Wire sensors per the pin definitions at the top of the `.ino`
2. Install libraries: `DHT`, `MAX30100_PulseOximeter`
3. Flash via Arduino IDE (ESP32-S3 board package)
4. Stand still for the calibration phase before expecting regime/alert output

---

## Limitations

- MQ2 requires a proper Rs/Ro calibration curve for true PPM output — current gas values are a linear approximation
- Regime thresholds (60/120) are placeholders pending calibration against real multi-sensor fused ranges
- CORDIC's magnitude output has no gain compensation — acceptable for prototype risk scoring, not a calibrated clinical measurement
- No flood-resilience sensing implemented — the PS's other two disaster scenarios (heat waves, pollution) are addressed; floods are not yet
- Classifier accuracy is bounded by the size and diversity of available training data.

**Authors**

## **Karthikeyan D** (Team Lead and ML Developer)
## **Tharun S** (Sensor Interfacing and Analog Front-end Design)
## **Pranav J** (VLSI Architect and FPGA interfacing)
## **Satvika Gobi** (Power Electronics Designer)
## **Adithya KS** (Embedded Systems and Integration)
## **Shreenithi A** (Embedded Systems and Power Regulation)
SAGE addresses all three with a **regime-adaptive, edge-autonomous architecture**: a dedicated FPGA watchdog continuously evaluates incoming vitals at near-zero power cost, and wakes the power-hungry ML classifier only when a reading genuinely warrants it. Four engineering domains contribute equally load-bearing pieces — remove any one and the system stops working, not just degrades.
