# RoboMaster Project

This repository contains the RoboMaster SDK along with assignments and development scripts for DJI RoboMaster EP.

## Structure

- **`assignment1/`**: Autonomous maze navigation, sensor fusion, manipulator control, and path plotting for Assignment 1.
- **`RoboMaster-SDK/`**: DJI RoboMaster Python SDK source, examples, and custom instructor/lab scripts.

## Getting Started

### Prerequisites
- Python 3.8+
- DJI RoboMaster EP

### Installation
```bash
# Setup virtual environment
python3 -m venv venv
source venv/bin/activate

# Install SDK requirements
pip install -r RoboMaster-SDK/requirements.txt
```

### Running Assignment 1
```bash
cd assignment1

# Dry run / simulation
python main.py --dry

# Real run
python main.py
```
