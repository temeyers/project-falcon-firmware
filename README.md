# Project Falcon Firmware

Official production firmware builds for **Project Falcon** pucks, the washer and dryer sensors that send tamper, theft and cycle-done alerts.

## Status
**No releases yet.** Version 1 (V1) is coming soon.

## How updates work
Production pucks check this repository's [Releases](https://github.com/temeyers/project-falcon-firmware/releases) for new firmware and install updates over the air (OTA). Every build published here will be signed, and pucks will verify the signature before installing. Unsigned or altered files will be rejected.

## What will be here
- Signed firmware images, one GitHub Release per version
- Release notes and checksums for each version

This repository contains only finished builds. Source code is kept in a separate private repository.

## Reporting a problem
If you find a security issue with Project Falcon firmware, please open an issue here without including sensitive details, and we'll follow up privately.
