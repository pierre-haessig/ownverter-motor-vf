# OwnVerter application: V/f motor control inverter

This repository host embedded microcontroller code for using the [OwnTech OwnVerter](https://www.owntech.io/ownverter/) board as an **motor drive**.
The inverter is driven as a voltage source:
- open loop (no position sensor feedback)
- with V/f control, i..e voltage amplitude proportional to frequency

   - with the option to adjust a small voltage delta.

This code is used in the context of an electric machines course at CentralSupélec, Rennes campus.

## Experiment schematics

Wiring diagram: TO BE UPDATED

![ownverter_wiring_inverter_load](images/ownverter_wiring_inverter_load.png)

## Usage

This code derives from the [OwnTech Power API Core repository](https://github.com/owntech-foundation/Core), and more specifically from the [ownverter-islanded](https://github.com/pierre-haessig/ownverter-islanded) example. It is designed to be used with VS Code and PlatformIO. The usage of this type of repository is documented at https://docs.owntech.org (e.g. Getting Started section).

