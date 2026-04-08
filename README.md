# Chladni Klangfiguren – STM32 Signal Generator

## Overview
This project uses an **STM32 Nucleo-L476RG** development board to generate a sinusoidal excitation signal for a **Chladni plate experiment**. By adjusting the excitation frequency, standing wave patterns can appear on a metal plate covered with fine salt or sand.

The signal is generated with the MCU DAC and can be controlled in real time with a **rotary encoder** and a **push button**. The selected frequency is shown on an **OLED display** and can also be sent over **UART** for monitoring.

## Goal
The purpose of the project is to excite a plate mechanically so that **Chladni figures** become visible. Different frequencies excite different vibration modes and therefore lead to different particle patterns on the plate.

## Important Note
Before starting, check the **firmware of the development board / ST-LINK**.

During development, communication problems were caused by:
- outdated board / debugger firmware
- a baud-rate mismatch

After updating the firmware and correcting the baud rate, UART communication worked reliably again.

## Hardware
- STM32 Nucleo-L476RG
- Rotary encoder
- Push button
- OLED display via I2C
- UART output for monitoring
- External amplifier / actuator / shaker
- Metal plate with salt or sand

## Main Software Functions

### Sine generation via DAC
The microcontroller generates a sinusoidal output signal using the internal DAC.

### Frequency control via rotary encoder
The output frequency can be increased or decreased during runtime with the rotary encoder.

### Button-controlled output enable
A push button toggles the signal output on and off.

### Controlled startup and shutdown
The output amplitude is ramped up and down to avoid abrupt signal jumps.

### OLED display output
The currently selected frequency is shown on the display.

### UART monitoring
Frequency and debug values can be transmitted over UART, for example for visualization with Teleplot.

## Typical Pin Usage
- `PA4` -> DAC output
- `PC6 / PC7` -> rotary encoder inputs
- `PB3` -> button input
- `PA5` -> onboard LED
- `PA2 / PA3` -> USART2 TX/RX

## Practical Note on Salt/Sand Motion
In the practical demonstrator, visible salt/sand motion depends strongly on the mechanical vibration amplitude of the plate. Therefore, the drive amplitude may need to be increased at higher frequencies to maintain sufficiently visible motion.

## Troubleshooting

### UART communication not working reliably
- Check ST-LINK / board firmware updates
- Verify the baud rate on both MCU and PC side
- Reflash the current firmware if needed

### Encoder behaves noisily
- Check pull-ups, filtering, and debounce
- Keep wiring short
- Verify both encoder channels

### No visible Chladni figures
- Check mechanical coupling between actuator and plate
- Try a different frequency range
- Increase amplitude carefully
- Use fine salt or fine sand

## Summary
This project is a compact microcontroller-based excitation source for **Chladni sound figure experiments**. It combines DAC-based sine generation, real-time frequency control, user interaction via encoder and button, display output, and serial monitoring.
