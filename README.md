# STM32WL LoRa Firmware

This repository contains two STM32CubeIDE firmware projects for a LoRa/SubGHz
link using STM32WL boards:

- `TX/`: transmitter firmware.
- `RX/`: receiver firmware.

Both projects are based on ST's `SubGHz_Phy_PingPong` example and keep the same
generated STM32CubeIDE structure.

## Project Structure

```text
TX/SubGHz_Phy_PingPong/
RX/SubGHz_Phy_PingPong/
```

Each project contains:

- `Core/Inc` and `Core/Src`: board setup, peripherals, main loop, GPIO, RTC,
  USART, DMA, and system support code.
- `SubGHz_Phy/App`: application-level radio logic. This is where the TX/RX
  behavior is mainly implemented.
- `SubGHz_Phy/Target`: radio target configuration and board interface.
- `Drivers`, `Middlewares`, `Utilities`: ST HAL, CMSIS, SubGHz middleware, and
  helper libraries.
- `STM32CubeIDE/Debug`: generated makefiles and build outputs such as `.elf`,
  `.map`, `.list`, and `.bin`.

## TX Firmware

The transmitter sends messages over LoRa. AES payload encryption is controlled by
the `USE_TX_AES` compile-time macro:

- `USE_TX_AES=0`: sends plain messages.
- `USE_TX_AES=1`: encrypts the payload before transmission.

Normal TX build:

```sh
PATH="/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.macos64_1.0.100.202602081740/tools/bin:$PATH" \
make -C TX/SubGHz_Phy_PingPong/STM32CubeIDE/Debug clean all -j2
```

Output:

```text
TX/SubGHz_Phy_PingPong/STM32CubeIDE/Debug/TX.bin
```

Build AES and non-AES TX comparison binaries:

```sh
zsh scripts/build_tx_aes_compare.sh
```

Outputs:

```text
TX/SubGHz_Phy_PingPong/STM32CubeIDE/variants/TX_with_aes.bin
TX/SubGHz_Phy_PingPong/STM32CubeIDE/variants/TX_without_aes.bin
```

## RX Firmware

The receiver listens for LoRa packets and processes the received payload. Its
main application behavior is in:

```text
RX/SubGHz_Phy_PingPong/SubGHz_Phy/App/subghz_phy_app.c
```

Build RX:

```sh
PATH="/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.macos64_1.0.100.202602081740/tools/bin:$PATH" \
make -C RX/SubGHz_Phy_PingPong/STM32CubeIDE/Debug clean all -j2
```

Output:

```text
RX/SubGHz_Phy_PingPong/STM32CubeIDE/Debug/RX.bin
```

## Experiment Scripts

Only the scripts below are used for the single-node AES vs non-AES experiment:

- `scripts/build_tx_aes_compare.sh`: builds `TX_with_aes.bin` and
  `TX_without_aes.bin`.
- `scripts/run_iotlab_single_tx.sh`: runs one FIT IoT-LAB experiment using a
  selected TX `.bin` file on one STM32WL node. Use it to test either the AES or
  non-AES transmitter firmware in isolation.
- `scripts/run_iotlab_single_tx_compare_batch.sh`: runs repeated single-node
  experiments alternating `TX_with_aes.bin` and `TX_without_aes.bin`, generating
  matched runs for the AES vs non-AES comparison.
