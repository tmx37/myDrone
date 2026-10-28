# myDrone
Drone basato su STM32 + Raspberry Pi Zero 2W.

## Stato progetto
Il progetto attuale è un firmware di test bare-metal (senza RTOS) per lo studio e l'implementazione di driver, layer e applicativi.

Fa parte dell'architettura complessiva del drone:

**Air**
- STM32
    - Firmware (questo repository)
- Raspberry Pi Zero 2W
    - Comunicazioni e flussi dati verso terra

**Ground**
- Linux
    - Controlli e dati

## Moduli del progetto
I driver dei sensori sono submodule Git. Per scaricarli dopo un clone:

```sh
git submodule update --init --recursive
```

In alternativa, è possibile scaricarli direttamente durante il clone:

```sh
git clone --recurse-submodules https://github.com/tmx37/myDrone.git
```

Per aggiornare i submodule alla revisione remota più recente:

```sh
git submodule update --remote --recursive
```

## Struttura progetto
Struttura classica di un progetto STM32CubeMX:

- `G070RB_TestPrj/`: progetto di test per STM32G070RB (configurazione in `G070RB_TestPrj.ioc`)
    - `Core/`: codice applicativo e generato da CubeMX (`main.c`, interrupt, wrapper I2C)
    - `Drivers/`: HAL STM32G0xx, CMSIS e driver di accesso diretto all'HW (submodule `drv-*`)
- `Docs/`: datasheet dei sensori e documenti di analisi dell'architettura
- todo: definire cartella apps
- todo: definire cartella stack_x

## Build e flash
Serve [STM32CubeCLT](https://www.st.com/en/development-tools/stm32cubeclt.html), con CMake, Ninja, `arm-none-eabi-gcc` e `STM32_Programmer_CLI` nel PATH. Da `G070RB_TestPrj/`:

```sh
cmake --preset Debug
cmake --build --preset Debug
STM32_Programmer_CLI --connect port=swd --download build/Debug/G070RB_TestPrj.elf -hardRst -rst --start
```

Gli stessi passaggi sono disponibili come task VS Code ("Build + Flash") in `G070RB_TestPrj/.vscode/`.
