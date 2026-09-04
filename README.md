# myDrone_test
STM32 + Raspberry Pi0 2W based drone

## Project modules
Per aggiornare i submolues
`git submodule update --remote --recursive`

# Struttura progetto:
- Architettura classica STM32
- `G070RB_TestPrj/Drivers`..............: contiene Drivers di accesso diretto all'HW
- todo: definire cartella apps
- todo: definire cartella stack_x

## Stato progetto
Il progetto attuale implementa un progetto di test su un firmware non-rtos based per permettere lo studio e implementazione di driver, layer e applicativi.

Questo progetto fa parte dell'insieme dell'architettura per la realizzazione di un drone, nello specifico:

[ Air ]
- STM32
    - FW
- Pi0
    - Communications and data streams to ground

[ Ground ]
- Linux
    - Controlls and data 
