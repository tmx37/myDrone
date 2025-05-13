# myDrone
STM32 + Raspberry Pi0 2W based drone

### STM32 (+ FreeRTOS) -> Controllo driver diretto + Comunicazione con Raspberry per comandi ed invio info + Procedure safety critiche

Idee schede:
- NUCLEO-H753ZI
- STM32F407G-DISC1

Considerazioni thread previsti:
- Controllo Stabilità (critico, real-time) // per correzzioni critiche direttamente su motori tramite accesso ai sensori
- API sensori ambientali (BMP180, GPS, HMC5883L) // Valutare se fornire un API unica, in quanto i sensori forniscono misurazioni ridondanti
- API sensori posizione/safety (MPU60X0, Distanza laser)
- API sensori tensione/corrente
- API controllo ESC (con vincoli imposti dal thread di stabilità) // Il controllo degli ESC deve essere limitato dal thread di "Controllo stabilità" quando si verificano situazioni "critiche" o per evitare si verifichino
- Communication Layer con Raspberry Pi (seriale, probabilmente UART o USB)

Considerazioni risorse necessarie:
- Sistema operativo real-time (RTOS) richiesto (FreeRTOS o CMSIS-RTOS).
- Scheduler a priorità per dare priorità ai thread critici (controllo stabilità).
- RAM ≥ 128 KB, specialmente per buffer, stack e gestione delle periferiche.
- FPU (unità a virgola mobile) per calcoli con sensori inerziali e GPS.
- Almeno 2 UART + 1 I2C + 1 SPI per gestire i sensori e la comunicazione col PI.
- Timer/PWM per il controllo degli ESC.

### Raspberry PI Zero 2W -> Comunicazione con STM32 per invio comandi e ricezione info + Streaming video RSTP con Controller + Comunicazione con Controller
- Funzione: Direzione tramite coordinate GPS
- Funzione: Streaming video
- Funzione: Trova zona sicura per atterraggio su piano
- Funzione: Naviga su set di coorinate in modo safe
- Funzione: Segui oggetti con riconoscimento immagini
- Comunication layer with STM32 
- Comunication layer with Controller (C UDP Socket)

### Controller
- Comunication layer with PI (C UDP Socket)
