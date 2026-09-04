# Studio possibile architettura sw RTOS

## Threads principali
- osThreadId FlightControl_Handle;
- osThreadId ReadSensors_Handle;
- osThreadId ExtCommunications_Handle;

#### FlightControl_Handle
- controllo PID su valori letti disponibili

#### ReadSensors_Handle
- lettura dei sensori e aggiornamento valori correnti

#### ExtCommunications_Handle
- gestione messaggi in entrata e uscita dal dispositivo

