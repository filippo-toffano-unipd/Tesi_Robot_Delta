Tesi Robot Delta
================

In questo repository è contenuto il programma che svolge le analisi cinematiche di posizione, velocità e accelerazione e l'analisi dinamica di forze e coppie agenti sul meccanismo Delta Robot.
Inoltre è stato implementato un algoritmo per la ricerca di una traiettoria ottimale dell'end-effector a partire da alcuni punti di passaggio e il tempo in cui si vuole completare lo spostamento.

---
## Setup

Vanno installati:

- gcc
- cmake
- gnuplot

Se usi una distribuzione basata su Ubuntu / Debian:

```bash
sudo apt-get install gcc
```

```bash
sudo apt-get install cmake
```

```bash
sudo apt-get install -y gnuplot
```

Se qualcosa non dovesse funzionare, prova ad eseguire prima:

```bash
sudo apt-get update -y
```

---
## Inserimento dati di input

L'inserimento dei dati va fatto dal file ./app/src/main.c inserendo il numero di punti nella macro 'POINTS' e i punti nel vettore 'points_elem' nel formato decritto direttamente nel programma.

---
## Compilazione

Dalla directory principale:

```bash
cd build && rm -r * && cmake.. && make && cd ..
```

---
## Generazione output

Dalla directory principale:

```bash
gnuplot plotscript.p
```