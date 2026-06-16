# Makefile - PC digital-twin build for the C6701 motion code.
#
#   make            build ./motion_sim
#   make run        build and run the default (movevp) scenario
#   make curve      build and run the curve scenario
#   make clean

CC      := gcc
DSP     := ../dsp
INC     := -Iinclude -I$(DSP)
CFLAGS  := -std=c11 -O2 -g -Wall -Wextra -DSIM_BUILD $(INC)
LDLIBS  := -lm

# DSP sources compiled unchanged (main.c only via #ifdef SIM_BUILD guards).
SRC := sim_main.c sim_hw.c serial_stub.c $(DSP)/main.c $(DSP)/interrupt.c
BIN := motion_sim

.PHONY: all run curve clean

all: $(BIN)

$(BIN): $(SRC) sim_hw.h include/*.h $(DSP)/*.h
	$(CC) $(CFLAGS) $(SRC) -o $(BIN) $(LDLIBS)

run: $(BIN)
	./$(BIN) movevp

curve: $(BIN)
	./$(BIN) curve

# Build + run + visualise in one step (SCN selects the scenario).
SCN ?= movevp

viz: $(BIN)            ## static plot
	./$(BIN) $(SCN) --out $(SCN)_log.csv
	cd ../python && python3 visualize.py ../sim/$(SCN)_log.csv

anim: $(BIN)           ## recorded animation
	./$(BIN) $(SCN) --out $(SCN)_log.csv
	cd ../python && python3 animate.py ../sim/$(SCN)_log.csv

live: $(BIN)           ## live: C piped straight into Python
	cd ../python && python3 animate_live.py $(SCN)

clean:
	rm -f $(BIN) *.csv
