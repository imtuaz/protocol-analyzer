CC      = gcc
CFLAGS  = -std=c99 -Wall -Wextra -Werror -O2 -Iinclude
SAN     = -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer
LIBS    = -lm

LIB_SRC = src/waveform.c src/events.c src/wave_io.c src/uart.c src/spi.c \
          src/i2c.c src/can.c src/report.c src/fuzz.c

.PHONY: all test fuzz demo clean

all: build/pa

build/pa: $(LIB_SRC) src/main.c include/*.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(LIB_SRC) src/main.c $(LIBS) -o $@

# Unit tests, built with AddressSanitizer + UndefinedBehaviorSanitizer
test: $(LIB_SRC) tests/test_main.c include/*.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(SAN) $(LIB_SRC) tests/test_main.c $(LIBS) -o build/test_runner
	./build/test_runner

# Fuzz test under sanitizers (catches crashes and memory bugs)
fuzz: $(LIB_SRC) src/main.c include/*.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(SAN) $(LIB_SRC) src/main.c $(LIBS) -o build/pa_san
	./build/pa_san fuzz 2000 1

demo: build/pa
	@mkdir -p out
	./build/pa demo out

clean:
	rm -rf build out
