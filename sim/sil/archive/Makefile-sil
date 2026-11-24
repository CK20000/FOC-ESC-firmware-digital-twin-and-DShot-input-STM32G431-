CC = gcc
# Changed -O3 to -O2 to prevent SMO loop unrolling / register spills.
# Added -fno-math-errno to remove the slow libc errno wrapper around sqrtf().
CFLAGS = -O2 -march=native -Wall -Wextra -fPIC -DHOST_BUILD -fno-math-errno -I. -I../../Inc
LDFLAGS = -shared -lm

TARGET = foc_sim.so
OBJS = foc_core.o foc_dshot.o virtual_stm32.o plant_c.o sim_bridge.o

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^

# Firmware sources from ../../Src
foc_core.o: ../../Src/foc_core.c
	$(CC) $(CFLAGS) -c $< -o $@

foc_dshot.o: ../../Src/foc_dshot.c
	$(CC) $(CFLAGS) -c $< -o $@

# SIL specific sources
virtual_stm32.o: virtual_stm32.c
	$(CC) $(CFLAGS) -c $< -o $@

plant_c.o: plant_c.c
	$(CC) $(CFLAGS) -c $< -o $@

sim_bridge.o: sim_bridge.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f *.o $(TARGET)