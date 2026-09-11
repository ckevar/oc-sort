
.PHONY: clean all

all: soa.bin #aos.bin

soa.bin: *.c soa/*.c src/*.c
	c++ -O3 -Wall -g -o $@ $^ -I. -lm

aos.bin: *.c aos/*.c src/*.c
	c++ -O3 -Wall -g -o $@ $^ -I. -lm

# --- Hungarian's --- {
correctness_fhungarian: src/hungarian.c include/hungarian.h test/correctness_fhungarian.c
	c++ -O3 -Wall -g -o $@ $^ -I. -Iinclude -lm
# }


# --- Kalman's --- {
correctness_kalman: src/utils.c soa/track.h soa/kalman.c test/correctness_kalman.c
	c++ -O3 -Wall -g -o $@ $^ -I. -Iinclude -lm

speed_kalman: src/utils.c soa/track.h kalman.c test/speed_kalman.c
	c++ -O3 -Wall -g -o $@ $^ -I. -Iinclude -lm
# }

clean:
	rm correctness_fhungarian correctness_kalman speed_kalman *.bin
