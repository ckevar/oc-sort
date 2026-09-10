main: *.c
	c++ -O3 -Wall -g -o $@ $^ -I. -lm

# --- Hungarian's ---
correctness_fhungarian: hungarian.c hungarian.h test/correctness_fhungarian.c
	c++ -O3 -Wall -g -o $@ $^ -I. -lm

# --- Kalman's ---
correctness_kalman: utils.c track.h kalman.c test/correctness_kalman.c
	c++ -O3 -Wall -g -o $@ $^ -I. -lm
speed_kalman: utils.c track.h kalman.c test/speed_kalman.c
	c++ -O3 -Wall -g -o $@ $^ -I. -lm

clean:
	rm main correctness_fhungarian
