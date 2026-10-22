# Host tests for epd4g. Needs only gcc and make:  make -C epd4g/tests
CC      ?= gcc
CFLAGS  ?= -std=c99 -Wall -Wextra -Wpedantic -O1 -g
SRC      = ../src/epd4g.c ../src/epd4g_fb.c test_epd4g.c
INC      = -I../include -I.

.PHONY: all test variants golden clean
all test: t_default
	./t_default

t_default: $(SRC) golden.inc ../include/*.h
	$(CC) $(CFLAGS) $(INC) $(SRC) -o $@

# the same tests with other configurations
variants:
	@set -e; for d in "-DEPD4G_KEEP_POWER=0" "-DEPD4G_FULL_SHAKE_RP=1" "-DEPD4G_FULL_SHAKE_RP=3" \
	  "-DEPD4G_T_BLACK=30 -DEPD4G_T_WHITE=50" "-DEPD4G_T_LIGHT=5 -DEPD4G_T_DARK=12" "-DEPD4G_PROFILE" "-DEPD4G_INTERNAL_MASK=1"; do \
	  echo "== $$d"; $(CC) $(CFLAGS) $(INC) $$d $(SRC) -o t_variant && ./t_variant; done

# regenerate golden.inc from the stock Waveshare library: make golden WS=/path/to/e-Paper/STM32/STM32-F103ZET6/User/e-Paper
golden:
	$(CC) -std=gnu11 -Itools/stub -I$(WS) tools/gen_golden.c $(WS)/EPD_2in7_V2.c -o gen_golden && ./gen_golden > golden.inc

clean:
	rm -f t_default t_variant gen_golden
