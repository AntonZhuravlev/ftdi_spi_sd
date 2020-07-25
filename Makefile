CFLAGS_DEBUG += -O0 -g3 -fmessage-length=0
CFLAGS_RELEASE += -O3 -fmessage-length=0

CFLAGS += -Wextra -Wall -Wpointer-arith -Wbad-function-cast
CFLAGS += $(CFLAGS_RELEASE)

L_FILES = ftdi1

PROGRAMS += ftdi_spi
C_FILES := $(wildcard *.c)
OBJS += $(patsubst %.c, %.o, $(C_FILES))

all: $(PROGRAMS)

%.o: %.c
	$(CC) $(CFLAGS) $^ -c -o $@

ftdi_spi: $(OBJS)
	@echo "Linking object files: " $^
	$(CC) $(LFLAGS) $^ $(addprefix -l, $(L_FILES)) -o $@

clean:
	rm -f *.o $(PROGRAMS)
