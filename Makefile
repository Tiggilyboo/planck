obj-m += planck.o
PLANCK_FLAGS += -g -DDEBUG
ccflags-y += ${PLANCK_FLAGS}
CC += ${PLANCK_FLAGS}

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
debug:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	EXTRA_FLAGS="${PLANCK_FLAGS}"
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
