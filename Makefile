obj-m+=rfdev.o

CFLAGS_rfdev.o := -Wall -Wextra -DDEBUG -g -ggdb

CROSS = arm-linux-gnueabi-
ARCH  = arm
KERN_PATH = $(shell find /root/axiom-firmware/build/ -name "linux-v*.git")/

all: 	cross

cross:
	make CROSS_COMPILE=$(CROSS) ARCH=$(ARCH) -C $(KERN_PATH) M=$(PWD) modules
clean:
	make CROSS_COMPILE=$(CROSS) ARCH=$(ARCH) -C $(KERN_PATH) M=$(PWD) clean

native:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules
clean_native:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) clean
