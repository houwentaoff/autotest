default:all

all:autotest_server

include ../env.mk

INC:=-I../jsonrpc-c/include
CFLAGS += $(INC)
LD:= -L../third_party/lib/arm/ -ljsonrpcc -lev -lm

autotest_server:autotest_server.o
	$(CC) $^ -o $@ $(LD)

%.o:%.c
	$(CC) -c $(CFLAGS) $< -o $@

clean:
	-@$(RM) *.o autotest_server

.PHONY:clean
