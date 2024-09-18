AR=ar
AR_FLAGS=rcs

CC=gcc
CC_FLAGS=-Wall -Wextra

RM=rm

all: libSgp30.a libSgp30.so

sgp30.o: sgp30.c
	$(CC) $(CC_FLAGS) -c $<

libSgp30.so: sgp30.o
	$(CC) -shared -fPIC -o $@ $<

libSgp30.a: sgp30.o
	$(AR) $(AR_FLAGS) $@ $<

clean:
	$(RM) -v *.o *.a *.so