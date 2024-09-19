CC=gcc
CC_FLAGS=-Wall -Wextra -pedantic

RM=rm
RM_FLAGS=-v

TEST_BIN=run_tests

LIB_A=../src/libSgp30.a

all: $(TEST_BIN)

$(TEST_BIN): main.c $(LIB_A)
	$(CC) $(CC_FLAGS) -o $@ $^

test: $(TEST_BIN)
	./$(TEST_BIN) 2>&1

clean:
	$(RM) $(RM_FLAGS) $(TEST_BIN)
