CC=gcc
CFLAGES=-g -Wall
TARGET=robong
OBJS=test.o test4.o

$(TARGET): $(OBJS)
	$(CC) -o $@ $(OBJS)

test.o : test.c
	$(CC) -c -o test.o test.c

test4.o : test4.c
	$(CC): -c -o test4.o test4.c

clean:
	rm $(OBJS) $(TARGET)
	
