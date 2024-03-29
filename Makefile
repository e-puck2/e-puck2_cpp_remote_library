CC = g++
AR = ar
RM = rm

all:
	$(CC) -c SerialComm.cpp epuck2.cpp -std=c++11
	$(AR) -r libepuck2bt.a SerialComm.o epuck2.o

clean:
	$(RM) *.o
	$(RM) *.a
