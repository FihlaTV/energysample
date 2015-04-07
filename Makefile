all:	test

test:	test.c
	gcc -Wall -o test test.c -lpcap
