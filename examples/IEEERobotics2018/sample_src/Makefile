
sumArr.so:
	g++ -c -fPIC sumArr.c -o sumArr.o
	g++ -shared -Wl,-soname,sumArr.so -o sumArr.so sumArr.o

clean:
	rm *.o *.so
