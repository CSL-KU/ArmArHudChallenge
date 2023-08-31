CC=gcc
CXX=g++
PGMS=Bw PLL BkPLL

CFLAGS=-O2 -Wall -g
CXXFLAGS=$(CFLAGS)
LDFLAGS=-Wl,--no-as-needed -lrt

all: $(PGMS)

Bw: Bw.o
	$(CXX) $(CFLAGS) $(LDFLAGS) $< -o $@
PLL: PLL.o
	$(CXX) $(CFLAGS) $(LDFLAGS) $< -o $@
BkPLL: BkPLL.o
	$(CXX) $(CFLAGS) $(LDFLAGS) $< -o $@

install:
	cp -v $(PGMS) /usr/local/bin
clean:
	rm *.o $(PGMS)
