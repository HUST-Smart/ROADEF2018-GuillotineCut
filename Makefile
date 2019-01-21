CPP      = g++
CARGS    = -O2 -pthread

all: challengeSG

challengeSG: *.cpp
	$(CPP) $(CARGS) -o $@ $^

clean:
	rm -rf challengeSG