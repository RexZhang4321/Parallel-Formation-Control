CC=mpic++
CFLAGS=-Wall -O3 -std=c++11
LDFLAGS=
SOURCES=main.cpp bicycle_model.cpp control_method.cpp defs.cpp Dstar.cpp formation_map.cpp model_components.cpp model.cpp
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=main

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm $(EXECUTABLE)
	rm *.o

