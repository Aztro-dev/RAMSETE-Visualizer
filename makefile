UNAME := $(shell uname)

ifeq ($(UNAME), Linux)
main: src/main.cpp
	g++ -g -o build/main src/main.cpp -D_DEFAULT_SOURCE -Wno-narrowing -Wno-missing-braces -O2 -D_DEFAULT_SOURCE -I. -I/home/aztro/Documents/Libraries/raylib/src -I/home/aztro/Documents/Libraries/raylib/src/external  -I/usr/local/include -I/home/aztro/Documents/Libraries/raylib/src/external/glfw/include -L. -L/home/aztro/Documents/Libraries/raylib/src -L/home/aztro/Documents/Libraries/raylib/src -L/usr/local/lib -lraylib -lGL -lm -lpthread -ldl -lrt -lX11 -latomic -DPLATFORM_DESKTOP -DPLATFORM_DESKTOP_GLFW
endif
ifeq ($(UNAME), Windows_NT)
main: src/main.cpp
	g++ -g -o build/main.exe src/main.cpp -Wno-narrowing -IC:\Users\azael\OneDrive\Desktop\Documents\GitHub\RAMSETE-Visualizer\raylib -Lraylib -lraylib -lgdi32 -lwinmm -I.
endif

clean:
	rm -r build/*

ifeq ($(UNAME), Linux)
run: main
	./build/main
endif
ifeq ($(UNAME), Windows_NT)
run:
	g++ -g -o build/main.exe src/main.cpp -Wno-narrowing -IC:\Users\azael\OneDrive\Desktop\Documents\GitHub\RAMSETE-Visualizer\raylib -Lraylib -lraylib -lgdi32 -lwinmm -I.
	./build/main.exe
endif
