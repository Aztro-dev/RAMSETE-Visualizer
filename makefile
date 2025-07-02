main: src/main.cpp
	g++ -o build/main src/main.cpp -D_DEFAULT_SOURCE -Wno-missing-braces -O2 -D_DEFAULT_SOURCE -I. -I/home/aztro/Documents/Libraries/raylib/src -I/home/aztro/Documents/Libraries/raylib/src/external  -I/usr/local/include -I/home/aztro/Documents/Libraries/raylib/src/external/glfw/include -L. -L/home/aztro/Documents/Libraries/raylib/src -L/home/aztro/Documents/Libraries/raylib/src -L/usr/local/lib -lraylib -lGL -lm -lpthread -ldl -lrt -lX11 -latomic -DPLATFORM_DESKTOP -DPLATFORM_DESKTOP_GLFW

clean:
	rm -r build/*

run: main
	./build/main
