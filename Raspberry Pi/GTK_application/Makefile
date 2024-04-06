app: main.c graph.h graph.c 
	gcc `pkg-config --cflags gtk+-3.0` -o app main.c graph.c `pkg-config --libs gtk+-3.0` -lm

mini: mini_app.c graph.h graph.c
	gcc `pkg-config --cflags gtk+-3.0` -o mini mini_app.c graph.c `pkg-config --libs gtk+-3.0` -lm

serial: serial_test.c
	gcc -o serial serial_test.c
