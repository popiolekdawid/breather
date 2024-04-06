#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int main() {
    int fd;
    struct termios options;

    // Open the serial port (replace "/dev/ttyUSB0" with your serial port)
    fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("Error opening serial port");
        return 1;
    }

    // Configure serial port settings
    tcgetattr(fd, &options);
    cfsetispeed(&options, B9600);  // Set baud rate
    cfsetospeed(&options, B9600);
    options.c_cflag &= ~PARENB;     // No parity
    options.c_cflag &= ~CSTOPB;     // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;         // 8 data bits
    options.c_cflag &= ~CRTSCTS;    // Disable hardware flow control
    options.c_cflag |= CREAD | CLOCAL; // Enable receiver and ignore modem control lines

    // Set raw input mode
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;

    // Set the new attributes
    tcsetattr(fd, TCSANOW, &options);
    fcntl(fd, F_SETFL, 0);

    // Write data to the serial port
    char data[] = "Hello, world!";
    int bytes_written = write(fd, data, sizeof(data) - 1);
    if (bytes_written == -1) {
        perror("Error writing to serial port");
        close(fd);
        return 1;
    }

    // Read data from the serial port
    char buffer[256];
    int bytes_read = read(fd, buffer, sizeof(buffer) - 1);
    if (bytes_read == -1) {
        perror("Error reading from serial port");
        close(fd);
        return 1;
    }

    buffer[bytes_read] = '\0'; // Null terminate the received data
    printf("Received: %s\n", buffer);

    // Close the serial port
    close(fd);

    return 0;
}
