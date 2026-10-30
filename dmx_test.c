#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/termbits.h>

int main(void)
{
    int serial = open("/dev/serial0", O_RDONLY | O_NOCTTY);

    if (serial < 0) {
        perror("Kunne ikke åpne seriellport");
        return 1;
    }

    struct termios2 options;

    if (ioctl(serial, TCGETS2, &options) < 0) {
        perror("TCGETS2");
        close(serial);
        return 1;
    }

    options.c_cflag &= ~(CBAUD | CSIZE | PARENB);
    options.c_cflag |= BOTHER | CS8 | CSTOPB | CLOCAL | CREAD;

    options.c_ispeed = 250000;
    options.c_ospeed = 250000;

    options.c_iflag = INPCK | PARMRK;
    options.c_oflag = 0;
    options.c_lflag = 0;

    options.c_cc[VMIN] = 1;
    options.c_cc[VTIME] = 0;

    if (ioctl(serial, TCSETS2, &options) < 0) {
        perror("TCSETS2");
        close(serial);
        return 1;
    }

    printf("UART-test startet. Venter på byte...\n");
    fflush(stdout);

    uint8_t buffer[32];

    while (1) {
        int n = read(serial, buffer, sizeof(buffer));

        if (n < 0) {
            perror("read");
            break;
        }

        if (n == 0)
            continue;

        printf("Mottatt %d byte: ", n);

        for (int i = 0; i < n; i++) {
            printf("%02X ", buffer[i]);
        }

        printf("\n");
        fflush(stdout);
    }

    close(serial);
    return 0;
}