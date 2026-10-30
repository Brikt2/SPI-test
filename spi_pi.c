#include <stdio.h>
#include <unistd.h>
#include <pigpio.h>

#define SPI_CHANNEL 0
#define SPI_HASTIGHET 1000

int main(void) {
    // Laste inn pigpio-bibliotek-greiene
    if (gpioInitialise() < 0) {
        printf("Pigpio lasta ikke inn riktig.\n");
        return 1;
    }

    // åpne spi ut på channel
    int utgang = spiOpen(SPI_CHANNEL, SPI_HASTIGHET, 0);
    if (utgang < 0) {
        printf("Fikk ikke åpna spi-utgangen.\n");
        gpioTerminate();
        return 1;
    }

    // Data som skal sendes
    unsigned char red[]   = {245};
    unsigned char green[] = {100};
    unsigned char blue[]  = {50};

    unsigned char rgb[3] = {
        red[0],
        green[0],
        blue[0]
    };

    spiWrite(utgang, (char *)rgb, 3);

    // Avslutt pigpio og lukk utgangen
    spiClose(utgang);
    gpioTerminate();
    return 0;
}
