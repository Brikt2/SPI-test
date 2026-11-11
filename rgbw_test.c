
#define _GNU_SOURCE

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <time.h>
#include <sched.h>
#include <errno.h>

// --------------------------------------------------
// KONFIGURASJON
// --------------------------------------------------

#define LED_COUNT       60
#define LED_PIN         18

// FW1834-timing, nanosekunder
#define T0H_NS          350
#define T1H_NS          700
#define BIT_PERIOD_NS   1450

// Reset må være minst 80 us
#define RESET_US        200

// Teststyrke
#define TEST_STRENGTH   32

// GPIO-register
#define GPIO_GPFSEL1    1
#define GPIO_GPSET0     7
#define GPIO_GPCLR0     10

#define GPIO_MASK       (1U << LED_PIN)

// --------------------------------------------------
// GPIO
// --------------------------------------------------

static volatile uint32_t *gpio;

static int gpio_init(void)
{
    int fd = open("/dev/gpiomem", O_RDWR | O_SYNC);

    if (fd < 0) {
        perror("Kunne ikke åpne /dev/gpiomem");
        return -1;
    }

    gpio = mmap(
        NULL,
        4096,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        fd,
        0
    );

    close(fd);

    if (gpio == MAP_FAILED) {
        perror("Kunne ikke mappe GPIO-register");
        return -1;
    }

    // Sett GPIO18 som utgang
    gpio[GPIO_GPFSEL1] &= ~(7U << 24);
    gpio[GPIO_GPFSEL1] |=  (1U << 24);

    // Start med lavt signal
    gpio[GPIO_GPCLR0] = GPIO_MASK;

    return 0;
}


// --------------------------------------------------
// TIDSMÅLING
// --------------------------------------------------

static inline uint64_t now_ns(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);

    return (uint64_t)ts.tv_sec * 1000000000ULL
         + (uint64_t)ts.tv_nsec;
}

static inline void wait_until(uint64_t deadline)
{
    while (now_ns() < deadline) {
        // Aktiv venting for presis timing
    }
}


// --------------------------------------------------
// SEND ÉN BIT
// --------------------------------------------------


static inline void send_bit(uint8_t bit)
{
    // Registrer starten på databiten
    uint64_t bit_start = now_ns();

    // Sett GPIO18 HØY
    gpio[GPIO_GPSET0] = GPIO_MASK;

    // Start målingen etter at GPIO er satt høy
    uint64_t high_start = now_ns();

    // Bestem hvor lenge signalet skal være høyt
    uint32_t high_time = bit ? T1H_NS : T0H_NS;

    wait_until(high_start + high_time);

    // Sett GPIO18 LAV
    gpio[GPIO_GPCLR0] = GPIO_MASK;

    // Vent til neste databits start
    wait_until(bit_start + BIT_PERIOD_NS);
}



// --------------------------------------------------
// SEND ÉN BYTE, MSB FØRST
// --------------------------------------------------

static void send_byte(uint8_t value)
{
    for (int bit = 7; bit >= 0; bit--) {
        send_bit((value >> bit) & 1U);
    }
}


// --------------------------------------------------
// SEND ÉN RAMME TIL ALLE LED
// --------------------------------------------------

static void send_frame(
    uint8_t red,
    uint8_t green,
    uint8_t blue,
    uint8_t white
)
{
    for (int i = 0; i < LED_COUNT; i++) {

        // FW1834: RGBW
        send_byte(red);
        send_byte(green);
        send_byte(blue);
        send_byte(white);
    }

    // Hold linjen lav under reset
    gpio[GPIO_GPCLR0] = GPIO_MASK;

    usleep(RESET_US);
}


// --------------------------------------------------
// GJENTA RAMME FOR Å REDUSERE RISIKO FOR AVBRUDD
// --------------------------------------------------

static void show_color(
    const char *name,
    uint8_t red,
    uint8_t green,
    uint8_t blue,
    uint8_t white
)
{
    printf("Tester %s...\n", name);
    fflush(stdout);

    for (int i = 0; i < 8; i++) {
        send_frame(red, green, blue, white);
    }

    sleep(1);
}


// --------------------------------------------------
// MAIN
// --------------------------------------------------

int main(void)
{
    printf("FW1834 GPIO18-test\n");

    printf("LED-antall: %d\n", LED_COUNT);
    printf("GPIO: %d\n", LED_PIN);

    printf("T0H: %d ns\n", T0H_NS);
    printf("T1H: %d ns\n", T1H_NS);
    printf("Bitperiode: %d ns\n", BIT_PERIOD_NS);

    // Forsøk å redusere avbrudd under dataoverføring
    struct sched_param priority;

    priority.sched_priority = 50;

    if (sched_setscheduler(0, SCHED_FIFO, &priority) < 0) {
        perror("Kunne ikke sette SCHED_FIFO");
    }

    if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0) {
        perror("Kunne ikke låse minnet");
    }

    if (gpio_init() < 0) {
        return 1;
    }

    // Test hver kanal separat
    show_color("RØD", TEST_STRENGTH, 0, 0, 0);
    show_color("GRØNN", 0, TEST_STRENGTH, 0, 0);
    show_color("BLÅ", 0, 0, TEST_STRENGTH, 0);
    show_color("HVIT", 0, 0, 0, TEST_STRENGTH);

    // Slå av alle LED
    printf("Slår av LED...\n");

    send_frame(0, 0, 0, 0);

    gpio[GPIO_GPCLR0] = GPIO_MASK;

    munmap((void *)gpio, 4096);

    printf("Test ferdig.\n");

    return 0;
}
