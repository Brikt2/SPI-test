#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/time.h"

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/uart.h"
#include "hardware/structs/uart.h"
#include "hardware/regs/uart.h"

#include "ws2812.pio.h"


/* ============================================================
   LED-INNSTILLINGER
   ============================================================ */

/*
   RGB:
       3 DMX-kanaler per piksel
       341 piksler fra 2 universer

   RGBW:
       4 DMX-kanaler per piksel
       256 piksler fra 2 universer

   RGB er standard.
*/

#ifndef LED_FORMAT_RGB
#define LED_FORMAT_RGB 1
#endif


#if LED_FORMAT_RGB

    #define LED_CHANNELS_PER_PIXEL 3u
    #define LED_BITS_PER_PIXEL     24u
    #define LED_PIXEL_COUNT        341u

    #define DMX1_REQUIRED_CHANNELS 512u
    #define DMX2_REQUIRED_CHANNELS 511u

#else

    #define LED_CHANNELS_PER_PIXEL 4u
    #define LED_BITS_PER_PIXEL     32u
    #define LED_PIXEL_COUNT        256u

    #define DMX1_REQUIRED_CHANNELS 512u
    #define DMX2_REQUIRED_CHANNELS 512u

#endif


/* ============================================================
   DMX-INNSTILLINGER
   ============================================================ */

#define DMX_UNIVERSE_COUNT 2u

#define DMX_CHANNEL_COUNT 512u

#define DMX_BAUDRATE 250000u

#define DMX_DATA_BITS 8u

#define DMX_STOP_BITS 2u


/* DMX-univers 1 */

#define DMX1_UART uart0
#define DMX1_RX_PIN 1u


/* DMX-univers 2 */

#define DMX2_UART uart1
#define DMX2_RX_PIN 9u


/* ============================================================
   LED-INNSTILLINGER
   ============================================================ */

#define LED_DATA_PIN 2u

#define LED_BRIGHTNESS 60u

#define WS281X_FREQUENCY 800000u

#define LED_RESET_US 150u


/*
   Fargerekkefølge ut til LED-stripen.

   GRB er vanlig for WS2812.

   Sett RGB_WIRE_ORDER_GRB til 0 dersom stripen
   forventer RGB i stedet for GRB.

   RGBW bruker her GRBW som standard.
*/

#define RGB_WIRE_ORDER_GRB 1

#define RGBW_WIRE_ORDER_GRBW 1


/* ============================================================
   UART RINGBUFFER
   ============================================================ */

/*
   Vi bruker en separat ringbuffer for hvert DMX-univers.

   UART-interruptene kan dermed fortsette å motta data
   mens hovedprogrammet behandler LED-data.

   Bufferstørrelsen må være en toerpotens.
*/

#define RX_RING_SIZE 1024u

#define RX_RING_MASK (RX_RING_SIZE - 1u)


#if (RX_RING_SIZE & (RX_RING_SIZE - 1u)) != 0
#error "RX_RING_SIZE must be a power of two"
#endif


/* ============================================================
   DMX-TILSTANDER
   ============================================================ */

enum DmxState {

    DMX_WAIT_BREAK,

    DMX_WAIT_START_CODE,

    DMX_READ_CHANNELS

};


/* ============================================================
   DMX-DATASTRUKTUR
   ============================================================ */

typedef struct {

    /*
       Ringbuffer.

       Disse variablene brukes både av UART-interrupten
       og hovedprogrammet.
    */

    volatile uint16_t rx_ring[RX_RING_SIZE];

    volatile uint32_t rx_write_index;

    volatile uint32_t rx_read_index;

    volatile uint32_t software_overruns;


    /*
       DMX-kanalene.

       channels:
           Data fra pakken som mottas nå.

       completed_frame:
           Sist fullførte DMX-pakke.
    */

    uint8_t channels[DMX_CHANNEL_COUNT];

    uint8_t completed_frame[DMX_CHANNEL_COUNT];


    enum DmxState state;

    uint16_t channels_received;

    bool frame_ready;


    /*
       Statistikk.
    */

    uint32_t break_count;

    uint32_t complete_frame_count;

    uint32_t error_count;

} DmxUniverse;


/*
   Vi lager to separate DMX-universer.
*/

static DmxUniverse dmx_universe[DMX_UNIVERSE_COUNT];


/*
   Hvor mange kanaler vi må motta før en pakke
   regnes som komplett.
*/

static const uint16_t required_channels[DMX_UNIVERSE_COUNT] = {

    DMX1_REQUIRED_CHANNELS,

    DMX2_REQUIRED_CHANNELS

};


/* ============================================================
   UART INTERRUPTS
   ============================================================ */

/*
   Denne funksjonen leser alle tilgjengelige bytes
   fra en UART og legger dem i ringbufferen.

   Vi lagrer hele registerverdien, ikke bare databyten.

   Dermed beholder vi også BREAK- og feilinformasjonsbitene.
*/

static void dmx_uart_irq_drain(

    uart_inst_t *uart,

    DmxUniverse *universe

) {

    uart_hw_t *hw = uart_get_hw(uart);


    while ((hw->fr & UART_UARTFR_RXFE_BITS) == 0u) {

        const uint16_t raw_word = (uint16_t)hw->dr;


        const uint32_t write_index =
            universe->rx_write_index;


        const uint32_t next_index =
            (write_index + 1u) & RX_RING_MASK;


        const uint32_t read_index =
            universe->rx_read_index;


        /*
           Sjekk om ringbufferen er full.
        */

        if (next_index == read_index) {

            universe->software_overruns++;

        } else {

            universe->rx_ring[write_index] = raw_word;

            universe->rx_write_index = next_index;

        }

    }

}


/*
   UART0 interrupt handler.

   DMX-univers 1.
*/

static void dmx_uart0_irq_handler(void)

{

    dmx_uart_irq_drain(

        DMX1_UART,

        &dmx_universe[0]

    );

}


/*
   UART1 interrupt handler.

   DMX-univers 2.
*/

static void dmx_uart1_irq_handler(void)

{

    dmx_uart_irq_drain(

        DMX2_UART,

        &dmx_universe[1]

    );

}


/* ============================================================
   UART INITIALISERING
   ============================================================ */

static void dmx_uart_init_one(

    uart_inst_t *uart,

    uint rx_pin,

    uint irq_number,

    irq_handler_t handler

) {


    /*
       Koble GPIO til UART.
    */

    gpio_set_function(

        rx_pin,

        GPIO_FUNC_UART

    );


    /*
       Start UART med DMX-hastighet.
    */

    uart_init(

        uart,

        DMX_BAUDRATE

    );


    /*
       250000 baud.
       8 databiter.
       Ingen paritet.
       2 stoppbiter.
    */

    uart_set_format(

        uart,

        DMX_DATA_BITS,

        DMX_STOP_BITS,

        UART_PARITY_NONE

    );


    /*
       Aktiver FIFO.
    */

    uart_set_fifo_enabled(

        uart,

        true

    );


    /*
       Nullstill eventuelle gamle feil.
    */

    uart_get_hw(uart)->rsr = 0u;


    /*
       Registrer interrupt handler.
    */

    irq_set_exclusive_handler(

        irq_number,

        handler

    );


    irq_set_enabled(

        irq_number,

        true

    );


    /*
       Aktiver mottaksinterrupt.
    */

    uart_set_irqs_enabled(

        uart,

        true,

        false

    );

}


/* ============================================================
   INITIALISER DMX
   ============================================================ */

static void dmx_init(void)

{

    memset(

        dmx_universe,

        0,

        sizeof(dmx_universe)

    );


    /*
       DMX-univers 1.
    */

    dmx_uart_init_one(

        DMX1_UART,

        DMX1_RX_PIN,

        UART0_IRQ,

        dmx_uart0_irq_handler

    );


    /*
       DMX-univers 2.
    */

    dmx_uart_init_one(

        DMX2_UART,

        DMX2_RX_PIN,

        UART1_IRQ,

        dmx_uart1_irq_handler

    );

}


/* ============================================================
   LED PIO OG DMA
   ============================================================ */

static PIO led_pio = pio0;

static uint led_sm;

static uint led_program_offset;

static int led_dma;


/*
   Array med ferdige LED-verdier.

   Én uint32_t per piksel.
*/

static uint32_t led_pixels[LED_PIXEL_COUNT];


/* ============================================================
   LYSSTYRKE
   ============================================================ */

static inline uint8_t scale_brightness(

    uint8_t component

)

{

    return (uint8_t)(

        ((uint16_t)component *

        (LED_BRIGHTNESS + 1u)) >> 8

    );

}


/* ============================================================
   INITIALISER LED
   ============================================================ */

static void led_init(void)

{


    /*
       Finn en ledig PIO state machine.
    */

    led_sm = pio_claim_unused_sm(

        led_pio,

        true

    );


    /*
       Finn en ledig DMA-kanal.
    */

    led_dma = dma_claim_unused_channel(

        true

    );


    /*
       Last inn PIO-programmet.
    */

    led_program_offset = pio_add_program(

        led_pio,

        &ws2812_program

    );


    /*
       Koble GPIO til PIO.
    */

    pio_gpio_init(

        led_pio,

        LED_DATA_PIN

    );


    pio_sm_set_consecutive_pindirs(

        led_pio,

        led_sm,

        LED_DATA_PIN,

        1u,

        true

    );


    /*
       Hent standardkonfigurasjon.
    */

    pio_sm_config config =

        ws2812_program_get_default_config(

            led_program_offset

        );


    /*
       Side-set brukes til å styre LED-pinnen.
    */

    sm_config_set_sideset_pins(

        &config,

        LED_DATA_PIN

    );


    /*
       MSB først.

       Autopull.

       24 bit for RGB eller 32 bit for RGBW.
    */

    sm_config_set_out_shift(

        &config,

        false,

        true,

        LED_BITS_PER_PIXEL

    );


    /*
       Bruk hele TX FIFO til utdata.
    */

    sm_config_set_fifo_join(

        &config,

        PIO_FIFO_JOIN_TX

    );


    /*
       PIO-programmet bruker 10 instruksjonssykluser
       per WS281x-bit.

       800 kHz LED-signal.
    */

    const float divider =

        (float)clock_get_hz(clk_sys) /

        ((float)WS281X_FREQUENCY * 10.0f);


    sm_config_set_clkdiv(

        &config,

        divider

    );


    /*
       Initialiser PIO state machine.
    */

    pio_sm_init(

        led_pio,

        led_sm,

        led_program_offset,

        &config

    );


    pio_sm_set_enabled(

        led_pio,

        led_sm,

        true

    );


    /*
       Konfigurer DMA.
    */

    dma_channel_config dma_config =

        dma_channel_get_default_config(

            led_dma

        );


    channel_config_set_transfer_data_size(

        &dma_config,

        DMA_SIZE_32

    );


    channel_config_set_read_increment(

        &dma_config,

        true

    );


    channel_config_set_write_increment(

        &dma_config,

        false

    );


    channel_config_set_dreq(

        &dma_config,

        pio_get_dreq(

            led_pio,

            led_sm,

            true

        )

    );


    dma_channel_configure(

        led_dma,

        &dma_config,

        &led_pio->txf[led_sm],

        NULL,

        0u,

        false

    );

}


/* ============================================================
   HENT DMX-KANAL
   ============================================================ */

/*
   Vi behandler begge DMX-universene som ett sammenhengende
   kanalområde på 1024 kanaler.

   RGB:
       Kanal 1-512 = univers 1.
       Kanal 513-1024 = univers 2.

   RGBW:
       Samme prinsipp.
*/

static uint8_t get_global_dmx_channel(

    uint32_t global_channel

)

{

    const uint universe_index =

        global_channel / DMX_CHANNEL_COUNT;


    const uint local_channel =

        global_channel % DMX_CHANNEL_COUNT;


    return dmx_universe[universe_index]

        .completed_frame[local_channel];

}


/* ============================================================
   DMX TIL LED
   ============================================================ */

static void convert_dmx_to_leds(void)

{


    for (

        uint32_t pixel = 0;

        pixel < LED_PIXEL_COUNT;

        pixel++

    ) {


        /*
           Finn første DMX-kanal for pikselen.
        */

        const uint32_t base =

            pixel * LED_CHANNELS_PER_PIXEL;


        /*
           Les RGB-verdiene.
        */

        const uint8_t red = scale_brightness(

            get_global_dmx_channel(base)

        );


        const uint8_t green = scale_brightness(

            get_global_dmx_channel(base + 1u)

        );


        const uint8_t blue = scale_brightness(

            get_global_dmx_channel(base + 2u)

        );


#if LED_FORMAT_RGB

        /*
           RGB.
        */

        #if RGB_WIRE_ORDER_GRB

            led_pixels[pixel] =

                ((uint32_t)green << 24) |

                ((uint32_t)red << 16) |

                ((uint32_t)blue << 8);

        #else

            led_pixels[pixel] =

                ((uint32_t)red << 24) |

                ((uint32_t)green << 16) |

                ((uint32_t)blue << 8);

        #endif


#else

        /*
           RGBW.
        */

        const uint8_t white = scale_brightness(

            get_global_dmx_channel(base + 3u)

        );


        #if RGBW_WIRE_ORDER_GRBW

            led_pixels[pixel] =

                ((uint32_t)green << 24) |

                ((uint32_t)red << 16) |

                ((uint32_t)blue << 8) |

                (uint32_t)white;

        #else

            led_pixels[pixel] =

                ((uint32_t)red << 24) |

                ((uint32_t)green << 16) |

                ((uint32_t)blue << 8) |

                (uint32_t)white;

        #endif

#endif

    }

}


/* ============================================================
   SEND LED-DATA
   ============================================================ */

static void send_led_data(void)

{


    /*
       Konverter DMX til LED-verdier.
    */

    convert_dmx_to_leds();


    /*
       Sett DMA-adresse.
    */

    dma_channel_set_read_addr(

        led_dma,

        led_pixels,

        false

    );


    /*
       Antall piksler.
    */

    dma_channel_set_trans_count(

        led_dma,

        LED_PIXEL_COUNT,

        false

    );


    /*
       Start DMA.
    */

    dma_channel_start(

        led_dma

    );


    /*
       Vent på at DMA har overført alle dataene
       til PIO FIFO.
    */

    dma_channel_wait_for_finish_blocking(

        led_dma

    );


    /*
       Vent til PIO FIFO er tom.
    */

    while (!pio_sm_is_tx_fifo_empty(

        led_pio,

        led_sm

    )) {

        tight_loop_contents();

    }


    /*
       Sørg for at siste piksel er ferdig sendt
       og at reset-perioden er lang nok.
    */

    sleep_us(

        50u + LED_RESET_US

    );

}


/* ============================================================
   OPPDATER LED NÅR BEGGE UNIVERS ER KLARE
   ============================================================ */

static void render_leds_if_both_universes_ready(void)

{


    /*
       Vi må ha en komplett DMX-pakke fra begge universene.
    */

    if (

        !dmx_universe[0].frame_ready ||

        !dmx_universe[1].frame_ready

    ) {

        return;

    }


    /*
       Send LED-data.
    */

    send_led_data();


    /*
       Begge universene er nå behandlet.
    */

    dmx_universe[0].frame_ready = false;

    dmx_universe[1].frame_ready = false;

}


/* ============================================================
   DMX PAKKEBEHANDLING
   ============================================================ */

static void process_dmx_word(

    DmxUniverse *universe,

    uint16_t raw_word

)

{


    /*
       BREAK.

       UART registrerer BREAK gjennom BE-biten.
       Vi kontrollerer denne før FE fordi BREAK
       også kan gi framing error.
    */

    if ((raw_word & UART_UARTDR_BE_BITS) != 0u) {


        universe->break_count++;


        const uint universe_index =

            (universe == &dmx_universe[0]) ? 0u : 1u;


        /*
           Hvis forrige pakke hadde nok kanaler,
           lagrer vi den som fullført.
        */

        if (

            universe->state == DMX_READ_CHANNELS &&

            universe->channels_received >=

            required_channels[universe_index]

        ) {


            memcpy(

                universe->completed_frame,

                universe->channels,

                DMX_CHANNEL_COUNT

            );


            universe->frame_ready = true;


            universe->complete_frame_count++;

        }


        /*
           Nullstill kanal-telleren.
        */

        universe->channels_received = 0u;


        /*
           Nå venter vi på startkode.
        */

        universe->state = DMX_WAIT_START_CODE;


        return;

    }


    /*
       Kontroller andre UART-feil.
    */

    if (

        (raw_word & (

            UART_UARTDR_FE_BITS |

            UART_UARTDR_PE_BITS |

            UART_UARTDR_OE_BITS

        )) != 0u

    ) {


        universe->error_count++;


        universe->channels_received = 0u;


        universe->state = DMX_WAIT_BREAK;


        return;

    }


    /*
       Hent den faktiske databyten.
    */

    const uint8_t value =

        (uint8_t)(raw_word & 0xFFu);


    /*
       Behandle DMX-tilstanden.
    */

    switch (universe->state) {


        case DMX_WAIT_BREAK:


            /*
               Vent på BREAK.
            */

            break;



        case DMX_WAIT_START_CODE:


            /*
               Standard DMX startkode er 0x00.
            */

            if (value == 0x00u) {


                universe->channels_received = 0u;


                universe->state = DMX_READ_CHANNELS;


            } else {


                /*
                   Ikke standard startkode.
                   Vent på neste BREAK.
                */

                universe->channels_received = 0u;


                universe->state = DMX_WAIT_BREAK;

            }


            break;



        case DMX_READ_CHANNELS:


            /*
               Les kanalverdier.
            */

            if (universe->channels_received < DMX_CHANNEL_COUNT) {


                universe->channels[

                    universe->channels_received

                ] = value;


                universe->channels_received++;

            }


            break;

    }

}


/* ============================================================
   BEHANDLE UART RINGBUFFER
   ============================================================ */

static void process_uart_ring(

    DmxUniverse *universe

)

{


    uint32_t read_index =

        universe->rx_read_index;


    /*
       Ta et øyeblikksbilde av skriveindeksen.

       Nye data som kommer senere behandles neste runde.
    */

    const uint32_t write_snapshot =

        universe->rx_write_index;


    while (read_index != write_snapshot) {


        const uint16_t raw_word =

            universe->rx_ring[read_index];


        read_index =

            (read_index + 1u) & RX_RING_MASK;


        process_dmx_word(

            universe,

            raw_word

        );

    }


    universe->rx_read_index = read_index;

}


/* ============================================================
   STATUS
   ============================================================ */

static void print_status(void)

{


#if LED_FORMAT_RGB

    const char *format_name = "RGB";

#else

    const char *format_name = "RGBW";

#endif


    printf(

        "\nDMX -> LED kjører | format: %s | totalt %u piksler\n",

        format_name,

        (unsigned)LED_PIXEL_COUNT

    );


    printf(

        "DMX1: BREAK=%lu frames=%lu feil=%lu ring-overrun=%lu\n",

        (unsigned long)dmx_universe[0].break_count,

        (unsigned long)dmx_universe[0].complete_frame_count,

        (unsigned long)dmx_universe[0].error_count,

        (unsigned long)dmx_universe[0].software_overruns

    );


    printf(

        "DMX2: BREAK=%lu frames=%lu feil=%lu ring-overrun=%lu\n",

        (unsigned long)dmx_universe[1].break_count,

        (unsigned long)dmx_universe[1].complete_frame_count,

        (unsigned long)dmx_universe[1].error_count,

        (unsigned long)dmx_universe[1].software_overruns

    );


    fflush(stdout);

}


/* ============================================================
   MAIN
   ============================================================ */

int main(void)

{


    /*
       Start USB serial.
    */

    stdio_init_all();


    /*
       Initialiser DMX-mottakerne.
    */

    dmx_init();


    /*
       Initialiser LED-utgangen.
    */

    led_init();


#if LED_FORMAT_RGB

    printf(

        "DMX -> RGB LED startet: %u piksler.\n",

        (unsigned)LED_PIXEL_COUNT

    );

#else

    printf(

        "DMX -> RGBW LED startet: %u piksler.\n",

        (unsigned)LED_PIXEL_COUNT

    );

#endif


    printf(

        "DMX1: UART0 RX GP%u\n",

        DMX1_RX_PIN

    );


    printf(

        "DMX2: UART1 RX GP%u\n",

        DMX2_RX_PIN

    );


    printf(

        "LED DATA: GP%u\n",

        LED_DATA_PIN

    );


    printf(

        "LED frekvens: 800 kHz\n"

    );


    printf(

        "Lysstyrke: %u/255\n",

        LED_BRIGHTNESS

    );


    printf(

        "Venter på DMX fra begge universene...\n"

    );


    fflush(stdout);


    uint32_t last_report_ms =

        to_ms_since_boot(get_absolute_time());


    while (true) {


        /*
           Behandle DMX-univers 1.
        */

        process_uart_ring(

            &dmx_universe[0]

        );


        /*
           Behandle DMX-univers 2.
        */

        process_uart_ring(

            &dmx_universe[1]

        );


        /*
           Oppdater LED hvis begge universene
           har mottatt en komplett pakke.
        */

        render_leds_if_both_universes_ready();


        /*
           Skriv status omtrent hvert sekund.
        */

        const uint32_t now_ms =

            to_ms_since_boot(get_absolute_time());


        if (

            (uint32_t)(now_ms - last_report_ms) >= 1000u

        ) {


            print_status();


            last_report_ms = now_ms;

        }


        tight_loop_contents();

    }


    return 0;

}