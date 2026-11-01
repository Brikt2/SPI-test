
#include <stdio.h>
#include <stdint.h> //for å kunne bruke heltall for jeg definerer størrelsen på forhånd
#include <unistd.h> //kommunisere med operativsystem (hente data fra UART-porten og pauseprogrammet)
#include <signal.h> //får signaler fra operativsystemet, som ctrl+c for å avslutte
#include <errno.h> //forr feilsøking, det gir oss en variabel hvor funksjoner kan lagre grunne til at de feiler sånn at vi finner ut om det er feil i koden eller signalet.
#include <time.h> //Gir oss funksjoner for tid, som for å styre hvor ofte jeg får output i terminalen.
#include <fcntl.h> //lar oss åpne filer og eheter, trengs for å åpne seriellporten ut til teipen.
#include <sys/ioctl.h> //kallibrere uart dirrekte gjennom operativsystemet
#include <asm/termbits.h> //Samme som termios, men for linux. Lar meg kalibrere den serielle porten jeg bruker ut på teip
#include <ws2811.h> //Et bibliotek lagd spesifikt for styring av ledteip, lar meg konfe programmet med info om den spesifike teipen

#define LED_PIN 18
#define ANTALL_PIKSLER 60
#define ANTALL_CHANNELS 3 //per piksel
#define DMX_CHANNELS 512
#define UART_ENHET "/dev/serial0" //velger mellom standard mini-UART og den bra uarten jeg fikk ved å skru av bluetooth.

static volatile sig_atomic_t running = 1; //definere en varianbel for å bestemme om programme skal fortsette å kjøre, sånn at vi kan lytte etter ctrl+c senere i programmet.

static ws2811_t ledstring = { //ws2811_t er en struktert struct som er lagd for å kunne lagre all infoen proggrammet trenger om ledstripa på et sted.
    .freq = 800000, //signalfrekvensen skal være på 800.000 (det sto i dokmentasjonen)
    .dmanum = 10, //sier at den skal bruke DMA-channel 10 når den skriver direkte til RAM for å spare på cpu-en
    .channel = {
        [0] = { //channel 1 av 2, vi kan bruke to separate channels til å styre to striper med ulike innstillinger, så lenge bitraten er lik.
            .gpionum = LED_PIN, // henter konstanten på hvilken gpiopin teipen er tilkoblet.
            .count = ANTALL_PIKSLER, //henter konstanten på hvor mange piksler den skal sende signal til
            .invert = 0, //Vi trenger ikke invertere
            .brightness = 60, //1-100 hvor mye lys skal den gi på maks, legges som et ekstra regnelag etter at den opprinnelige dmx-verdien er hentet.
            .strip_type = WS2811_STRIP_GRB, //fargerekkefølge
        },
    },
};

enum MarkerState { //Enum er en datatype hvor vi gir navn til ulike heltall, ofte for å få oversikt over ulike scenarioer eller versjoner av noe. Her er det for å skille mellom de forskjellige typene tilstandene vi kan være i når vi leser bytes: 
    MARK_NORMAL, //leser vanlige bytes
    MARK_AFTER_FF, //vi har lest FF og må se hva som kommer etter
    MARK_AFTER_FF_00 //vi har lest FF00 og må se hva som kommer etterpå
};

enum DmxState { //Denne enumen forteller oss hvor vi er i lesingen av DMX-data.
    WAIT_BREAK, //venter på en ny dmx-pakke
    WAIT_STARTCODE, //har funnet break, og venter på startkoden
    READ_CHANNELS //har funnet startkoden og leser kanaler
};

static void avbryt(int signal) //avbryte programmet
{
    (void)signal; //ungå varsler om ubrukte parameter.
    running = 0; //setter running til 0 og avslutter dermed løkka og programmet
}

static int setup_uart(int fd) //konfe uart for å kunne kommunisere med MAX-brikka, fd står for file descriptor
{
    struct termios2 konfig; //lage en tabell, konfig, med datatypen termios 2, som er en standard for å definere ulike parametre for uart-kommunikasjon i linux

    if (ioctl(fd, TCGETS2, &konfig) < 0) { //ioctl er en funksjon som henter eksisterende innstillinger for seriell komunikasjon. fd forteller hvilken serielle port vi snakker via, tcgets2 forteller at vi vil hente dem, og &options peker på options som stedet for å lagre resultatet. Skjer det en feil, får vi et negativt tall tilbake
        perror("TCGETS2"); // fyll perror med en feilmelding som peker hit
        return -1; //avslutt funksjonen
    }
    //C-cflag er det feltet i konfig som inneholder innstillingene for selve uart-kommunikasjonen
    konfig.c_cflag &= ~(CBAUD | CSIZE | PARENB | PARODD | CSTOPB); //&= er AND i c, | er OR, ~inverterer det, og parantes definerer rekkefølgen operasjonene skal skje i. altså setter (CBAUD | CSIZE | PARENB | PARODD | CSTOPB) de parametrene til 1 før ~ inverterer dem til 0 og &= merger dem med de gjeldene innstilingene ved å si at med mindre de både sto på fra før og står på i de nye innstilingene er de av, sånn ungår vi å påvirke de andre. Altså flipper vi de innstillingen vi ville skru av til 0.
    //CBAUD = En bestemt baudrate
    //CSIZE = Antall databiter
    //PARENB = Paritet (trengs ikke til DMX)
    //PARODD = Om det skal være oddetalls- eller partalsparitet
    //CSTOPB = Antall stoppbiter
    
    konfig.c_cflag |= BOTHER | CS8 | CSTOPB | CLOCAL | CREAD; // Setter de nye innstillingene på tilsvarende måte, men med OR istedet for AND
    //BOTHER = sett egendefinert baudrate i c_ispeed og c_ospeed
    //CS8 = Bruke åtte databits
    //CSTOPB = Bruke to stoppbits
    //CLOCAL = ikke vent må modem-signal
    
    
    //CREAD = aktiver mottakeren
    konfig.c_ispeed = 250000; //Inngående hastighet
    konfig.c_ospeed = 250000; //Utgående hastighet

   
    konfig.c_iflag = INPCK | PARMRK; // disse leter egentlig etter feil i overføringene, men fordi de oppfatter BREAK-signalene som feil kan vi bruke det til å finne starten på en dmx-pakke.
    konfig.c_oflag = 0; //Skrur av unødvendig databehandling
    konfig.c_lflag = 0; //Skrur av unødvendig databehandling

    konfig.c_cc[VMIN] = 1; //Returnerer ikke data før den har fått inn minst en byte
    konfig.c_cc[VTIME] = 0; //Ingen tidsbegrensning som gjør at den returnerer noe etter en periode uten data, altså venter den til den får inn noe data.

    if (ioctl(fd, TCSETS2, &konfig) < 0) { //setter konfig som de nye innstillingene. Får vi et tall under 0, vet vi at noe er feil og lagrer det i perror og avslutter funksjonen
        perror("TCSETS2");
        return -1;
    }

    return 0;
}

//Konvertere DMX til verdier for LED
static void oppdater_led(const uint8_t dmx[]) //Funksjonen tar et array av 8-bits heltall som argument. Dette blir dmxchannels, derfor kaller jeg arrayet dmx. Startkoden er ikke med her. Funksjonen endrer ikke innholdet i arrayet, derfor er det en const.
{
    for (int i = 0; i < ANTALL_PIKSLER; i++) { // en løkke som itererer gjennok alle leds og fordeler 24 bit til hver

        int startkanal = i * ANTALL_CHANNELS; //første kanal for en piksel blir i * 3

        uint8_t red   = dmx[startkanal]; // definerer et 8-bits heltall som får verdien fra startkanalen i dmx-arrayet
        uint8_t green = dmx[startkanal + 1]; //definerer et 8-bits heltall som får verdien fra kanalen etter startkanalen i dmx-arrayet
        uint8_t blue  = dmx[startkanal + 2]; //definerer et 8-bits heltall som får verdien fra kanalen to kanaler etter startkanalen i dmx-arrayet

        uint32_t farge = //konstruerer et 32-bits heltall av fargeverdier
            ((uint32_t)red << 16) | //rød starter 16 bits telt bakfra
            ((uint32_t)green << 8) | //grønn starter 8 bits telt bakfra
            (uint32_t)blue; //blått starter helt til høyre

        ledstring.channel[0].leds[i] = farge; //lagrer verdiene inn i ledstring, sammen med konfingen av ledstripa. Velger channel 1 (indeks 0), av 2 i ledstring. Lagrer det til i i arrayet med ledverdier.
    }

    ws2811_return_t ret = ws2811_render(&ledstring); //sender det ferdig fyllte arrayet med ledverdier til stripa, ret er en variabel med datatype ws2811_return_t, som er en respons på sendiungen som forteller oss hvordan det gikk

    if (ret != WS2811_SUCCESS) { //sjekker om vi fikk en feil og skriver det til terminal
        fprintf(stderr, "LED-oppdatering feilet: %s\n",
                ws2811_get_return_t_str(ret));
    }
}

int main(void) //Selve programmet som kjøres
{
    signal(SIGINT, avbryt); //Hvis man trykker ctrl+c, kjør avbryt()
    signal(SIGTERM, avbryt); //Hvis man avbryter programmet, f.eks ved å trykke på det krysset i hjørnet, kjør avbryt()


    ws2811_return_t ret = ws2811_init(&ledstring); //gjør klar for å sende signal til leden, blant annet ved å gjøre det jeg har skrevet i konfen i ledstring

    if (ret != WS2811_SUCCESS) { //Hvis den ikke klarte å gjøre det
        fprintf(stderr, "LED-init feilet: %s\n",
                ws2811_get_return_t_str(ret));
        return 1;
    }

    for (int i = 0; i < ANTALL_PIKSLER; i++) { //Skru av alle piksler i utgangspunktet
        ledstring.channel[0].leds[i] = 0;
    }

    ws2811_render(&ledstring);

    // Åpne UART
    int serial = open(UART_ENHET, O_RDONLY | O_NOCTTY); // åpner den serielle porten, som linux behandler som en fil (banen står definert i en konstant i toppen av programmet). Når vi åpner fila/porten får vi tilbake en fd (file deskriptor) som vi lagrer i variabelen serial. 
        //O_RDONLY = bare lese, ikke skrive
        //O_NOCTTY = ikke bruke den som terminal
    if (serial < 0) { //lukker ledbiblioteket og avslutter programmet om det ikke går som det skal.
        perror("Kunne ikke åpne UART");
        ws2811_fini(&ledstring);
        return 1;
    }

    if (setup_uart(serial) < 0) { //sende inn uart-konfen, om den ikke fungerer får vi et negativt tall tilbake og lukker den serielle port og ledbiblioteket før vi avslutter programmet
        close(serial);
        ws2811_fini(&ledstring);
        return 1;
    }

    uint8_t dmx[DMX_CHANNELS] = {0}; // Her lager vi arrayet med dmx-verdiene

    enum MarkerState marker_state = MARK_NORMAL; //vi skal starte i MARK_NORMAL, fordi vi ennå ikke har rukket å møte en byte som er FF
    enum DmxState dmx_state = WAIT_BREAK; //vi skal starte i WAIT_BREAK, fordi vi må vente på at det skal starte en dmx-pakke før vi kan slenge oss på

    // variabler som teller dataen vi har jobbet oss gjennom
    int kanaler = 0; //Hvor mange channels har vi jobbet oss gjennom i den pakken vi er i nå
    unsigned long antall_breaks = 0; //Hvor mange breaks har vi funnet
    unsigned long antall_frames = 0; //Hvor mange ganger har vi oppdatert ledstripa
    unsigned long antall_error = 0; //Hvor mange feil har vi funnet i dmx-signalet

    time_t last_report = time(NULL); //henter nåværende tidspunkt for å bestemme når vi skal sende neste melding i terminalen.

    printf("DMX til LED startet.\n");
    printf("UART: %s, 250000 baud, 8N2\n", UART_ENHET); //Printer hvilken UART vi bruker
    printf("Venter på DMX...\n");
    fflush(stdout); //tømmer teksten som lå i tekstbufferen i programmet

    uint8_t buffer[256]; //Jeg lager en buffer med plass til 256 bytes hvor jeg kan mallomlagre en halv dmxpakke fra operativsystemet av gangen og så gå gjennom en byte av gangen.

    while (running) { //Løkken som er det egentlige programmet som kjører

        ssize_t n = read(serial, buffer, sizeof(buffer)); //ssize_t en datatype som kan lagre både positive og negative tall. serial er fd-en til den UART-en vi vil at det skal lese fra. buffer er stedet det skal lagres. sizeof(buffer) er hvor mange bytes vi maks vil lese (256). Funksjonen venter til det kommer data fra UART og kopierer dem til bufferen, så returnerer den størrelsen på bufferen som n. 

        if (n < 0) { //hvis n er mindre enn 0, hart det skjedd noe feil
            if (errno == EINTR) //Ble vi avbrutt, altså ikke en feil med signalet?
                continue;

            perror("UART read"); //ellers sender vi feilmelding til perror og avslutter whileløkka
            break;
        }

        if (n == 0) //hvis vi ikke fikk noe data i bufferen dropper vi den neste delen og starter på nytt
            continue;

        for (ssize_t j = 0; j < n; j++) { //Vi starter på første byte i bufferen og itererer gjennom til vi har behandlet alle bytes som hadde data

            uint8_t byte = buffer[j]; //Vi henter en og en byte fra buffer og lagrer i en variabel byte


            //
            int event = 0; //Her lagrer vi hvilken type hendelse vi har funnet. 0 = vi har ikke funnet en hendelse, 1 = Vanlig byte, 2 = BREAK, 3 = feil i byten
            uint8_t value = 0; //Her lagrer vi verdien om det er en helt vanlig byte

            switch (marker_state) { //her bruker vi enumen fra tidligere for å finne ut om vi har funnet en BREAK

                case MARK_NORMAL: //Hvis vi er i normal-staten
                    if (byte == 0xFF) { //sjekk om neste byte er FF
                        marker_state = MARK_AFTER_FF; //Bytt til MARK_AFTER_FF
                    } else {
                        event = 1; //lagre at vi fant en vanlig byte
                        value = byte; //lagre verdien til value
                    }
                    break;

                case MARK_AFTER_FF: //Hvis vi er i MARK_AFTER_FF-staten
                    if (byte == 0xFF) { //sjekk om neste byte er FF. Da er det 255, altså en normal byte
                        marker_state = MARK_NORMAL; 
                        event = 1;
                        value = 0xFF; //Lagre 255 til value
                    } else if (byte == 0x00) { //sjekk om neste byte er 00
                        marker_state = MARK_AFTER_FF_00; //Bytt til MARK_AFTER_FF_00
                    } else { 
                        marker_state = MARK_NORMAL; //gå tilbake til normal-staten
                        event = 3; //Lagre at det var en feil av noe slag
                    }
                    break;

                case MARK_AFTER_FF_00:
                    marker_state = MARK_NORMAL; //Nå går vi uansett tilbake til normalen

                    if (byte == 0x00) { //Se om vi får 00
                        event = 2; // BREAK registrert
                    } else {
                        event = 3; // feil
                    }
                    break;
            }

            if (event == 2) { // det var en BREAK

                antall_breaks++;

                if (dmx_state == READ_CHANNELS && 
                    kanaler >= ANTALL_PIKSLER * ANTALL_CHANNELS) { //hvis vi er ferdig å gjøre klart det vi skal sende på dmx, push signalet
                    oppdater_led(dmx);
                    antall_frames++;
                }

                kanaler = 0;
                dmx_state = WAIT_STARTCODE;
            }

            else if (event == 3) { // Hvis det var en feil, tell den og gå videre

                antall_error++;
                kanaler = 0;
                dmx_state = WAIT_BREAK;
            }

            else if (event == 1) { // Hvis det var en vanlig byte

                switch (dmx_state) { // bruke enumen for dmx-state

                    case WAIT_BREAK:
                        // Vent på neste BREAK
                        break;

                    case WAIT_STARTCODE:
                        if (value == 0x00) {
                            // Standard DMX-startkode
                            kanaler = 0;
                            dmx_state = READ_CHANNELS;
                        } else {
                            // Ikke startkode, vent videre
                            dmx_state = WAIT_BREAK;
                        }
                        break;

                    case READ_CHANNELS: // Hvis vi er i read-channels
                        if (kanaler < DMX_CHANNELS) { //Så lenge vi ikke har skrevet mer enn et univers
                            dmx[kanaler++] = value; //sett dmx til value og gå videre til neste kanal
                        }
                        break;
                }
            }
        }


        time_t sekund = time(NULL); //lagrer nåværende klokkeslett

        if (sekund != last_report) { //sjekker om nåværende klokkeslett er helt likt som det forrige, og kjøre funksjonen om det ikke er det

            printf("BREAK: %lu | DMX-pakker: %lu | Feil: %lu", //skrive antall breaks, frames og error
                   antall_breaks, antall_frames, antall_error);

            if (antall_frames > 0) { //Hvis vi har fått sendt en frame, print siste verdier for første led for å se at det virker
                printf(" | LED 1 RGB: %u, %u, %u",
                       dmx[0], dmx[1], dmx[2]);
            }

            printf("\n"); // linjeskift
            fflush(stdout); // tømmer tekstbuffferen til terminalen for å skrive med en gang

            last_report = sekund;
        }
    }

    printf("\nAvslutter...\n");

    //Lukke og avslutte
    close(serial);
    ws2811_fini(&ledstring);

    return 0;
}
