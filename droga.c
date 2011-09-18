/*
 * Lukasz Panek
 * Sterowanie robotem RFID
 * Galez optymalizacja
 */

#define PC_COMPILATION
#define ARDUINO_DB
#define ROZSZERZONA_SIATKA_T

#ifdef PC_COMPILATION
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "test.h"
#endif

#ifndef PC_COMPILATION
#include <WProgram.h>
#include <EEPROM.h>
#endif
/****** DIJKSTRA ***************/

#define LICZBA_WEZLOW 16
#define MAX_SASIADOW_PROSTOKAT 4
#define L_KOLUMN 4
#define NO_DEF 249
#define INF 244
#define POZ_NIEZNANA (-1)

#ifdef PC_COMPILATION
extern serial Serial;
extern char PORTD;
extern eeprom EEPROM;
#endif

/* Globalna tablica poprzednikow wyliczana w algorytmie dijkstry.
 * Wedlug niej laczymy wezly tworzac scierzke do wezla 'cel'*/
int P[LICZBA_WEZLOW] = {0};

/* Przechowuje kolejke celow 
 * Jesli nie programowanio celow to jednym domyslnym celem 
 * jest wezel 0 */
int C[9] = {0,-1,-1,-1,-1,-1,-1,-1,-1};

/* Definicja sasiadow poszczegolnych wezlow dla siatki prostokatnej*/
uint8_t Q_P[][L_KOLUMN] =
{
    {1,4,INF,INF}, {0,2,5,INF}, {1,3,6,INF}, {2,7,INF,INF},
    {0,5,8,INF}, {1,6,9,4}, {2,7,10,5}, {3,11,6,INF},
    {4,9,12,INF}, {5,10,13,8}, {6,11,14,9}, {7,15,10,INF},
    {8,13,INF,INF}, {9,14,12,INF}, {10,15,13,INF}, {11,14,INF,INF}
};

#ifdef ROZSZERZONA_SIATKA_T
/* Rozszerzona siatka trojkatna. W tej siatce punkty na brzegach posiadaja
 * wiecej sasiadow niez w siatce podstatwowej. Niestety zeby wszystko dobrze
 * dzialalo trzeba bylo dodac obsluge skretow o 60 i 120 stopni */
uint8_t Q_T[][L_KOLUMN] =
{
    {4,1,INF,INF},{0,2,4,5},{1,3,5,6},{2,6,7,INF},
    {0,1,5,8},{1,2,8,9},{2,3,9,10},{3,6,10,11},
    {4,5,9,12},{5,6,12,13},{6,7,13,14},{7,10,14,15},
    {8,9,13,INF},{9,10,12,14},{10,11,13,15},{14,11,INF,INF}
};

#else
/* Definicja sasiadow poszczegolnych wezlow dla siatki trojkatnej*/
uint8_t Q_T[][L_KOLUMN] =
{
    {4,INF,INF,INF}, {4,5,INF,INF}, {5,6,INF,INF}, {6,7,INF,INF},
    {0,1,8,9}, {1,2,9,10}, {2,3,10,11}, {3,11,INF,INF},
    {4,12,INF,INF}, {4,5,12,13}, {5,6,13,14}, {6,7,14,15},
    {8,9,INF,INF}, {9,10,INF,INF}, {10,11,INF,INF}, {11,INF,INF,INF}
};

#endif //ROZSZERZONA_SIATKA_T

/* Przechowywanie aktualnego wektora ruch pojazdu */
struct wektor_ruchu{
    int pp;
    int ap;
    int np;
}POZ;

/* Parametr funkcji zapisujacych do EEPROMu.
 * PRZESZKODY- zapisywanie w bloku o adr. 1-10
 * CELE - zapisywanie w bloku o adr. 11-20 */
enum blok_t{
    PRZESZKODY = 1,
    CELE = 2
};

/*********** HARDWARE *************/
/* Zwiazane z funkcja odczytaj_karte */
#define CZYTAJ_DO_SKUTKU 0
#define PROBKUJ 1

/* Opoznienia potrzebne do skretu o 60,90,120 stopni */
enum cz_skretu{
    S_60 = 1533,
    S_90 = 2300,
    S_120= 3066
};

/* STAN POJAZDU */
volatile int CEL = 0;                    //nr wezla do ktorego pojazd jedzie
volatile int LICZBA_CELOW = 0;           //liczba zdefinniowanych celow
volatile char METODA_STEROWANIA = 'P';   //info o typie siatki lub jej braku

/* Przykladowe nr kart, posortowane! */
const int ID_KART[LICZBA_WEZLOW] = 
{ 
    4850,4867,4967,5054,
    5065,5070,5166,5255,
    5269,5348,5349,5450,
    5452,5555,5566,5648
};
/***** KONIEC DEKLARACJI ZMIENNYCH **********/


static void dijkstra(const int, const uint8_t Q[][L_KOLUMN]);
static void drukuj_wyniki(const int*, const int*);
static int minimum(const int*, const int, const int*);
static int wektor(const int, const int);
static void kieruj(const int, const int, const int);
static int znajdz_nr_wezla(const int, const int, const int);
static int numer_wezla(const int);
static void wyklucz_wezel(const int w_s, uint8_t Q[][L_KOLUMN]);
static void przywroc_wezel(uint8_t Q[][L_KOLUMN]);

static void zapisz_w_eeprom(const int* tab, const enum blok_t);
static void programuj_pamiec(const enum blok_t, const int);
static void zaladuj_eeprom(void);
static void zrzut_eeprom(void);
inline static int liczba_celow(void);
inline static int liczba_przeszkod(void);

static void dodaj_przeszkode(const int);
static void dodaj_cel(const int, const int);
static int odczytaj_karte(const char);
static void stop(void);
static void start(void);
static void skrecajLewo(enum cz_skretu);
static void skrecajPrawo(enum cz_skretu);
static void error(void);
void setup();
void loop();

#ifdef PC_COMPILATION
int main(int argc, char* argv[])
{
    if(argc > 1){
        if(!strcmp(argv[1],"P"))
            digitalWrite(A2, HIGH);
        else if(!strcmp(argv[1],"T"))
            digitalWrite(A3, HIGH);
        else if(!strcmp(argv[1],"N"))
            digitalWrite(A4, HIGH);
        else if(!strcmp(argv[1],"C"))
            digitalWrite(A5, HIGH);

        if(argc > 2 && !strcmp(argv[2],"dijkstra")){
            setup();
            return 0;
        }    
    }else{
        digitalWrite(A2, HIGH);
    }
    setup();
    loop();
    return 0;
}
#endif

/* uproszczenie wywolania fun. znajdz_nr_wezla.
 * Wyszukuje w fizycznych numerach kart RFID index danej karty */
static int numer_wezla(const int id)
{
    return znajdz_nr_wezla(id, 0, LICZBA_WEZLOW-1);
}

/* Przeszukiwanie binarne */
static int znajdz_nr_wezla(const int id, const int poczatek, const int koniec)
{
    if( poczatek > koniec )
        return -1;

    int index = (poczatek + koniec) / 2;
    if( id == ID_KART[index] ){
        return index;
    }
    else if( id > ID_KART[index] ){
        return znajdz_nr_wezla(id, index +1, koniec);
    }
    else if( id < ID_KART[index] ){
        return znajdz_nr_wezla(id, poczatek, index -1);
    }

    return -1;
}

/* Kierunki */
#define N 0
#define E 3
#define S 6
#define W 9
#define NE 2
#define NW 10
#define SE 5
#define SW 7

/* Okresla wektor na podstawie punktu poczatkowego
 * i punktu nastepnego */
static int wektor(const int ap, const int np)
{
    int k = np - ap;
    if(METODA_STEROWANIA == 'P'){
        switch(k){
            //roznica miedzy wezlami -4 wiec kierunek N
            case (-L_KOLUMN):
                return N;
            //riznica miedy wezlami 1 wiec kierunek E itd.
            case (1):
                return E;
            case (L_KOLUMN):
                return S;
            case (-1):
                return W;
            default:
                return -1;
        }
    }else if(METODA_STEROWANIA == 'T'){
        switch(k){
            case (-1):
                return W;
            case (1):
                return E;
            case (-L_KOLUMN+1):
                return NE;
            case (L_KOLUMN):
                return SE;
            case (L_KOLUMN-1):
                return SW;
            case (-L_KOLUMN):
                return NW;
            default:
                return -1;
        }
    }
    return -1;
}

/* Decyduje o wywalaniu skretu w ktoras ze stron lub jezdzie na wprost*/
static void kieruj(const int pp, const int ap, const int np)
{
    int orientacja = wektor(pp, ap);
    int nastepny_kierunek = wektor(ap, np);

    if( orientacja == nastepny_kierunek
          || orientacja == -1
          || nastepny_kierunek == -1 )
    {
#ifdef ARDUINO_DB
        Serial.print("kieruj:jazda prosto\n");
#endif
        return;
    }

    /* Dopasowuje wszystko co implikuje skret w lewo o 90 stopnik */
    else if(  ((orientacja == N) && (nastepny_kierunek == W)) 
            ||((orientacja == E) && (nastepny_kierunek == N))
            ||((orientacja == S) && (nastepny_kierunek == E))
            ||((orientacja == W) && (nastepny_kierunek == S))
            ||((orientacja ==NE) && (nastepny_kierunek ==NW))
            ||((orientacja ==SE) && (nastepny_kierunek ==NE))
            ||((orientacja ==SW) && (nastepny_kierunek ==SE))
            ||((orientacja ==NW) && (nastepny_kierunek ==SW)) )
    {
#ifdef ARDUINO_DB
        Serial.print("kieruj:skrecam w lewo o 90 st\n");
#endif
        skrecajLewo(S_90);
    }

    /* Dopasowuje wszystko co implikuje skret w prawo o 90 stopni */
    else if(  ((orientacja == N) && (nastepny_kierunek == E))
            ||((orientacja == E) && (nastepny_kierunek == S))
            ||((orientacja == S) && (nastepny_kierunek == W))
            ||((orientacja == W) && (nastepny_kierunek == N)) 
            ||((orientacja ==NE) && (nastepny_kierunek ==SE))
            ||((orientacja ==SE) && (nastepny_kierunek ==SW))
            ||((orientacja ==SW) && (nastepny_kierunek ==NW))
            ||((orientacja ==NW) && (nastepny_kierunek ==NE)) )
    {
#ifdef ARDUINO_DB
            Serial.print("kieruj:skrecam w prawo o 90 st\n");
#endif
            skrecajPrawo(S_90);
    }

#ifdef ROZSZERZONA_SIATKA_T
    /* Obsluga rozszerzonej siatki trojkatnej. Skrety o 60 i 120 stopni */
    else if(  ((orientacja == NE) && (nastepny_kierunek == E))
            ||((orientacja == SW) && (nastepny_kierunek == W))
            ||((orientacja == W ) && (nastepny_kierunek ==NW))
            ||((orientacja == E ) && (nastepny_kierunek ==SE)) ) 
                        { skrecajPrawo(S_60);
#ifdef ARDUINO_DB
                       Serial.print("kieruj:skrecam w prawo o 60 st\n"); 
#endif
                        }
    else if(  ((orientacja == NW) && (nastepny_kierunek == E))
            ||((orientacja == SE) && (nastepny_kierunek == W))
            ||((orientacja == W ) && (nastepny_kierunek ==NE))
            ||((orientacja == E ) && (nastepny_kierunek ==SW)) )
                        { skrecajPrawo(S_120);
#ifdef ARDUINO_DB
                        Serial.print("kieruj:skrecam w prawo o 120 st\n");
#endif
                        }
    else if(  ((orientacja == NW) && (nastepny_kierunek == W))
            ||((orientacja == SE) && (nastepny_kierunek == E))
            ||((orientacja == W ) && (nastepny_kierunek ==SW))
            ||((orientacja == E ) && (nastepny_kierunek ==NE)) )
                        { skrecajLewo(S_60); 
#ifdef ARDUINO_DB
                        Serial.print("kieruj:skrecam w lewo o 60 st\n");
#endif
                        }
    else if(  ((orientacja == NE) && (nastepny_kierunek == W)) 
            ||((orientacja == SW) && (nastepny_kierunek == E))
            ||((orientacja == W ) && (nastepny_kierunek ==SE))
            ||((orientacja == E ) && (nastepny_kierunek ==NW)) )
                        { skrecajLewo(S_120); 
#ifdef ARDUINO_DB
                        Serial.print("kieruj:skrecam w lewo o 120 st\n");
#endif
                        }

#endif

    return;
}

/* Wyszukuje element o nakrotszej drodze do celu */
static int minimum(const int* d, const int size, const int* inS)
{
    int m = INF; 
    int m_index = INF;

    int i = 0;
    for(; i < size; i++){
        //warunek: jest mniejsze od aktualnego m i nie znajduje sie w przerobionych wezlach
        if( (d[i] < m) && (inS[i] != 1) ){
            m       = d[i];
            m_index = i;
        }
    }
    return m_index;
}

/* 
 * Buduje tablice poprzednikow (P[]) pod katem najkrotszej drogi 
 * do wezla 'cel'. Dziala dla siatki prostokatnej i trojkatnej.
 */
static void dijkstra(const int cel, const uint8_t Q[][L_KOLUMN]){
#ifdef ARDUINO_DB
    Serial.print("Wywolanie: dijkstra(CEL, Q_?)\n");
#endif
    int d[LICZBA_WEZLOW];
    int inS[LICZBA_WEZLOW] = {0}; //tablica informujaca o obecnosci w zbiorze Q

    int i,j = 0;

    for(i=0; i < LICZBA_WEZLOW; i++){                       //inicializujemy d maksymalna odlegloscia (nie NULL!)
        d[i] = NO_DEF;
    }

    d[cel] = 0;                                             //droga do celu z niego samego jest rowna 0
    P[cel] = cel;                                           //cel nie ma poprzednika

    int h = 0;                                              //pomocnicza do obliczania dlugosci drogi do nastepnikow
    int v;                                                  //aktualnie badany wezel
    for(i = 0; i < LICZBA_WEZLOW; i++){                     //AKTUALNIE ROZPATRYWANY WEZEL
        v = minimum(d, LICZBA_WEZLOW, inS);                 //index najmniejszego wezla

        inS[v] = 1;                                         //przesuwamy wezel do zbioru S - przerobionych wezlow

        h = d[v] + 1;                                       //dlugosc drogi do hipotetycznych nastepnikow aktualnego wezla
        for(j=0; j < MAX_SASIADOW_PROSTOKAT; j++){          //OGLADAMY KAZDEGO SASIADA aktualnie rozpatrywanego wezla
            if( (Q[v][j] != INF) && (inS[ Q[v][j] ] != 1) ){//Czy sasiad nie jest juz przerabianym wezlem 
                if( h < d[ Q[v][j] ]){                      //Czy droga to tego sasiada jest dluzsza od jego aktualnej?
                    P[Q[v][j]] = v;                         //Sasiad dostaje poprzednika w postaci aktualnie rozpatrywanego wezla v
                    d[Q[v][j]] = h;                         //Nowa dlugosc drogi do wezla = droga poprzednika + 1
                }
            }
        }

    }
#ifdef PC_COMPILATION
    drukuj_wyniki(d,P);
#endif
}

#ifdef PC_COMPILATION
static void drukuj_wyniki(const int* d, const int* p){
    int x = 0;
    printf("Q:");
    for(x=0; x < LICZBA_WEZLOW; x++){
        if(x > 9)
            printf("%i ", x);
        else
            printf("%i  ", x);
    }
    printf("\n");
    
    printf("d:");
    for(x=0; x < LICZBA_WEZLOW; x++){
        if(d[x] > 9)
            printf("%i ", d[x]);
        else
            printf("%i  ", d[x]);
    }
    printf("\n");

    printf("P:");
    for(x=0; x < LICZBA_WEZLOW; x++){
        if(p[x] > 9)
            printf("%i ", p[x]);
        else
            printf("%i  ", p[x]);
    }
    printf("\n");
}
#endif

#ifdef ARDUINO_DB
static void zrzut_eeprom(void)
{
    int i;
    uint8_t val;
    for(i = 0; i < 21; i++){
        Serial.print(i, DEC);
        if(i < 10)
            Serial.print("   ");
        else if( i > 9 )
            Serial.print("  ");
    }
    Serial.print("\n");
    for(i = 0; i < 21; i++){
        val = EEPROM.read(i);
        Serial.print(val, DEC);
        if( val < 10 )
            Serial.print("   ");
        else if( val > 9 && val < 100)
            Serial.print("  ");
        else
            Serial.print(" ");
    }
    Serial.print("\n");
}
#endif

/* Obsluga sretu w lewo */
static void skrecajLewo(enum cz_skretu czas_skretu_o_X_stopni){
#ifdef ARDUINO_DB
    Serial.print("Wywolanie: skrecajLewo()\n");
#endif
    PORTD &= ~(1<<2);
    delay(czas_skretu_o_X_stopni);
    PORTD |= (1<<2);

    return;
}

/* Obsluga skretu w prawo */
static void skrecajPrawo(enum cz_skretu czas_skretu_o_X_stopni){
#ifdef ARDUINO_DB
    Serial.print("Wywolanie: skrecajPrawo()\n");
#endif
    PORTD &= ~(1<<3);
    delay(czas_skretu_o_X_stopni);
    PORTD |= (1<<3);

    return;
}

/* Zatrzymanie pojazdu */
static void stop(void){
    PORTD &= ~((1<<2)|(1<<3)|(1<<4));
#ifdef ARDUINO_DB
    Serial.print("Silniki stop\n");
#endif
    return;
}

/* Uruchomienie silniczkow pojazdu */
static void start(void){
    PORTD |= (1<<2)|(1<<3)|(1<<4);
#ifdef ARDUINO_DB
    Serial.print("start()-silniki uruchomione\n");
#endif
    return;
}

/* Odczytuje nr karty RFID przez interfejs szeregowy */
static int odczytaj_karte(const char probkuj)
{
    volatile int data = 0;
    char done = CZYTAJ_DO_SKUTKU;
    if(probkuj)
        done = PROBKUJ;

    do{
        if(Serial.available() > 0){
            data = Serial.read();
            delay(10);
            data = data * 100;
            data = data + Serial.read();
            delay(10);
            Serial.flush();
            done = 1;
            return data;
        }
    }while(!done);

    return -1;          //powrot w przypadku nie udanego PROBKUJ
}

/* Ustawia INF w liscie jego sasiadow co jest
 * jednoznaczne z ustawieniem go jako przeszkody */
static void dodaj_przeszkode(const int przeszkoda){
    int i;
    for(i=0; i < L_KOLUMN; i++){
        Q_P[przeszkoda][i] = INF;
        Q_T[przeszkoda][i] = INF;
    }
}

static void dodaj_cel(const int cel, const int nr){
   C[nr] = cel; 
}

inline static int liczba_celow(void){
    return EEPROM.read(11);
}

inline static int liczba_przeszkod(void){
    return EEPROM.read(1);
}

/* 
 * Oraganizacja pamieci EEPROM:
 * addr 0:      NIE UZYWANY - zalecenie ATMELA
 * addr 1:      przechowuje liczbe przeszkod
 * addr 2-10:   max dziewiec wezlow 'przeszkod'
 * addr 11:     przechowuje liczbe celow
 * addr 12-20:  max dziewiec wezlow 'cel'
 */
#define EEPROM_WRITE_DELAY 5
/* zaisuje tablice 'tab' w pamieci EEPROM w zaleznosci 
 * od bloku ktory jest celem */
static void zapisz_w_eeprom(const int tab[], const enum blok_t blok){
    /* i- nr. bloku w EEPROM, k- licznik tablicy */
    int k;

    /* przesuniecia w pamieci dla przeszkod i celow */
    const int shift_p = 2;
    const int shift_c = 12;

    if(blok == PRZESZKODY){
        for(k=0; tab[k] != -1; k++){
            EEPROM.write(shift_p + k, tab[k]);
            delay(EEPROM_WRITE_DELAY);
        }
        EEPROM.write(1, k);         // liczba wezlow 'przeszkoda'
    } else {                        //CELE
        for(k=0; tab[k] != -1; k++){
            EEPROM.write(shift_c + k, tab[k]);
            delay(EEPROM_WRITE_DELAY);
        }
        EEPROM.write(11, k);        // liczba wezlow 'cel'
    }
}

static void programuj_pamiec(const enum blok_t blok, const int pin_look){
    int wezel_do_zmiany = -1;           //zmienna pomocnicza, -1 gdyby nie przeczytano karty
    int tab_przeszkod_celow[9] =       //tab. przekazywana do zapisana w eeprom
        {-1,-1,-1,-1,-1,-1,-1,-1,-1}; 
    int k = 0;                          //licznik wprowadzonych wezlow do aktualizacji

    while(digitalRead(pin_look)){       //w petli programowania dopuki nie przelaczono dipSwitcha
        wezel_do_zmiany = numer_wezla(odczytaj_karte(PROBKUJ));
        if(wezel_do_zmiany == -1)    
            continue;                   //jesli nie odczytano karty lub nie znaleziono w bazie kart
        /* TODO UWAGA NA TO [k-1] !!!
         * tab_przeszkod_celow[k-1] sprawdzamy po to zeby nie dodac 2 razy tej samiej karty po sobie.
         * Moze to wystapic kiedy przytrzymamy karte w polu odczytu dluzej. */
        if(tab_przeszkod_celow[k-1] != wezel_do_zmiany && k < 9){
            tab_przeszkod_celow[k] = wezel_do_zmiany;
            k++;
        }
    }

    zapisz_w_eeprom(tab_przeszkod_celow, blok);//zapisz w pamieci EEPROM
}

static void zaladuj_eeprom(void){
#ifdef ARDUINO_DB
    Serial.print("Wywolanie zaladuj_eeprom \n");
#endif

    int k = 0;
    int l_p = liczba_przeszkod();
    int l_c = liczba_celow();

#ifdef ARDUINO_DB
    Serial.print("Wartosc kolejno l_p i l_c: ");
    Serial.print(l_p, DEC);
    Serial.print(" ");
    Serial.print(l_c, DEC);
    Serial.print("\n");
#endif

    const int shift_p = 2;      //przesuniecia indexu w pamieci dla przeszkod
    const int shift_c = 12;     //przesuniecie dla celow

    for(k = 0; k < l_p; k++){
        dodaj_przeszkode(EEPROM.read(shift_p + k));
    }

    for(k = 0; k < l_c; k++){
        dodaj_cel(EEPROM.read(shift_c + k), k);
    }
}

int8_t BUFF_W[5] = {INF,INF,INF,INF,INF};
/* Dziala jak dodaj_przeszkode tyle ze, mozna przywrocic usuniety wezel.
 * Poprawia blad zawracania */
static void wyklucz_wezel(const int w_s, uint8_t Q[][L_KOLUMN])
{
    int x = 0;

    while( x < 4 ){
        BUFF_W[x] = Q[w_s][x];  //zapamietane zeby przywrocic sasiadow wezla
        x++;
    }
    BUFF_W[4] = w_s;            //numer wezla usunietego

    dodaj_przeszkode(w_s);
}

/* Przywraca wezel po dzialaniu fun. wyklucz_wezel */
static void przywroc_wezel(uint8_t Q[][L_KOLUMN])
{
    int x = 0;

    while( x < 4 ){
        Q[BUFF_W[4]][x] = BUFF_W[x];
        x++;
    }
}

/* W przypadku jakiegokolwiek bledu zatrzymuje pojazd 
 * i pozostaje w petli */
static void error(void)
{
    stop();
#ifdef ARDUINO_DB
    Serial.print("Wystalpil jakis blad!\n");
    Serial.print("Program w petli, zrestartuj procesor\n");
#endif
    for(;;);
}

/*
 * Sterowanie dla pojazdu nie poruszajacego sie w zadnej
 * siatce. Karty podzielono na grupy rozkazow. Po najechaniu
 * na jedna z nich wykonywany jest odpoiwadajacy jej rozkaz.
 */
static void sterowanie_bezsiatkowe()
{
    while(1){
        POZ.ap = numer_wezla(odczytaj_karte(PROBKUJ));
        if(POZ.ap < 4 && POZ.ap > -1){      //jazda prosto
            start();
            delay(500);
        }
        else if(POZ.ap > 3 && POZ.ap < 8){
            skrecajPrawo(S_90);
            delay(500);
        }
        else if(POZ.ap > 7 && POZ.ap < 12){
            skrecajLewo(S_90);
            delay(500);
        }
        else if(POZ.ap > 11){
            stop();
            delay(500);
        }
        else
            continue;
    }
}

#define SILNIK_LEWY 2
#define SILNIK_PRAWY 3
#define SILNIKI_AKTYWNE 4

void setup(){
    /* inicjalizacja komunikacji szerogowej z modulem RFID */
    Serial.begin(9600);

    /* okreslenie pinow sterujacych silnikami */
    pinMode(SILNIK_LEWY, OUTPUT);
    pinMode(SILNIK_PRAWY, OUTPUT);
    pinMode(SILNIKI_AKTYWNE, OUTPUT);

    /* Piny dipSwitcha sa wejsciami */
    pinMode(A2, INPUT);
    pinMode(A3, INPUT);
    pinMode(A4, INPUT);
    pinMode(A5, INPUT);

    /* Po starcie pojazd nie porusza sie */
    stop();

    /* 
     * Odczyt rodzaju siatki/sterowania. 
     * Obsluga blednego ustawienia dipSwitcha.
     */ 
    if( digitalRead(A2) && digitalRead(A3) ){
        METODA_STEROWANIA = 'N';
    }
    else if( digitalRead(A2) ){
        METODA_STEROWANIA = 'P';
    }
    else if( digitalRead(A3) ){
        METODA_STEROWANIA = 'T';
    }
    else{
        METODA_STEROWANIA = 'E';
    } 

    /* 
     * Sprawdzanie pinow programowania
     */
    /* Programowanie przeszkod */
    if( digitalRead(A4) ){
#ifdef ARDUINO_DB
        Serial.print("Programowenie przeszkod\n");
#endif
        programuj_pamiec(PRZESZKODY, A4);
    }
    /* Programowanie celow */
    if( digitalRead(A5) ){
#ifdef ARDUINO_DB
        Serial.print("Programowanie celow\n");
#endif
        programuj_pamiec(CELE, A5);
    }

#ifdef ARDUINO_DB
    zrzut_eeprom();                 //Wydruk komorek pamieci EEPROM
#endif

#ifdef ARDUINO_DB
    Serial.print("METODA_STEROWANIA=");
    Serial.print(METODA_STEROWANIA, BYTE);
    Serial.print("\n");
    Serial.print("Czekam na wybor karty startowej:\n");
#endif

    /* 
     * Inicjalizajcia struktury z informacja o
     * pozycjach robota. Robot nie wystartuje
     * i nie wygeneruje tablicy sasiedztwa,
     * dopoki nie odczyta pierwszej karty.
     */
    POZ.pp = POZ_NIEZNANA;
    POZ.ap = numer_wezla(odczytaj_karte(CZYTAJ_DO_SKUTKU));
    POZ.np = POZ_NIEZNANA;

    /* 
     * Generowanie tablicy kolejnych wezlow prowadzacych do celu,
     * lub w przypadku sterowania nadarznego: skok do funkcji 
     * obslugujacej to sterowanie. 
     */
    if( METODA_STEROWANIA == 'P'){
        /* Po programowaniu lub nie nalezy wczytac ustawienia z EEPROM */
        zaladuj_eeprom();
        LICZBA_CELOW = liczba_celow();
        /* Usuwanie wezla startowego. Fix: blad proby zawracania */
        wyklucz_wezel(POZ.ap, Q_P);
        /* Generowanie tablicy poprzednikow */
        dijkstra(C[0], Q_P);
        /* Okreslanie pierwszego celu */
        CEL = C[0];
        /* DALEJ NASTEPUJE SKOK DO FUNKCJI loop(); */
#ifdef ARDUINO_DB
        Serial.print("Powrot z: dijkstra(CEL, Q_P)\n");
#endif
    }
    else if( METODA_STEROWANIA == 'T'){
        /* Wszystko tutaj dziala tak jak przy siatce prostokatnej */
        zaladuj_eeprom();
        LICZBA_CELOW = liczba_celow();
        wyklucz_wezel(POZ.ap, Q_T);
        dijkstra(C[0], Q_T);
        CEL = C[0];
#ifdef ARDUINO_DB
        Serial.print("Powrot z: dijkstra(CEL, Q_T)\n");
#endif
    }
    else if( METODA_STEROWANIA == 'N'){
        /* Dalej pojazd wchodzi w petle nieskonczona. Wyjsc z niej moze tylko
         * po resecie sprzetowym. */
        sterowanie_bezsiatkowe();
    }
    else if( METODA_STEROWANIA == 'E'){
        error();
    }

}

/* GLOWNA PETLA PROGRAMU */
void loop(){
    int k = 0;                      //licznik petli
    volatile int DATA = 0;          //odczytany bajt nr karty
   //START
   if( POZ.ap != CEL)
       start();

   while(1){
           /********************************************/
           /*** Odczyt karty znajdujacej sie pod pojazdem*/
           /********************************************/   
       if(Serial.available() > 0){
           DATA = Serial.read();
           delay(10);               //ze wzgledu na bledy w odczycie
           DATA = DATA * 100;
           DATA = DATA + Serial.read();
           delay(10);               //lekkie opoznienia
           Serial.flush();
#ifdef ARDUINO_DB
           Serial.print("Warunek serial.avail > 0\t");
           Serial.print("DATA = ");
           Serial.print(DATA, DEC);
           Serial.print("\n");
#endif
           /*** Obsluga blednego odczytania karty    ***/
           if( (numer_wezla(DATA)) == -1){
#ifdef ARDUINO_DB
               Serial.print("Nierozpoznana karta\n");
#endif
               continue;
           /********************************************/
           /*** Aktualizacjie informacji o pozycjach ***/
           /********************************************/   
           } else {
               POZ.pp = POZ.ap;
               POZ.ap = numer_wezla(DATA);
               POZ.np = P[POZ.ap];
#ifdef ARDUINO_DB
               Serial.print("Jestem w punkcie ");
               Serial.print(POZ.ap, DEC);
               Serial.print(", nastepnym punktem powinien byc wezel: ");
               Serial.print(POZ.np, DEC);
               Serial.print("\n");
#endif
           }
           /*********************************************/
           /*** Obsluga zdarzenia 'dotarcie do celu', ***/
           /*** lub dalsze sterowanie pojazdem        ***/
           /*********************************************/
           if(POZ.ap == CEL){
               k++;                     //Kolejny cel osiagniety
               if(k >= LICZBA_CELOW){    //Wszystkie cele osiagniete ?
                   stop();
                   k--;
               }
               else{
                   CEL = C[k];
                   if(METODA_STEROWANIA == 'P'){
                       przywroc_wezel(Q_P);
                       wyklucz_wezel(POZ.pp, Q_P);
                       dijkstra(C[k], Q_P);
                   }
                   else if(METODA_STEROWANIA == 'T'){
                       przywroc_wezel(Q_T);
                       wyklucz_wezel(POZ.pp, Q_T);
                       dijkstra(C[k], Q_T);
                   }
                   POZ.np = P[POZ.ap]; //Aktualizacja nastepnego wezla
                   kieruj(POZ.pp, POZ.ap, POZ.np);
               }
           }
           /*********************************************/
           /*** Pojazd nie w celu - steruj poj. dalej ***/
           /*********************************************/
           else
               kieruj(POZ.pp, POZ.ap, POZ.np);
       }
   }
}

/* TODO:
 * Jesli nie zdefinniowano celu to wczytywany jest domyslny
 * wezel '0' - to jest ok ale, LICZBA_CELOW jest rowna 0 - dziala
 * poprawnie ale troche nie logiczne.
 *
 */

