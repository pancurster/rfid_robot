/*
 * Lukasz Panek
 * Sterowanie robotem RFID
 * Galaz optymalizacja
 */

#define PC_COMPILATION
#define ARDUINO_DB

#ifdef PC_COMPILATION
#include <stdio.h>
#include <string.h>
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
int Q_P[][L_KOLUMN] =
{
    {1,4,INF,INF}, {0,2,5,INF}, {1,3,6,INF}, {2,7,INF,INF},
    {0,5,8,INF}, {1,6,9,4}, {2,7,10,5}, {3,11,6,INF},
    {4,9,12,INF}, {5,10,13,8}, {6,11,14,9}, {7,15,10,INF},
    {8,13,INF,INF}, {9,14,12,INF}, {10,15,13,INF}, {11,14,INF,INF}
};

/* Definicja sasiadow poszczegolnych wezlow dla siatki trojkatnej*/
int Q_T[][L_KOLUMN] =
{
    {4,INF,INF,INF}, {4,5,INF,INF}, {5,6,INF,INF}, {6,7,INF,INF},
    {0,1,8,9}, {1,2,9,10}, {2,3,10,11}, {3,11,INF,INF},
    {4,12,INF,INF}, {3,5,12,13}, {5,6,13,14}, {6,7,14,15},
    {8,9,INF,INF}, {9,10,INF,INF}, {10,11,INF,INF}, {11,INF,INF,INF}
};

/* Przechowywanie aktualnego wektora ruch pojazdu */
struct wektor_ruchu{
    int pp;
    int ap;
    int np;
}POZ;

enum blok_t{
    PRZESZKODY = 1,
    CELE = 2
};

/*********** HARDWARE *************/
/* Zwiazane z funkcja odczytaj_karte */
#define CZYTAJ_DO_SKUTKU 0
#define PROBKUJ 1

/* SILNIKI */
#define CZAS_SKRETU_90_STOPNI 2000

/* STAN POJAZDU */
int CEL = 0;                    //nr wezla do ktorego pojazd jedzie
int LICZBA_CELOW = 0;           //liczba zdefinniowanych celow
char METODA_STEROWANIA = 'P';   //info o typie siatki lub jej braku

/* Przykladowe nr kart, posortowane! */
int ID_KART[LICZBA_WEZLOW] = 
{ 
    4850,4867,4967,5054,
    5065,5070,5166,5255,
    5269,5348,5349,5450,
    5452,5555,5566,5648
};
/***** KONIEC DEKLARACJI **********/


static void dijkstra(int, int[][L_KOLUMN]);
static void drukuj_wyniki(int*, int*);
static int minimum(int*, int, int*);
static int kierunek(int, int);
static void kieruj(int, int, int);
static int znajdz_nr_wezla(int, int, int);
static int numer_wezla(int);
static void wyklucz_wezel(int w_s, int Q[][L_KOLUMN]);
static void przywroc_wezel(int Q[][L_KOLUMN]);

static void erase_eeprom(enum blok_t );
static void zapisz_w_eeprom(int*, enum blok_t);
static void programuj_pamiec(enum blok_t, int);
static void zaladuj_eeprom();
inline static int liczba_celow();

static void dodaj_przeszkode(int);
static void dodaj_cel(int, int);
static int odczytaj_karte(char );
static void stop(void);
static void start(void);
static void skrecajLewo(void);
static void skrecajPrawo(void);
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
static int numer_wezla(int id)
{
    return znajdz_nr_wezla(id, 0, LICZBA_WEZLOW-1);
}

/* Przeszukiwanie binarne */
static int znajdz_nr_wezla(int id, int poczatek, int koniec)
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

/* Okresla kierunek na podstawie punktu poczatkowego
 * i punktu nastepnego */
static int kierunek(int ap, int np)
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
            case (-L_KOLUMN):
                return NE;
            case (L_KOLUMN):
                return SE;
            case (L_KOLUMN-1):
                return SW;
            case (-L_KOLUMN-1):
                return NW;
            default:
                return -1;
        }
    }
    return -1;
}

/* Decyduje o wywalaniu skretu w ktoras ze stron lub jezdzie na wprost*/
static void kieruj(int pp, int ap, int np)
{
    int orientacja = kierunek(pp, ap);
    int nastepny_kierunek = kierunek(ap, np);

    if( orientacja == nastepny_kierunek
          || orientacja == -1
          || nastepny_kierunek == -1 )
    {
#ifdef ARDUINO_DB
        Serial.print("kieruj:jazda prosto\n");
#endif
        return;
    }

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
        Serial.print("kieruj:skrecam w lewo\n");
#endif
        skrecajLewo();
    }

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
            Serial.print("kieruj:skrecam w prawo\n");
#endif
            skrecajPrawo();
    }

    return;
}

/* Wyszukuje element o nakrotszej drodze do celu */
static int minimum(int* d, int size, int* inS)
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

/* TODO: dla trojkata nie wybiera najkrotszych sciezek.
 * To jest chyba problem z wagami ?
 * Algorytm dziala jakby z punktu widzenia siatki prostokatnej */
static void dijkstra(int cel, int Q[][L_KOLUMN]){
#ifdef ARDUINO_DB
    Serial.print("Wywolanie: dijkstra(CEL, Q_?)\n");
#endif
    int d[LICZBA_WEZLOW];
    int inS[LICZBA_WEZLOW] = {0}; //tablica informujaca o obecnosci w zbiorze Q

    int i,j = 0;
    //na wszelki wypadek inicializujemy d niezdefiniowana odlegloscia (nie NULL!)
    for(i=0; i < LICZBA_WEZLOW; i++){
        d[i] = NO_DEF;
    }

    d[cel] = 0;     //droga do celu z niego samego jest rowna 0
    P[cel] = cel;   //cel nie ma poprzednika

    int h = 0;      //pomocnicza
    int v;          //aktualnie badany wezel
    for(i = 0; i < LICZBA_WEZLOW; i++){
        v = minimum(d, LICZBA_WEZLOW, inS); //index najmniejszego wezla
       /* przsuwamy wezel do zbioru S */
        inS[v] = 1;

        /* sasiedzi */
        h = d[v] + 1;
        if(h > d[v]){ //wazny warunek
            for(j=0; j < MAX_SASIADOW_PROSTOKAT; j++){
                if( (Q[v][j] != INF) && (inS[ Q[v][j] ] != 1) ){ 
                    P[Q[v][j]] = v; //poprzenik
                    d[Q[v][j]] = h; //waga + waga poprzednika
                }
            }
       }

    }
#ifdef PC_COMPILATION
    drukuj_wyniki(d,P);
#endif
}

#ifdef PC_COMPILATION
static void drukuj_wyniki(int* d, int* p){
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

/* Obsluga sretu w lewo */
static void skrecajLewo(void){
#ifdef ARDUINO_DB
    Serial.print("Wywolanie: skrecajLewo()\n");
#endif
    PORTD &= ~(1<<2);
    delay(CZAS_SKRETU_90_STOPNI);
    PORTD |= (1<<2);

    return;
}

/* Obsluga skretu w prawo */
static void skrecajPrawo(void){
#ifdef ARDUINO_DB
    Serial.print("Wywolanie: skrecajPrawo()\n");
#endif
    PORTD &= ~(1<<3);
    delay(CZAS_SKRETU_90_STOPNI);
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

/* TODO UWAGA: ta funkcje trzeba przetestwac pod katem 
 * narzutu czasowego, moze powinna byc inline ? */
static int odczytaj_karte(char probkuj)
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

/* Czysci pamiec eeprom. Bloki przeszkod lub celow.
 * addr 0:      NIE UZYWANY - zalecenie ATMELA
 * addr 1:      przechowuje liczbe przeszkod
 * addr 2-10:   max dziewiec wezlow 'przeszkod'
 * addr 11:     przechowuje liczbe celow
 * addr 12-20:  max dziewiec wezlow 'cel'
 */
static void erase_eeprom(enum blok_t blok){
    int i;

    if(blok == PRZESZKODY){
        for(i=1; i <= 10; i++)
            EEPROM.write(i, NO_DEF);
    } else {                            //CELE
        for(i=11; i <=20; i++)
            EEPROM.write(i, NO_DEF);
    }
    return;
}

/* Ustawia INF w sasiadach danego wezla co jest 
 * jednoznaczne z ustawieniem go jako przeszkody */
static void dodaj_przeszkode(int przeszkoda){
    int i;
    for(i=0; i < L_KOLUMN; i++){
        Q_P[przeszkoda][i] = INF;
        Q_T[przeszkoda][i] = INF;
    }
}

static void dodaj_cel(int cel, int nr){
   C[nr] = cel; 
}

inline static int liczba_celow(){
    return EEPROM.read(11);
}

/* zaisuje tablice 'tab' w pamieci EEPROM w zaleznosci 
 * od bloku ktory jest celem */
static void zapisz_w_eeprom(int tab[], enum blok_t blok){
    /* i- nr. bloku w EEPROM, k- licznik tablicy */
    int i,k;

    /* przesuniecia w pamieci dla przeszkod i celow */
    const int shift_p = 2;
    const int shift_c = 12;

    if(blok == PRZESZKODY){
        for(k=0; tab[k] != -1; k++){
            EEPROM.write(shift_p + k, tab[k]);
        }
        EEPROM.write(1, k);         // liczba wezlow 'przeszkoda'
    } else {                        //CELE
        for(k=0; tab[k] != -1; k++){
            EEPROM.write(shift_c + k, tab[k]);
        }
        EEPROM.write(11, k);        // liczba wezlow 'cel'
    }
}

static void programuj_pamiec(enum blok_t blok, int pin_look){
    int wezel_do_zmiany = -1;           //zmienna pomocnicza, -1 gdyby nie przeczytano karty
    int tab_przeszkod_celow[9] =       //tab. przekazywana do zapisana w eeprom
        {-1,-1,-1,-1,-1,-1,-1,-1,-1}; 
    int k = 0;                          //licznik wprowadzonych wezlow do aktualizacji

    erase_eeprom(blok);                 //czyszczenie calego eeprom przeszkod albo celow
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

static void zaladuj_eeprom(){
    int k = 0;                  //licznik petli
    int l_p = EEPROM.read(1);   //liczba przeszkod
    int l_c = EEPROM.read(11);  //liczba celow
    const int shift_p = 2;      //przesuniecia indexu w pamieci dla przeszkod
    const int shift_c = 12;     //przesuniecie dla celow

    for(k = 0; k < l_p; k++){
        dodaj_przeszkode(EEPROM.read(shift_p + k));
    }

    for(k = 0; k < l_c; k++){
        dodaj_cel(EEPROM.read(shift_c + k), k);
    }
}

int BUFF_W[5] = {INF,INF,INF,INF,INF};
static void wyklucz_wezel(int w_s, int Q[][L_KOLUMN])
{
    int x = 0;

    while( x < 4 ){
        BUFF_W[x] = Q[w_s][x];  //zapamietane zeby przywrocic sasiadow wezla
        x++;
    }
    BUFF_W[4] = w_s;            //numer wezla usunietego

    dodaj_przeszkode(w_s);
}

static void przywroc_wezel(int Q[][L_KOLUMN])
{
    int x = 0;

    while( x < 4 ){
        Q[BUFF_W[4]][x] = BUFF_W[x];
        x++;
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

    /* Odczyt rodzaju siatki/sterowania, lub celu */
    if( digitalRead(A2) ){
        METODA_STEROWANIA = 'P';
    }
    else if( digitalRead(A3) ){
        METODA_STEROWANIA = 'T';
    } else {
        METODA_STEROWANIA = 'N';
    }

    /* Programowanie przeszkod */
    if( digitalRead(A4) ){
        programuj_pamiec(PRZESZKODY, A4);
    }
    /* Programowanie celow */
    if( digitalRead(A5) ){
        programuj_pamiec(CELE, A5);
    }

    /* Po programowaniu lub nie nalezy wczytac ustawienia z EEPROM */
    zaladuj_eeprom();
    LICZBA_CELOW = liczba_celow();

#ifdef ARDUINO_DB
    Serial.print("METODA_STEROWANIA=");
    Serial.print(METODA_STEROWANIA, BYTE);
    Serial.print("\n");
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

    /* Generowanie tablicy kolejnych wezlow prowadzacych do celu,
     * lub w przypadku sterowania nadarznego: brak akcji */
    if( METODA_STEROWANIA == 'P'){
        wyklucz_wezel(POZ.ap, Q_P);
        dijkstra(C[0], Q_P);
        CEL = C[0];
#ifdef ARDUINO_DB
        Serial.print("Powrot z: dijkstra(CEL, Q_P)\n");
#endif
    }
    else if( METODA_STEROWANIA == 'T'){
        wyklucz_wezel(POZ.ap, Q_T);
        dijkstra(C[0], Q_T);
        CEL = C[0];
#ifdef ARDUINO_DB
        Serial.print("Powrot z: dijkstra(CEL, Q_T)\n");
#endif
    }

}

/* GLOWNA PETLA PROGRAMU */
void loop(){
    int k = 0;                      //licznik petli
    volatile int DATA = 0;          //odczytany bajt nr karty
   //START
   if( POZ.ap != CEL)
       start();

   while(POZ.ap != CEL){            //TODO to powinna byc petla nieskonczona
           /********************************************/
           /*** Odczyt karty znajdujacej sie pod pojazdem
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
               if(k >= LICZBA_CELOW)    //Wszystkie cele osiagniete ?
                   stop();
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

