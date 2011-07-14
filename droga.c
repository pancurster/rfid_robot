/*
 * Lukasz Panek
 * Sterowanie robotem RFID
 */

#define PC_DEBUG

#ifdef PC_DEBUG
#include <stdio.h>
#endif

/****** DIJKSTRA ***************/

#define LICZBA_WEZLOW 16
#define MAX_SASIADOW_PROSTOKAT 4
#define L_KOLUMN 4
#define NO_DEF 777
#define INF 666

/* Globalna tablica poprzednikow wyliczana w algorytmie dijkstry.
 * Wedlug niej laczymy wezly tworzac scierzke do wezla 'cel'*/
int P[LICZBA_WEZLOW] = {0};

int Q_P[][LICZBA_WEZLOW] =
{
    {1,4,INF,INF}, {0,2,5,INF}, {1,3,6,INF}, {2,7,INF,INF},
    {0,5,8,INF}, {1,6,9,4}, {2,7,10,5}, {3,11,6,INF},
    {4,9,12,INF}, {5,10,13,8}, {6,11,14,9}, {7,15,10,INF},
    {8,13,INF,INF}, {9,14,12,INF}, {10,15,13,INF}, {11,14,INF,INF}
};

int Q_T[][LICZBA_WEZLOW] =
{
    {4,INF,INF,INF}, {4,5,INF,INF}, {5,6,INF,INF}, {6,7,INF,INF},
    {0,1,8,9}, {1,2,9,10}, {2,3,10,11}, {3,11,INF,INF},
    {4,12,INF,INF}, {3,5,12,13}, {5,6,13,14}, {6,7,14,15},
    {8,9,INF,INF}, {9,10,INF,INF}, {10,11,INF,INF}, {11,INF,INF,INF}
};

struct wektor_ruchu{
    int pp;
    int ap;
    int np;
}POZ;

/* Kierunki */
#define N 1
#define E 2
#define S 4
#define W 7

/*********** HARDWARE *************/
    
const uint8_t SILNIK_LEWY = 2;
const uint8_t SILNIK_PRAWY = 3;
const uint8_t SILNIKI_AKTYWNE = 4;

/* Zwiazane z funkcja odczytaj_karte */
#define CZYTAJ_DO_SKUTKU 0
#define PROBKUJ 1

/* SILNIKI */
#define START HIGH
#define STOP LOW
#define SILNIKI_LEWY_PRAWY 23
#define CZAS_SKRETU_90_STOPNI 2000

volatile int DATA = 0;
int STARTSTOP = 0;

/* Przykladowe nr kart, posortowane! */
int ID_KART[LICZBA_WEZLOW] = 
{ 
    15,16,23,25,
    28,34,36,41,
    42,47,49,58,
    68,73,78,93
};
/***** KONIEC DEKLARACJI **********/


void dijkstra(int, int[][LICZBA_WEZLOW]);
void drukuj_wyniki(int*, int*);
int minimum(int*, int, int*);
int kierunek(int, int);
void decyzja(int, int, int);
int znajdz_nr_wezla(int, int, int);
int numer_wezla(int);

void stop();
void skrecajLewo();
void skrecajPrawo();
void motorInterface(uint8_t, uint8_t);
void setup();
void loop();

#ifdef PC_DEBUG
int main(int argc, char* argv[])
{
    /*
    if(argc > 1)
        dijkstra(atoi(argv[1]), Q_T);
    else
        dijkstra(0, Q_P);
        */
    printf("Index tej kartu to: %i\n", znajdz_wezel(atoi(argv[1]),0,15));
    return 0;
}
#endif

/* uproszczenie wywolania fun. znajdz_nr_wezla.
 * Wyszukuje w fizycznych numerach kart RFID index danej karty */
int numer_wezla(int id)
{
    return znajdz_nr_wezla(id, 0, LICZBA_WEZLOW-1);
}

/* Przeszukiwanie binarne */
int znajdz_nr_wezla(int id, int poczatek, int koniec)
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
}

/* Okresla kierunek na podstawie punktu poczatkowego
 * i punktu nastepnego */
int kierunek(int ap, int np)
{
    int k = np - ap;

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
}

void decyzja(int pp, int ap, int np)
{
    int orientacja = kierunek(pp, ap);
    int nastepny_kierunek = kierunek(ap, np);

    if( orientacja == nastepny_kierunek ){
        return;
    }
    else if( orientacja == N ){
        if( nastepny_kierunek == W ){
            //skrecajLewo();
        }
        else if( nastepny_kierunek == E ){
            //skrecajPrawo();
        }
    }
    else if( orientacja == E ){
        if( nastepny_kierunek == N ){
            //skrecajLewo();
        }
        else if( nastepny_kierunek == S ){
            //skrecajPrawo();
        }
    }
    else if( orientacja == S ){
        if( nastepny_kierunek == W ){
            //skrecajLewo();
        }
        else if( nastepny_kierunek == E ){
            //skrecajPrawo();
        }
    }
    else if( orientacja == W ){
        if( nastepny_kierunek == S ){
            //skrecajLewo();
        }
        else if( nastepny_kierunek == N ){
            //skrecajPrawo();
        }
    }

    return;
}

int minimum(int* tab, int size, int* inS)
{
    int m = INF; 
    int m_index = INF;

    int i = 0;
    for(; i < size; i++){
        //warunek: jest mniejsze od aktualnego m i nie znajduje sie w przerobionych wezlach
        if( (tab[i] < m) && (inS[i] != 1) ){
            m = tab[i];
            m_index = i;
        }
    }
    return m_index;
}

void dijkstra(int cel, int Q[][LICZBA_WEZLOW]){
    int d[LICZBA_WEZLOW];
    int inS[LICZBA_WEZLOW] = {0}; //tablica informujaca o obecnosci w zbiorze Q

    int i,j = 0;
    //na wszelki wypadek inicializujemy d niezdefiniowana odlegloscia
    for(i=0; i < LICZBA_WEZLOW; i++){
        d[i] = NO_DEF;
    }

    d[cel] = 0; //droga do celu z niego samego jest rowna 0
    P[cel] = cel; //cel nie ma poprzednika

    int h = 0; //pomocnicza
    int v; //aktualnie badany wezel
    for(i = 0; i < LICZBA_WEZLOW; i++){
        v = minimum(d, LICZBA_WEZLOW, inS); //index najmniejszego wezla
        /* przsuwamy wezel do zbioru S */
        inS[v] = 1;

        /* sasiedzi */
        h = d[v] + 1;
        if(h > d[v]){ //wazny warunek
            for(j=0; j < MAX_SASIADOW_PROSTOKAT; j++){
                if( (Q[v][j] != INF) && (inS[ Q[v][j] ] != 1) ){ // warunki: istnieje sasiad i nie nalezy do S
                    P[Q[v][j]] = v; //poprzenik
                    d[Q[v][j]] = h; //waga + waga poprzednika
                }
            }
        }

    }
#ifdef PC_DEBUG
    drukuj_wyniki(d,P);
#endif
}

#ifdef PC_DEBUG
void drukuj_wyniki(int* d, int* p){
    int x = 0;
    for(x=0; x < LICZBA_WEZLOW; x++){
        printf("%i ", x);
    }
    printf("\n");

    for(x=0; x < LICZBA_WEZLOW; x++){
        printf("%i ", d[x]);
    }
    
    printf("\n");

    for(x=0; x < LICZBA_WEZLOW; x++){
        printf("%i ", p[x]);
    }
}
#endif

/*
 * Kod dotyczacy arduino dolaczany tylko w wersji finalnej
 * lub w kompilacji w srodowisku arduino
 */
#ifndef PC_DEBUG

/*
 * motor: SILNIK_LEWY, SILNIK_PRAWY, SILNIK_LEWY_PRAWY
 * command: START, STOP
 */
void motorInterface(uint8_t motor, uint8_t command){
    //wykonaj polecenia na obu silnikach
    if(motor == SILNIKI_LEWY_PRAWY){
        digitalWrite(SILNIK_LEWY, command);
        digitalWrite(SILNIK_PRAWY, command);
    }
    //wykonaj polecenie na prawym lub lewym silniku
    else{
        digitalWrite(motor, command);
    }
}
/*
 * Obsluga sretu w lewo
 */
void skrecajLewo(){
    motorInterface(SILNIK_LEWY, STOP);
    delay(CZAS_SKRETU_90_STOPNI);
    motorInterface(SILNIK_LEWY, START);
}

/*
 * Obsluga skretu w prawo
 */
void skrecajPrawo(){
    motorInterface(SILNIK_PRAWY, STOP);
    delay(CZAS_SKRETU_90_STOPNI);
    motorInterface(SILNIK_PRAWY, START);
}

void stop(){
    if(STARTSTOP == 1){
        digitalWrite(SILNIKI_AKTYWNE, LOW);
        STARTSTOP = 0;
    }
    else{
        digitalWrite(SILNIKI_AKTYWNE, HIGH);
        /* HACK i inne cuda -------------------------
        motorInterface(SILNIKI_LEWY_PRAWY, STOP);
        delay(CZAS_SKRETU_90_STOPNI);
        motorInterface(SILNIKI_LEWY_PRAWY, START);
        * KONIEC HACKA -----------------------------*/
        STARTSTOP = 1;
    }
}

/* TODO UWAGA: ta funkcje trzeba przetestwac pod katem 
 * narzutu czasowego */
int odczytaj_karte(char probkuj)
{
    volatile int data = 0;
    char done = 0;
    if(probkuj)
        done = 1;

    do{
        if(Serial.available() > 0){
            data = Serial.read();
            Serial.flush();
            done = 1;
            return data;
        }
    }while(!done);
}

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

    /* Numer wezla 'celu'.
     * Jesli nie jest ustawiany to cel = wezel nr 0 */
    int CEL = 0;

    char METODA_STEROWANIA = 'N';

    /* Odczyt rodzaju siatki/sterowania, lub celu */
    if( digitalRead(A2) ){
        METODA_STEROWANIA = 'P';
    }
    else if( digitalRead(A3) ){
        METODA_STEROWANIA = 'T';
    }
    else if( digitalRead(A4) ){
        METODA_STEROWANIA = 'N';
    }
    else if( digitalRead(A5) ){
        CEL = numer_wezla(odczytaj_karte(CZYTAJ_DO_SKUTKU));
    }
    else{
        /* w przypadku gdyby zaden przelacznik nie byl przelaczony */
        METODA_STEROWANIA = 'N';
    }

    /* Generowanie tablicy kolejnych wezlow prowadzacych do celu,
     * lub w przypadku sterowania nadarznego: brak akcji */
    if( METODA_STEROWANIA == 'P'){
        dijkstra(CEL, Q_P);
    }
    else if( METODA_STEROWANIA == 'T'){
        dijkstra(CEL, Q_T);
    }

    /* Po starcie pojazd nie porusza sie */
    digitalWrite(SILNIKI_AKTYWNE, LOW);

    /* Inicjalizajcia struktury z informacja o
     * pozycjach robota */
    POZ.pp = -1;
    if( POZ.ap = numer_wezla(odczytaj_karte(0)) == -1)
        //TODO tutaj powinna byc obsluga bledu
    POZ.np = P[POZ.ap];

}

/*
 * GLOWNA PETLA PROGRAMU
 */
void loop(){
   if(Serial.available() > 0){
       //funkcja sprawdzajaca dzialanie przypisane do karty
       DATA = Serial.read();
       if(DATA == 56) { skrecajPrawo(); }
       if(DATA == 55) { skrecajLewo(); } 
       if(DATA == 49) { stop(); }
       Serial.flush();
   }

}

#endif

/* TODO:
 * * Ciagle po wlaczenie i wylaczeniu karta, jedno z kol 
 * * wchodzi w tryb skrecania.
 *
 * * funkcja decyzja wyglada kiepsko, jakis wzor na to 
 * * znajdz :)
 */

