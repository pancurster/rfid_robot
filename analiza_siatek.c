/*
 * Program do wyciagania statystyk z konstrukcji siatek
 * do programu droga.exe
 */

#include <stdio.h>
#include <math.h>
#include <malloc.h>
#include <stdlib.h>
#include <time.h>

#define INF 249
#define NO_DEF 254
#define MAX_SASIADOW_PROSTOKAT 4

int L_KOLUMN;
int LICZBA_WEZLOW;

static void dijkstra(const int cel, int** Q, int liczba_wezlow);
static int minimum(const int* d, const int size, const int* inS);
static void generuj_q(const int l_wezlow, int** q);
static void drukuj_wyniki(const int* d, const int* p, int l_wezlow);
static void srednia_d_d(int* d, int l_elem);
static void dodaj_przeszkode(int** q, const int przeszkoda);

int main(int argc, char** argv)
{
    /* Liczba wezlow podawana z lini polecen */
    LICZBA_WEZLOW = atoi(argv[1]);

    /* Liczba wezlow powinna byc potega liczby calkowitej */
    L_KOLUMN = sqrt(LICZBA_WEZLOW);

    /* Alokacja pamieci dla tablicy Q */
    int** q = (int**)malloc(LICZBA_WEZLOW * sizeof(int));
    int i = 0;
    for(i; i < LICZBA_WEZLOW; i++){
        q[i] = (int*)malloc(4 * sizeof(int));
    }

    /* Generuje tablice sasiedztwa dla wezlow */
    generuj_q(LICZBA_WEZLOW, q);

    /* dodawanie randomowych przeszkod jesli zadano */
    int s_rand;
    int l_przeszkod;
    int k = 0;
    volatile int przeszkoda;
    if( argc > 2 ){
        s_rand = time(NULL);
        srand(s_rand);
        rand();

        l_przeszkod = atoi(argv[2]);
        for(k=0; k < l_przeszkod; k++){
            przeszkoda = rand() % LICZBA_WEZLOW;
            dodaj_przeszkode(q, przeszkoda);
        }

    }

    /* Obiczanie najkrotszych drog i liczby skokow */
    dijkstra(0, q, LICZBA_WEZLOW);

    return 0;
}

static void generuj_q(const int l_wezlow, int** q)
{
    int x;
    int s;
    for(x=0; x < l_wezlow; x++){
        s = x - 1;
        if( s > -1 )
            q[x][0] = s;
        else
            q[x][0] = INF;

        s = x + 1;
        if( s < l_wezlow)
            q[x][1] = s;
        else
            q[x][1] = INF;
         
        s = x - L_KOLUMN;
        if( s > -1 )
            q[x][2] = s;
        else
            q[x][2] = INF;

        s = x + L_KOLUMN;
        if( s < l_wezlow)
            q[x][3] = s;
        else
            q[x][3] = INF;
    }
}

/* 
 * Buduje tablice poprzednikow (P[]) pod katem najkrotszej drogi 
 * do wezla 'cel'. Dziala dla siatki prostokatnej i trojkatnej.
 */
static void dijkstra(const int cel, int** Q, int liczba_wezlow)
{
    int* d = (int*)malloc(liczba_wezlow * sizeof(int));
    int* inS = (int*)malloc(liczba_wezlow * sizeof(int));
    int* P = (int*)malloc(liczba_wezlow * sizeof(int));

    int i,j = 0;

    for(i=0; i < liczba_wezlow; i++){                       //inicializujemy d maksymalna odlegloscia (nie NULL!)
        d[i] = NO_DEF;
    }

    d[cel] = 0;                                             //droga do celu z niego samego jest rowna 0
    P[cel] = cel;                                           //cel nie ma poprzednika

    int h = 0;                                              //pomocnicza do obliczania dlugosci drogi do nastepnikow
    int v;                                                  //aktualnie badany wezel
    for(i = 0; i < liczba_wezlow; i++){                     //AKTUALNIE ROZPATRYWANY WEZEL
        v = minimum(d, liczba_wezlow, inS);                 //index najmniejszego wezla

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
    drukuj_wyniki(d, P, liczba_wezlow);
    srednia_d_d(d, liczba_wezlow);
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

static void drukuj_wyniki(const int* d, const int* p, int l_wezlow){
    int x = 0;
    printf("Q:");
    for(x=0; x < l_wezlow; x++){
        if(x > 9)
            printf("%i ", x);
        else
            printf("%i  ", x);
    }
    printf("\n");
    
    printf("d:");
    for(x=0; x < l_wezlow; x++){
        if(d[x] > 9)
            printf("%i ", d[x]);
        else
            printf("%i  ", d[x]);
    }
    printf("\n");

    printf("P:");
    for(x=0; x < l_wezlow; x++){
        if(p[x] > 9)
            printf("%i ", p[x]);
        else
            printf("%i  ", p[x]);
    }
    printf("\n");
}

static void srednia_d_d(int* d, int l_elem)
{
    unsigned int sum = 0;

    int j = 0;
    for(j; j < l_elem; j++){
        sum += d[j];
    }

    float srednia = sum / l_elem;
    printf("Srednia droga dojscja do celu wynosi: %f\n", srednia);

}

/* Ustawia INF w sasiadach danego wezla co jest 
 * jednoznaczne z ustawieniem go jako przeszkody */
static void dodaj_przeszkode(int** q, const int przeszkoda){
    int i;
    printf("Wezel %i jest przeszkoda\n", przeszkoda);
    for(i=0; i < MAX_SASIADOW_PROSTOKAT; i++){
        q[przeszkoda][i] = INF;
    }
}
