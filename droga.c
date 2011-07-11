#include <stdio.h>

#define LICZBA_WEZLOW 16
//maksymalna liczba sasiadow dla siatki prostokatnej
#define MAX_SASIADOW_PROSTOKAT 4
//maksymalna l. sasiadow dla siatki trojkatnej
//#define MAX_SASIADOW_TROJKAT 8

#define NO_DEF 777
#define INF 666

void dijkstra(int, int[][LICZBA_WEZLOW]); //czy mozna uzyc int** ?
void drukuj_wyniki(int*, int*);
int min(int*, int, int*);

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

/* --------------------------------- */
int main(int argc, char* argv[])
{
    if(argc > 1)
        dijkstra(atoi(argv[1]), Q_T);
    else
        dijkstra(0, Q_P);
    return 0;
}

int min(int* tab, int size, int* inS)
{
    int m = INF; 
    int m_index = INF;

    int i = 0;
    for(i; i < size; i++){
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
    int p[LICZBA_WEZLOW] = {0};
    int inS[LICZBA_WEZLOW] = {0}; //tablica informujaca o obecnosci w zbiorze Q

    int i,j = 0;
    //na wszelki wypadek inicializujemy d niezdefiniowana odlegloscia
    for(i=0; i < LICZBA_WEZLOW; i++){
        d[i] = NO_DEF;
    }

    d[cel] = 0; //droga do celu z niego samego jest rowna 0
    p[cel] = cel; //cel nie ma poprzednika

    int h = 0; //pomocnicza
    int v; //aktualnie badany wezel
    for(i = 0; i < LICZBA_WEZLOW; i++){
        v = min(d, LICZBA_WEZLOW, inS); //index najmniejszego wezla
        /* przsuwamy wezel do zbioru S */
        inS[v] = 1;

        /* sasiedzi */
        h = d[v] + 1;
        if(h > d[v]){ //wazny warunek
            for(j=0; j < MAX_SASIADOW_PROSTOKAT; j++){
                if( (Q[v][j] != INF) && (inS[ Q[v][j] ] != 1) ){ // warunki: istnieje sasiad i nie nalezy do S
                    p[Q[v][j]] = v; //poprzenik
                    d[Q[v][j]] = h; //waga + waga poprzednika
                }
            }
        }

    }
   drukuj_wyniki(d,p); 
}

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
