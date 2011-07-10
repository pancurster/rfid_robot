#include <stdio.h>

void dijkstra(int);
void drukuj_wyniki(int*, int*);
int min(int*, int, int*);

#define LICZBA_WEZLOW 16

//maksymalna liczba sasiadow dla siatki prostokatnej
#define MAX_SASIADOW_PROSTOKAT 4
//maksymalna l. sasiadow dla siatki trojkatnej
#define MAX_SASIADOW_TROJKAT 8

#define NIESKACZONOSC 555

int Q[][LICZBA_WEZLOW] =
{
    {1,4,666,666}, {0,2,5,666}, {1,3,6,666}, {2,7,666,666},
    {0,5,8,666}, {1,6,9,4}, {2,7,10,5}, {3,11,6,666},
    {4,9,12,666}, {5,10,13,8}, {6,11,14,9}, {7,15,10,666},
    {8,13,666,666}, {9,14,12,666}, {10,15,13,666}, {11,14,666,666}
};

/* --------------------------------- */
int main(int argc, char* argv[])
{
    if(argc > 1)
        dijkstra(atoi(argv[1]));
    else
        dijkstra(0);
    return 0;
}

int min(int* tab, int size, int* inS)
{
    int m = 666; //pseudo nieskonczonosc :)
    int m_index = 666;

    int i = 0;
    for(i; i < size; i++){
        if( (tab[i] < m) && (inS[i] != 1) ){
            m = tab[i];
            m_index = i;
        }
    }
    return m_index;
}

void dijkstra(int cel){
    int d[LICZBA_WEZLOW];
    int p[LICZBA_WEZLOW] = {0};
    int inS[LICZBA_WEZLOW] = {0}; //tablica informujaca o obecnosci w zbiorze Q

    int i,j = 0;
    for(i=0; i < LICZBA_WEZLOW; i++){
        d[i] = NIESKACZONOSC; // 555 oznacza nieskonczonosc
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
                if( (Q[v][j] != 666) && (inS[ Q[v][j] ] != 1) ){ // warunki: istnieje sasiad i nie nalezy do S
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
