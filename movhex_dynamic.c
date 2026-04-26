#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#define CACHE 8191
#define MAX(X,Y) ((X) < (Y) ? (Y) : (X))
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))

//prototipi
void init(int arg1, int arg2);
void change_cost(int arg1, int arg2, int arg3, int arg4);
void toggle_air_route(int arg1, int arg2, int arg3, int arg4);
void travel_cost(int arg1, int arg2, int arg3, int arg4);

//STRUTTURE
/*typedef  struct 
{
    int xdst;
    int ydst;
    //int cost; //costo della rotta aerea
}AirRoute;
*/


typedef struct Hex
{
    int x; 
    int y;
    struct Hex **air_routes;  //costo
    __int8_t land_cost;
    /*int vicinox[6];
    int vicinoy[6]; */
    __int8_t air_count;
}Hex;

typedef struct
{
    int righe;
    int colonne;
    Hex ***griglia;  //matrice dinamica (2D)
}Map;

typedef struct
{
    Hex* h;    // puntatore a esagono
    int d; //dist da sorgente
} Node;

typedef struct 
{
    int x, y;
    int d;
}Node2;

typedef struct      //min-heap per dijkstra (classico)
{
    Node *buf;
    short unsigned int size;
    short unsigned int c;
} MinHeap;

typedef struct      
{
    Node2 *buf;
    short unsigned int size;
    short unsigned int c;
} MinHeap2;

typedef struct
{
    int da;    
    int a;
    int costo;
    __uint8_t valid; //byte di valid (generazioni<=255)
} Cella; //per cache

//VARIABILI GLOBALI
Map *m = NULL;

//set di inizializzazione per i vicini
/*static const int dy_p[6] = {0, 1, 1, 0, -1, -1};
static const int dx_p[6] = {1, 0, -1, -1, -1, 0};
static const int dy_d[6] = {-1, 0, 1, 1, 0, -1};
static const int dx_d[6] = {0, 1, 1, 0, -1, 1};
*/

const int KSIZE = 50;
const int I = 200000000;        //inifinito (200M)
Cella cache_glob[CACHE];        //cache per travel_cost
unsigned char gen_cache = 0;    //generazione cache, per invalidare
int n;  
int *dist;
bool *vis;


//FUNZIONI
/*
void debug_map(void) //debug generale
{
    if (m == NULL || m->griglia == NULL) {
        printf("La mappa non è inizializzata.\n");
        return;
    }

    printf("\n======= STATO ATTUALE DELLA MAPPA =======\n");
    printf("Dimensione: %d righe x %d colonne\n\n", m->righe, m->colonne);

    for (int y = 0; y < m->righe; y++) {
        for (int x = 0; x < m->colonne; x++) {
            Hex *h = &m->griglia[y][x];
            printf("Esagono (%d,%d):\n", h->x, h->y);
            printf("  - Costo uscita via terra: %d\n", h->land_cost);

            printf("  - Rotte aeree (%d):\n", h->air_count);
            for (int i = 0; i < h->air_count; i++) {
                AirRoute *r = &h->air_routes[i];
                printf("    -> verso (%d,%d), costo = %u\n", r->xdst, r->ydst, r->cost);
            }
            printf("\n");
        }
    }

    printf("======= FINE STATO MAPPA =======\n\n");
}
*/

Hex* alloca_hex(int x, int y)
{

    if ( m->griglia[y][x] != NULL) //esagono già esistente
    {
        return m->griglia[y][x];
    }
    Hex *h = malloc(sizeof(Hex));
    h->x = x;
    h->y = y;
    h->land_cost = 1;
    h->air_count = 0;
    h->air_routes = NULL;

    /*for (int k = 0; k < 6; k++) 
    {
        int vi, vj;
        if (y % 2 == 0) { // riga pari
            vi = y + dy_p[k];
            vj = x + dx_p[k];
        } 
        else 
        {          // riga dispari
            vi = y + dy_d[k];
            vj = x + dx_d[k];
        }
        if (vi >= 0 && vi < m->righe && vj >= 0 && vj < m->colonne) 
        {
            h->vicinoy[k] = vi;
            h->vicinox[k] = vj;
        }
        else 
        {
            h->vicinoy[k] = -1;
            h->vicinox[k] = -1;
        }
    } */
    
    m->griglia[y][x] = h; //assegna alla mappa
    return h;
}

void dealloca_Hex(Hex *h)
{
    if (h != NULL)
    {
        free(h);
    }
}

void distruggi_m()
{
    if (m != NULL)
    {
        for (int i = m->righe -1; i >= 0; i--)
        {
            if (m->griglia[i]) 
            {
                for (int j = m->colonne -1; j >=0; j--)
                {
                    free(m->griglia[i][j]); // OK anche se NULL
                }
                free(m->griglia[i]);
            }
        }
        free(m->griglia);
        free(m);
        m = NULL;
    }
    return;
}

int disth(int i1, int j1, int i2, int j2) //adattamento esagonale --> coordinate cubiche
{
    //se gli esagoni coincidono
    if (i1 == i2 && j1 == j2) return 0;

    int z1 = i1;
    int z2 = i2;
    int x2 = j2 - (i2 - (i2 % 2))/2 ;
    int x1 = j1 - (i1 - (i1 % 2))/2 ;  
    int y1 = -x1 - z1;  
    int y2 = -x2 - z2;
    int max = abs(x1-x2);
    if (abs(z1-z2) > max) max = abs(z1-z2);

    if (abs(y1-y2) > max) max = abs(y1-y2);
    return max;
}       

bool coordinate_vic_old(int x, int y, int k, int *vx, int *vy)  //calcola coordinate e fai check validità
{
    const int dx_p[6] = {-1, 0, 1, 0, -1, -1};
    const int dy_p[6] = {-1, -1, 0, 1, 1, 0};
    const int dx_d[6]  = {0, 1, 1, 1, 0, -1};
    const int dy_d[6]  = {-1, -1, 0, 1, 1, 0};

    const int *dx;
    if (y % 2 == 0)     //pari
    {
        dx = dx_p;
    } 
    else                //dispari
    {
        dx = dx_d;
    }
    const int *dy;
    if (x % 2 == 0)     //pari
    {
        dy = dy_p;
    } 
    else                //dispari
    {
        dy = dy_d;
    }

    *vx = x + dx[k];
    *vy = y + dy[k];

    return (*vx >= 0 && *vx < m->colonne && *vy >= 0 && *vy < m->righe);    //true se vicino valido
}

bool coordinate_vic(int x, int y, __int8_t k, int *vx, int *vy)
{
    bool result=0;
    switch (k)
    {
    case 0:
        *vx = x ;
        *vy = y - 1;
        result = *vy >= 0;
        break;
    case 1:
        *vx = x;
        *vy = y + 1;
        result = *vy < m->righe;
        break;
    case 2:
        *vx = x - 1;
        *vy = y;
        result = *vx >= 0;
        break;
    case 3:
        *vx = x + 1;
        *vy = y;
        result = *vx < m->colonne;
        break;
    case 4:
        if (y % 2 == 0) //riga pari
        {
            *vx = x - 1;
            *vy = y + 1;
            result = *vx >= 0 && *vy < m->righe;
        }
        else
        {
            *vx = x + 1;
            *vy = y - 1;
            result = *vx < m->colonne && *vy >= 0;
        }
        break;
    case 5:
        if(y % 2 == 0)
        {
            *vx = x - 1;
            *vy = y - 1;
            result = *vx >= 0 && *vy >= 0;
        }
        else
        {
            *vx = x + 1;
            *vy = y + 1;
            result = *vx < m->colonne && *vy < m->righe;
        }
        break;
    }
    return result;    //true se vicino valido
}

void heap_init(MinHeap *h, int cap)
{
    h->buf = malloc(cap * sizeof(Node));
    h->c = cap; 
    h->size = 0;
}

void heap_init2(MinHeap2 *h, int cap)
{
    h->buf = malloc(cap * sizeof(Node2));
    h->c = cap;
    h->size = 0;
}

void heap_push(MinHeap *h, Node v)
{

    int i = h->size++;
    if (i > h->c - 1)
    {
        h->c += KSIZE;
        h->buf = realloc (h->buf, h->c * sizeof(Node));
    }
    h->buf[i] = v;
    while (i > 0)//while non è radice
    {
        int p = (i - 1) >> 1;   //  i-1/2
        if (h->buf[p].d <= h->buf[i].d) break;
        Node tmp = h->buf[p];
        h->buf[p] = h->buf[i];
        h->buf[i] = tmp;
        i = p;
    }
}

void heap_push2(MinHeap2 *h, Node2 v)
{

    int i = h->size++;
    if (i > h->c - 1)
    {
        h->c += KSIZE;
        h->buf = realloc (h->buf, h->c * sizeof(Node2));
    }
    h->buf[i] = v;
    while (i > 0)//while non è radice
    {
        int p = (i - 1) >> 1;   //  i-1/2
        if (h->buf[p].d <= h->buf[i].d) break;
        Node2 tmp = h->buf[p];
        h->buf[p] = h->buf[i];
        h->buf[i] = tmp;
        i = p;
    }
}

Node heap_pop(MinHeap *h)
{
    Node ret = h->buf[0];
    //heapify!!
    h->buf[0] = h->buf[--h->size]; //prendo la foglia e la metto in root, decremento e assegno
    int i = 0;
    while (1)
    {
        int l = 2 * i + 1;
        int r = l + 1;
        int m = i;

        // trova il più piccolo tra i child
        if (l < h->size && h->buf[l].d < h->buf[m].d)
            {m = l;}        
        if (r < h->size && h->buf[r].d < h->buf[m].d)
            {m = r;}

        if (m == i)
            {break;}
            
        // swap
        Node temp = h->buf[i];
        h->buf[i] = h->buf[m];
        h->buf[m] = temp;

        //aggiorna e continua da quella posizione
        i = m;
    }
    return ret;
}

Node2 heap_pop2(MinHeap2 *h)
{
    Node2 ret = h->buf[0];
    h->buf[0] = h->buf[--h->size]; //prendo la foglia e la metto in root, decremento e assegno
    int i = 0;
    while (1)
    {
        int l = 2 * i + 1;
        int r = l + 1;
        int m = i;

        // trova il più piccolo tra i child
        if (l < h->size && h->buf[l].d < h->buf[m].d)
            {m = l;}        
        if (r < h->size && h->buf[r].d < h->buf[m].d)
            {m = r;}

        if (m == i)
            {break;}
            
        // swapp
        Node2 temp = h->buf[i];
        h->buf[i] = h->buf[m];
        h->buf[m] = temp;

        //aggiorna e continua da quella posizione
        i = m;
    }
    return ret;
}

void heap_free(MinHeap *h)
{
    free(h->buf);
    h->buf = NULL;
    h->size = 0;
    h->c = 0;
}

void heap_free2(MinHeap2 *h)
{
    free(h->buf);
    h->buf = NULL;
    h->size = 0;
    h->c = 0;
}

//COMANDI
/*void init(int col, int rig) 
{
    printf("OK\n"); //nessun check su comandi
    if (m != NULL) 
    {
        distruggi_m();
    }
    
    m = malloc(sizeof(Map));
    //imposta le dimensioni
    m->righe = rig;     //N DI RIGHE!!! VANNO DA 0 A N-1
    m->griglia = malloc(rig * sizeof(Hex*));    //crea tutti gli indici di righe
    m->colonne = col;
   
    
    //inizializza tutti gli esagoni
    for (int i = rig-1; i >=0; i--) 
    {
        m->griglia[i] = malloc(col * sizeof(Hex));  //crea tutti gli esagoni
        for (int j = col-1 ; j >=0 ; j--)
        {
            m->griglia[i][j].x = j; //riga iy colonna jx
            m->griglia[i][j].y = i;
            m->griglia[i][j].air_count = 0;  //nessuna rotta aerea iniziale
            m->griglia[i][j].land_cost = 1;  //costo iniziale = 1
            //vedi appunti
            //inizializza array dei vicini
            for (int k = 5; k >= 0; k--) //A OGNI ITERAZIONE CREA UN VICINO
            {
                int vi, vj; //vicini i e j
                
                if (i % 2 == 0) 
                {
                    vi = i + dy_p[k];
                    vj = j + dx_p[k];
                } 
                else 
                {
                    vi = i + dy_d[k];
                    vj = j + dx_d[k];
                }
                
                //verifica vicino in griglia
                if (vi >= 0 && vi < rig && vj >= 0 && vj < col) 
                {
                    m->griglia[i][j].vicinox[k] = vj;//assegna vicino se è valido
                    m->griglia[i][j].vicinoy[k] = vi;
                } 
                else
                {
                    m->griglia[i][j].vicinox[k] = -1;//indica che il vicino non è valido
                    m->griglia[i][j].vicinoy[k] = -1;
                }
            }
        }
    }
    //debug_map(); // Stampa la mappa per debug
    return;
}
*/

void lazy_init(int col, int rig)    //alloca on-demand
{
    printf("OK\n"); //nessun check su comandi
    if (m != NULL) 
    {
        distruggi_m();
    }
    m = malloc(sizeof(Map));
    m->righe = rig;
    m->colonne = col;
    m->griglia = malloc(rig * sizeof(Hex**));
    for (int i = rig-1; i >= 0; i--) 
    {
        m->griglia[i] = calloc(col, sizeof(Hex*));
    }
    n=col * rig;
}

void change_cost(int x, int y, int v, int raggio)
{
    //x e y sono indici

    if (x < 0 || y < 0 || x >= m->colonne || y >= m->righe || raggio == 0 || v < -10 || v > 10) //check validità coordinate e v
    {
        printf("KO\n");
        return;
    }

    if(0 == v)
    {
        printf("OK\n");
        return; //change non ha nessun effetto, evita il calcolo
    }


    //invalida cache
   gen_cache++;

    //alloco gli esagoni necessari per la procedura

    
    int i_start = MAX(y-raggio,0);
    int i_end = MIN(y+raggio,m->righe-1);
    //scansione ottimizzata(non tanto utile), vedi appunti
    int j_start = MAX(x-raggio,0);
    int j_end = MIN(x+raggio,m->colonne-1);
    for(int i = i_start; i <= i_end;i++)
    {
        for(int j = j_start; j <= j_end;j++)
        {
            //VERSIONE OTTIMIZZATA DELLA FORMULA
            //nota: non puoi fare tutto in int !!!!!!!
            //fai calcoli in float e poi fai floor !
            int d = disth(i, j, y, x);
            if (d >= raggio) continue;
            //calcolo ottimizzato del delta
            int sum = v * (raggio - d) / raggio;
            if (v < 0 && (v * (raggio - d) % raggio != 0)) //ffloor manuale
            {
                sum--;  //arrotondamento per difetto per valori negativi
            }
            if(sum == 0) continue;
            Hex *h = alloca_hex(j, i);   

            // costo terra
            int tempcost = h->land_cost;
            tempcost += sum;
            if (tempcost < 0) h->land_cost = 0;
            else if (tempcost > 100) h->land_cost = 100;
            else h->land_cost = tempcost;
        }
    }
    printf("OK\n");
    return;
}

void toggle_air_route(int x1, int y1, int x2, int y2)
{
    if (y1 < 0 || x1 < 0 || y2 < 0 || x2 < 0 ||
        y1 >= m->righe || x1 >= m->colonne || y2 >= m->righe || x2 >= m->colonne) {
        printf("KO\n"); return;
    }
    if (x1 == x2 && y1 == y2) { printf("OK\n"); return; }

    gen_cache++;

    Hex *h = alloca_hex(x1, y1); // crea il sorgente

    // rimuovi se esiste
    //cintrolla se allcato o no
    for (int i = h->air_count-1; i >=0; i--) 
    {
        if (h->air_routes[i]->x == x2 && h->air_routes[i]->y == y2) 
        {
            h->air_routes[i] = h->air_routes[h->air_count - 1]; //sposta l'ultimo in quello rimosso
            h->air_count--;
            h->air_routes = realloc(h->air_routes, h->air_count * sizeof(Hex*)); //ridimensiona
            printf("OK\n");
            return;
        }           

    }

    //aggiungi
    if(h->air_count == 0)
    {
        h->air_routes = malloc(sizeof(Hex*));
        h->air_routes[0] = alloca_hex(x2, y2);
        h->air_count = 1;
        printf("OK\n");
        return;
    }
    else if (h->air_count < 5)
    {
        h->air_count++;
        h->air_routes = realloc(h->air_routes, (h->air_count) * sizeof(Hex*));
        h->air_routes[h->air_count-1] = alloca_hex(x2, y2);
        printf("OK\n");
        return;
    }

    /*for(int i=0; i<5;i++)
    {
        if(h->air_routes[i] == NULL) //trova posto libero
        {
            h->air_routes[i] = alloca_hex(x2, y2);
            printf("OK\n");
            return;
        }
    }*/
    printf("KO\n");
    return;
}

void travel_cost(int x, int y, int xd, int yd)
{
    if (m==0 ||x >= m->colonne || y >= m->righe || xd >= m->colonne || yd >= m->righe ||
        x < 0 || y < 0 || xd < 0 || yd < 0)
    {
        printf("-1\n");
        return;
    }

    if (xd == x && yd == y)
    {
        printf("0\n");
        return;
    }
    //cache
    int da = y * m->colonne + x;
    int a = yd * m->colonne + xd;
    int hash = (da * 23 + a) % CACHE; //pericoloso
    Cella *cache = &cache_glob[hash];
    if(cache->valid == gen_cache && cache->da == da && cache->a == a) //se già calcolato
    {
        printf("%d\n", cache->costo);
        return;
    }

    int n = m->righe * m->colonne;  //dim
    dist = malloc(n * sizeof(int));    // TOTALE O(5n)
    vis = calloc(n, sizeof(bool));    //inizializzato
    // Initialize distances
    for (int i = n-1; i >= 0; i--) dist[i] = I;

    //MinHeap heap; 
    MinHeap2 heap2;
    //heap_init(&heap, KSIZE);
    heap_init2(&heap2, KSIZE);

    ///Hex *start_h = alloca_hex(x, y);
    int start = y * m->colonne + x;
    dist[start] = 0;
    //heap_push(&heap, (Node){ start_h, 0 });
    heap_push2(&heap2, (Node2){ x, y, 0 });
    Hex *h;

    int ans = I;
    while (heap2.size > 0)
    {
        Node2 u = heap_pop2(&heap2);
        //h = alloca_hex(u.h->x, u.h->y); // assicura esagono
        int ui = u.y * m->colonne + u.x;
        if (vis[ui]) continue;
        vis[ui] = true;

        if (u.x == xd && u.y == yd) 
        { 
            ans = u.d; 
            break; 
        }
        // terra
        if(m->griglia[u.y][u.x] == NULL) //se non è mai stato allocato
        {
            //espando sapendo che land cost = 1 e no rotte aeree
            for (__int8_t k = 5; k >=0; k--) 
            {
                int nx ,ny;
                if (coordinate_vic(u.x, u.y, k, &nx, &ny) == false) continue;
                //if (nx < 0 || ny < 0) continue;
                //if (nx >= m->colonne || ny >= m->righe) continue;
                
                //Hex *nh = alloca_hex(nx, ny); // crea vicino on-demand
                int vi = ny * m->colonne + nx;
                if (vis[vi]) continue;
                int nd = u.d + 1;
                if (nd < dist[vi]) 
                {
                    dist[vi] = nd;
                    heap_push2(&heap2, (Node2){ nx, ny, nd });
                }
            }
            /*for (int k = 0; k < 5; k++) 
            {
                if(h->air_routes[k] == NULL) continue;
                //AirRoute *r = &h->air_routes[k];
                int nx = h->air_routes[k]->x, ny = h->air_routes[k]->y;
                if (nx < 0 || ny < 0 || nx >= m->colonne || ny >= m->righe) continue;
                int vi = ny * m->colonne + nx;
                if (vis[vi]) continue;
                int nd = u.d + h->land_cost;
                if (nd < dist[vi]) 
                {
                    dist[vi] = nd;
                    heap_push(&heap, (Node){ h->air_routes[k], nd });
                }
            }NO AIR ROUTE*/
        }
        else
        {
            h = m->griglia[u.y][u.x];
            if (h->land_cost > 0) 
            {
                for (__int8_t k = 5; k >= 0; k--)
                {
                    int nx, ny;
                    if (coordinate_vic(h->x, h->y, k, &nx, &ny) == false) continue;
                    /*if (nx < 0 || ny < 0) continue;
                    if (nx >= m->colonne || ny >= m->righe) continue;
                    */
                    //Hex *nh = alloca_hex(nx, ny); // crea vicino on-demand
                    int vi = ny * m->colonne + nx;
                    if (vis[vi]) continue;

                    int nd = u.d + h->land_cost;
                    if (nd < dist[vi]) 
                    {
                        dist[vi] = nd;
                        heap_push2(&heap2, (Node2){ nx, ny, nd });
                    }
                }
                
                //espansione aerea
                for (__int8_t k = h->air_count-1; k >=0; k--) 
                {
                    //if(h->air_routes[k] == NULL) continue;
                    //AirRoute *r = &h->air_routes[k];
                    int nx = h->air_routes[k]->x, ny = h->air_routes[k]->y;
                    //if (nx < 0 || ny < 0 || nx >= m->colonne || ny >= m->righe) continue;
                    int vi = ny * m->colonne + nx;
                    if (vis[vi]) continue;

                    int nd = u.d + h->land_cost;
                    if (nd < dist[vi]) 
                    {
                        dist[vi] = nd;
                        heap_push2(&heap2, (Node2){ h->air_routes[k]->x, h->air_routes[k]->y, nd });
                    }
                }
            }
        }
        // rotte aeree
    }
    // Aggiorna cache
    cache->da = da;
    cache->a = a;
    cache->valid = gen_cache;


    if (ans != I)
    {
        printf("%d\n", ans);
        cache->costo = ans;
    }
    else
    {
        printf("-1\n");
        cache->costo = -1;
    }


    heap_free2(&heap2);
    free(dist);
    free(vis);
    
    return;
}




int main(void)
{
    char cmd[50];
    int ntokens;
    char *tokens[5]; // Array di 5 puntatori a char
    //scansione da stdin
    while(fgets(cmd, sizeof(cmd), stdin) != NULL) //se non sono arrivato a EOF       
    {
        cmd[strcspn(cmd, "\n")] = '\0';
        ntokens = 0;
        char *tok = strtok(cmd, " \t");
        
        while (tok != NULL && ntokens < 5)  //crea tokens
        {
            tokens[ntokens] = tok;
            ntokens++;
            tok = strtok(NULL, " \t");
        }

        //riconosci comandi
        //ottimizzo leggendo solo un carattere
        if (cmd[0] == 'i')            
        {
            lazy_init(atoi(tokens[1]), atoi(tokens[2]));
            continue;
        }
        if(cmd[0] == 'c')
        {
            change_cost(atoi(tokens[1]), atoi(tokens[2]), atoi(tokens[3]), atoi(tokens[4]));
            continue;
        }
        if (cmd[1] == 'o') //toggle air route
        {
            toggle_air_route(atoi(tokens[1]), atoi(tokens[2]), atoi(tokens[3]), atoi(tokens[4]));
            continue;
        }
        if (cmd[1] == 'r') //travel cost
        {
            travel_cost(atoi(tokens[1]), atoi(tokens[2]), atoi(tokens[3]), atoi(tokens[4]));
            continue;
        }
        if (cmd[0] == 'd')    // Stampa la mappa per debug
        {
            //printf("Comando 'debug_map' ricevuto e riconosciuto\n");
            //debug_map();
        }
    }
    distruggi_m(); // free finale!
    return 0;
}