#include <stdio.h>
#include "matrix.h"
#include "kinematics_position.h"
#include "kinematics_velocity.h"
#include "kinematics_acceleration.h"
#include "kinematics_torque.h"
#include "path_generation.h"


#define POINTS 2        //Numero di punti inseriti


int main(void)
{

    /**
     * I punti vanno insteriti nel vettore 'points_elem', come negli esempi qui sotto.
     * Devono essere inseriti nella forma di 'vettore colonna', quindi il vettore
     * sarà riempito prima con le 'x' di tutti i punti, successivamente le 'y' e le 'z'.
     * Inserire anche il numero di punti nella macro 'POINTS'.
     */




    /*
    double points_elem[3*POINTS] = {    +0.0,   +0.0,   +0.1,   -0.2,   -0.2,
                                        +0.0,   +0.0,   +0.2,   +0.2,   +0.0,
                                        -0.6,   -0.7,   -0.7,   -0.7,   -0.8};
    */
    
    double points_elem[3*POINTS] = {    +0.0,   +0.1,
                                        +0.0,   +0.3,
                                        -0.6,   -0.7};

    /*
    double points_elem[3*POINTS] = {    +0.4,   -0.1,
                                        +0.0,   +0.3,
                                        -0.6,   -0.8};
    */
    /*
    double points_elem[3*POINTS] = {    +0.0,   +0.0,    +0.2,   -0.1,   +0.3,   -0.3,   -0.1,
                                        +0.0,   -0.2,    +0.3,   +0.1,   +0.2,   +0.3,   +0.0,
                                        -0.6,   -0.7,    -0.6,   -0.7,   -0.7,   -0.7,   -0.8};
    */

    matrix *points = fill_matrix(3, POINTS, points_elem);
    matrix *pos = NULL;
    matrix *vel = NULL;
    matrix *acc = NULL;


    /**
     * L'algoritmo può essere impostato dal secondo, terzo e quarto parametro.
     * Rispettivamente:
     * - tempo in cui completare il persorso [s]
     * - precisione con cui l'algoritmo ricerca il percorso ottimale (consigliato < 0.1)
     * - risoluzione della curva campionata (per ogni tratto tra due punti)
     */

    path(points, 1.0, 0.01, 500, &pos, &vel, &acc);

    write_data(pos, vel, acc, 1.0);

    free_matrix(points);
    free_matrix(pos);
    free_matrix(vel);
    free_matrix(acc);

    printf("Done!\n");

    return 0;
}