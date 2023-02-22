#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <mpi.h>

using namespace std;

int n =1;


int check_circuit (int id, int z) {
	int v[16];
	int i;
	for(i=0;i <16;i++) 
                        v[i]=(n &(1<<i)) ? 1: 0;	
        return n;
}


int main (int argc, char * argv[]) {
    int global_solutions;
    /* Total number of solutions */
    int i ;
    int id;
    /* Process rank */
    int p;
    /* Number of processes */
    int solutions;
    /* Solutions found by this proc */
    int  check_circuit (int, int) ;
    
    
    MPI_Init ( &argc, &argv);
    
    MPI_Comm_rank (MPI_COMM_WORLD, &id);
    MPI_Comm_size (MPI_COMM_WORLD, &p);
    
    solutions = 0;
    for (i = id; i < 65536; i += p)
        solutions += check_circuit (id,i);
    
    MPI_Reduce (&solutions,  &global_solutions,  1,  MPI_INT,  MPI_SUM, 0, MPI_COMM_WORLD);
    printf ("Process %d is done\n", id);
   
    MPI_Finalize();
    if (id == 0) printf ("There are %d different solutions\n", global_solutions);
    return 0;
}
