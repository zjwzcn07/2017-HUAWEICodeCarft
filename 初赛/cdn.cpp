#include "deploy.h"
#include "lib_io.h"
#include "lib_time.h"
#include "stdio.h"
#include <stdlib.h>
#include "min_cost_max_flow.h"



int main(int argc, char *argv[])
{

    char *topo[MAX_EDGE_NUM];
    int line_num;

	char *topo_file = argv[1];
    clock_t stt = clock();
    line_num = read_file(topo, MAX_EDGE_NUM, topo_file);

    printf("line num is :%d \n", line_num);
    if (line_num == 0)
    {
        printf("Please input valid topo file.\n");
        return -1;
    }

	char *result_file = argv[2];

    deploy_server(topo, line_num, result_file);

    release_buff(topo, line_num);
    clock_t edd = clock();
    clock_t ddd = edd - stt;
    printf("cost time =%ds %dms\n",ddd/CLOCKS_PER_SEC,ddd%CLOCKS_PER_SEC);
    return 0;
}

