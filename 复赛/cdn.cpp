#include "deploy.h"
#include "lib_io.h"
#include "lib_time.h"
#include "stdio.h"
#include <stdlib.h>
#include <ctime>

//char* argv1 = (char*) "./case/case_easy.txt";

//char* argv1 =(char*) "./case/case1.txt";
char* argv1 = (char*) "./case_example/1/0/case0.txt";
char* argv2 = (char*) "./result/result.txt";


int main(int argc, char *argv[])
{

	srand(time(0));/////////
    print_time("Begin");
    char *topo[MAX_EDGE_NUM];
    int line_num;

	char *topo_file = argv1;

    line_num = read_file(topo, MAX_EDGE_NUM, topo_file);

    printf("line num is :%d \n", line_num);
    if (line_num == 0)
    {
        printf("Please input valid topo file.\n");
        return -1;
    }

	char *result_file = argv2;

    deploy_server(topo, line_num, result_file);

    release_buff(topo, line_num);

    print_time("End");

	system("pause");
	return 0;
}

