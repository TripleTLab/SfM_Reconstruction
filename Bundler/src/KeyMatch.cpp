#include <time.h>

#include "keys2.h"

int main(int argc, char **argv) {
    char *keys1_in;
    char *keys2_in;
    char *file_out;
    double ratio;

    if (argc != 4) {
	printf("Usage: %s <keys1.in> <keys2.in> <out.txt>\n", argv[0]);
	return -1;
    }

    keys1_in = argv[1];
    keys2_in = argv[2];
    ratio = 0.6;
    file_out = argv[3];

    clock_t start = clock();

    short int *keys1, *keys2;

    int num1 = ReadKeyFile(keys1_in, &keys1);
    int num2 = ReadKeyFile(keys2_in, &keys2);

    clock_t end = clock();
    printf("Reading keys took %0.3fs\n",
	   (end - start) / ((double) CLOCKS_PER_SEC));

    /* 计算两个点集中对应点的相似度 */
    std::vector<KeypointMatch> matches =
	MatchKeys(num1, keys1, num2, keys2, ratio);


    int num_matches = (int) matches.size();

    printf("num_matches = %d\n", num_matches);


    if (num_matches >= 16) {
	FILE *f = fopen(file_out, "w");

	/* 写入匹配数量 */
	fprintf(f, "%d\n", (int) matches.size());

	for (int i = 0; i < num_matches; i++) {
	    fprintf(f, "%d %d\n", matches[i].m_idx1, matches[i].m_idx2);
	}

	fclose(f);
    }
}
