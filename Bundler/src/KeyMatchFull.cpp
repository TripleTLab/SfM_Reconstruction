#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "keys2a.h"

int ReadFileList(char* list_in, std::vector<std::string>& key_files) {
    FILE* fp;

    if ((fp = fopen(list_in, "r")) == NULL) {
        printf("Error opening file %s for reading.\n", list_in);
        return 1;
    }

    char buf[512], *start;
    while (fgets(buf, 512, fp)) {
        if (buf[strlen(buf) - 1] == '\n') buf[strlen(buf) - 1] = '\0';

        // 找到第一个非空字符
        start = buf;
        while(isspace(*start)) start++;

        // 跳过空行
        if (strlen(start) == 0) continue;

        // 添加文件名
        key_files.push_back(std::string(buf));
    }

    // 检查输入文件
    if (key_files.size() == 0) {
        printf("No input files found in %s.\n", list_in);
        return 1;
    }

    return 0;
}

int main(int argc, char **argv) {
    char *list_in;
    char *file_out;
    double ratio;

    if (argc != 3 && argc != 4) {
        printf("Usage: %s <list.txt> <outfile> [window_radius]\n", argv[0]);
        return EXIT_FAILURE;
    }

    list_in = argv[1];
    ratio = 0.6;
    file_out = argv[2];

    int window_radius = -1;
    if (argc == 4) {
        window_radius = atoi(argv[3]);
    }

    clock_t start = clock();

    /* 读取文件列表 */
    std::vector<std::string> key_files;
    if (ReadFileList(list_in, key_files) != 0) return EXIT_FAILURE;

    FILE *f;
    if ((f = fopen(file_out, "w")) == NULL) {
        printf("Could not open %s for writing.\n", file_out);
        return EXIT_FAILURE;
    }

    int num_images = (int) key_files.size();

    std::vector<unsigned char*> keys(num_images);
    std::vector<int> num_keys(num_images);

    /* 读取所有特征点 */
    for (int i = 0; i < num_images; i++) {
        keys[i] = NULL;
        num_keys[i] = ReadKeyFile(key_files[i].c_str(), &keys[i]);
    }

    clock_t end = clock();
    printf("[KeyMatchFull] Reading keys took %0.3fs\n",
           (end - start) / ((double) CLOCKS_PER_SEC));

    for (int i = 0; i < num_images; i++) {
        if (num_keys[i] == 0)
            continue;

        printf("[KeyMatchFull] Matching to image %d\n", i);

        start = clock();

        /* Create a tree from the keys */
        ANNkd_tree *tree = CreateSearchTree(num_keys[i], keys[i]);

        /* Compute the start index */
        int start_idx = 0;
        if (window_radius > 0)
            start_idx = std::max(i - window_radius, 0);

        for (int j = start_idx; j < i; j++) {
            if (num_keys[j] == 0)
                continue;

            /* Compute likely matches between two sets of keypoints */
            std::vector<KeypointMatch> matches =
                MatchKeys(num_keys[j], keys[j], tree, ratio);

            int num_matches = (int) matches.size();

            if (num_matches >= 16) {
                /* Write the pair */
                fprintf(f, "%d %d\n", j, i);

                /* Write the number of matches */
                fprintf(f, "%d\n", (int) matches.size());

                for (int i = 0; i < num_matches; i++) {
                    fprintf(f, "%d %d\n",
                            matches[i].m_idx1, matches[i].m_idx2);
                }
            }
        }

        end = clock();
        printf("[KeyMatchFull] Matching took %0.3fs\n",
               (end - start) / ((double) CLOCKS_PER_SEC));
        fflush(stdout);

        delete tree;
    }

    /* Free keypoints */
    for (int i = 0; i < num_images; i++) {
        if (keys[i] != NULL)
            delete [] keys[i];
    }

    fclose(f);
    return EXIT_SUCCESS;
}
