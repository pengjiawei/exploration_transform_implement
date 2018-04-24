#include <iostream>
#include <cstring>
#include <algorithm>
#include "transform_explorer.h"
void diaplay_map(int* map_array,int size_x,int size_y){
    for (int i = 0; i < size_y; ++i) {
        for (int j = 0; j < size_x; ++j) {
            printf("%d ",map_array[j + i * size_x]);
        }
        printf("\n");
    }
    printf("\n");
}
void display_trajectory(int* map_array,int size_x,int size_y,std::vector<int>& trajectory){
    for (int i = 0; i < size_y; ++i) {
        for (int j = 0; j < size_x; ++j) {
            auto iter = std::find(trajectory.begin(),trajectory.end(),j + i * size_x);
            if (iter != trajectory.end())
                printf("# ");
            else
                printf("%d ",map_array[j + i * size_x]);
        }
        printf("\n");
    }
    printf("\n");
}
int main() {
    transform_explorer t(10,10);

    int* occ_grid = new int[10 * 10]();
    occ_grid[63] = OBSTACLE_COST;
    printf("occ grid\n");
    diaplay_map(occ_grid,10,10);
    t.init_occupancy_grid_array(occ_grid);
    t.init_obstacle_trans_array();
    printf("obstacle array\n");
    diaplay_map(t.get_obs_trans_array(),10,10);

    t.init_explorer_trans_array(60);
    printf("explorer array 0 is target 100 is obstacle\n");
    diaplay_map(t.get_explorer_trans_array(),10,10);
    std::vector<int> trajectory;
    t.get_trajectory(36,trajectory);

    printf("\nadd trajectory explorer array\n");
    display_trajectory(t.get_explorer_trans_array(),10,10,trajectory);
    std::cout << "Hello, World!" << std::endl;
    return 0;
}