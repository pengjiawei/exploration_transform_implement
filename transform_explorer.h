//
// Created by pengjiawei on 2018/4/24.
//

#ifndef EXPLORATION_TRANSFORM_IMPLEMENT_TRANSFORM_EXPLORER_H
#define EXPLORATION_TRANSFORM_IMPLEMENT_TRANSFORM_EXPLORER_H
#include <queue>
#include <cstring>
#include <cstdint>

const int FREE_COST = 0;
const int OBSTACLE_COST = -1;
//上下左右方向上的代价
const int STRAIGHT_COST = 1;
//对角线的代价
const int DIAGONAL_COST = 2;

const int DEFAULT_COST = 100;
class transform_explorer {
public:
    transform_explorer(int size_x_,int size_y_){
        size_x = size_x_;
        size_y = size_y_;
        num_cells = size_x * size_y;
        obstacle_trans_array = new int[num_cells];
        explorer_trans_array = new int[num_cells];
        occupancy_grid_array = new int[num_cells];

        std::fill_n(obstacle_trans_array,num_cells,DEFAULT_COST);
        std::fill_n(explorer_trans_array,num_cells,DEFAULT_COST);
    }

    void init_obstacle_trans_array();
    void init_explorer_trans_array(int goal_point);
    void init_occupancy_grid_array(int* occupancy_grid_array_){
        occupancy_grid_array = occupancy_grid_array_;
    }
    int* get_obs_trans_array(){
        return obstacle_trans_array;
    }
    int* get_explorer_trans_array(){
        return explorer_trans_array;
    }
    void get_trajectory(int start_point,std::vector<int>& trajectory);

    inline int left(int point){
        if((point % size_x != 0)){
            return point-1;
        }
        return -1;
    }
    inline int upleft(int point){
        if((point % size_x != 0) && (point >= (int)size_x)){
            return point-1-size_x;
        }
        return -1;

    }
    inline int up(int point){
        if(point >= (int)size_x){
            return point-size_x;
        }
        return -1;
    }
    inline int upright(int point){
        if((point >= (int)size_x) && ((point + 1) % (int)size_x != 0)){
            return point-size_x+1;
        }
        return -1;
    }
    inline int right(int point){
        if((point + 1) % size_x != 0){
            return point+1;
        }
        return -1;

    }
    inline int downright(int point){
        if(((point + 1) % size_x != 0) && ((point/size_x) < (size_y-1))){
            return point+size_x+1;
        }
        return -1;

    }
    inline int down(int point){
        if((point/size_x) < (size_y-1)){
            return point+size_x;
        }
        return -1;

    }
    inline int downleft(int point){
        if(((point/size_x) < (size_y-1)) && (point % size_x != 0)){
            return point+size_x-1;
        }
        return -1;

    }
private:
    bool is_valid(int point);
    bool is_free(int point);
    void get_straight_point(int point,int straight_point[4]);
    void get_diagonal_point(int point,int diagonal_point[4]);
    void get_adjacent_point(int point,int adjacent_point[8]);
    int size_x,size_y,num_cells;
    int* obstacle_trans_array;
    int* occupancy_grid_array;
    int* explorer_trans_array;
    int goal_point;
};


#endif //EXPLORATION_TRANSFORM_IMPLEMENT_TRANSFORM_EXPLORER_H
