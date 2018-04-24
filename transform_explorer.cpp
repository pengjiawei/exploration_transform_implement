//
// Created by pengjiawei on 2018/4/24.
//


#include "transform_explorer.h"

//目前在探索路径的时候还没有加上obstacle的影响
void transform_explorer::init_obstacle_trans_array() {
    std::queue<int> queue1;

    // 广度优先，从障碍物的点开始传播
    for(int i=0; i < num_cells; ++i){
        if(occupancy_grid_array[i] == OBSTACLE_COST){
            queue1.push(i);
            obstacle_trans_array[i] = 0;
        }
    }

    // obstacle transform algorithm
    while(queue1.size()){
        int point = queue1.front();
        queue1.pop();


        unsigned int current_point_cost = obstacle_trans_array[point];

        int straight_points[4];
        get_straight_point(point,straight_points);
        int diagonal_points[4];
        get_diagonal_point(point,diagonal_points);

        // check all 8 directions
        for(int i = 0; i < 4; ++i){
            if (is_valid(straight_points[i]) && (obstacle_trans_array[straight_points[i]] > current_point_cost + STRAIGHT_COST)) {
                obstacle_trans_array[straight_points[i]] = current_point_cost + STRAIGHT_COST;
                queue1.push(straight_points[i]);
            }
            if (is_valid(diagonal_points[i]) && (obstacle_trans_array[diagonal_points[i]] > current_point_cost + DIAGONAL_COST)) {
                obstacle_trans_array[diagonal_points[i]] = current_point_cost + DIAGONAL_COST;
                queue1.push(diagonal_points[i]);
            }
        }
    }
}

void transform_explorer::get_straight_point(int point, int *straight_point) {
    straight_point[0] = left(point);  //left
    straight_point[1] = up(point); //up
    straight_point[2] = right(point);  //right
    straight_point[3] = down(point); //down
}

void transform_explorer::get_diagonal_point(int point, int *diagonal_point) {
    diagonal_point[0] = upleft(point); //upleft
    diagonal_point[1] = upright(point);  //upright
    diagonal_point[2] = downright(point);  //downright
    diagonal_point[3] = downleft(point);  //downleft
}

bool transform_explorer::is_valid(int point) {
    return  point > 0 && point < num_cells;
}

void transform_explorer::init_explorer_trans_array(int goal_point_) {
    goal_point = goal_point_;
    std::queue<int> queue1;
    queue1.push(goal_point_);
    explorer_trans_array[goal_point_] = 0;

    int straight_points[4];
    int diagonal_points[4];

    while (!queue1.empty()){
        int point = queue1.front();
        queue1.pop();

        get_straight_point(point,straight_points);
        get_diagonal_point(point,diagonal_points);

        unsigned int current_point_cost = explorer_trans_array[point];

        // calculate the minimum exploration value of all adjacent cells
        for (int i = 0; i < 4; ++i) {
            //如果某些格子是障碍，那么就保留它的原代价值
            if (is_free(straight_points[i])) {
                unsigned int neighbor_cost = current_point_cost + STRAIGHT_COST;

                if (explorer_trans_array[straight_points[i]] > neighbor_cost) {
                    explorer_trans_array[straight_points[i]] = neighbor_cost;
                    queue1.push(straight_points[i]);
                }
            }

            if (is_free(diagonal_points[i])) {
                unsigned int neighbor_cost = current_point_cost + DIAGONAL_COST;

                if (explorer_trans_array[diagonal_points[i]] > neighbor_cost) {
                    explorer_trans_array[diagonal_points[i]] = neighbor_cost;
                    queue1.push(diagonal_points[i]);
                }
            }
        }

    }
}

bool transform_explorer::is_free(int point) {
    return occupancy_grid_array[point] == 0;
}

void transform_explorer::get_trajectory(int start_point,std::vector<int>& trajectory) {
    int current_point = start_point;
    trajectory.push_back(current_point);
    int next_point;
    while(current_point != goal_point){
        int adjacentPoints[8];
        get_adjacent_point(current_point,adjacentPoints);
        int max_delta = 0;
        int current_delta;
        //遍历开始位置的周围八个点，寻找出最小探索代价的点
        for(int i = 0; i < 8; ++i){
            if(is_free(adjacentPoints[i])){
                current_delta = explorer_trans_array[current_point] - explorer_trans_array[adjacentPoints[i]];
                if(current_delta > max_delta){
                    max_delta = current_delta;
                    next_point = adjacentPoints[i];
                }
            }
        }
        trajectory.push_back(next_point);
        //相当于行走一步
        current_point = next_point;
    }
}

void transform_explorer::get_adjacent_point(int point, int *adjacent_point) {
    adjacent_point[0] = left(point);  //left
    adjacent_point[1] = up(point); //up
    adjacent_point[2] = right(point);  //right
    adjacent_point[3] = down(point); //down
    adjacent_point[4] = upleft(point); //upleft
    adjacent_point[5] = upright(point);  //upright
    adjacent_point[6] = downright(point);  //downright
    adjacent_point[7] = downleft(point);  //downleft
}


