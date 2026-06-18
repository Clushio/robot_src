/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include<myglobal_planner/astar.h>
#include<costmap_2d/cost_values.h>
#include<iostream>

using namespace std;


namespace myglobal_planner {

AStarExpansion::AStarExpansion(PotentialCalculator* p_calc, int xs, int ys) :
        Expander(p_calc, xs, ys) {
}

bool AStarExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                        int cycles, float* potential) {
   
    queue_.clear();           //queue_其实就是openlist
    int start_i = toIndex(start_x, start_y);
    queue_.push_back(Index(start_i, 0));      

    std::fill(potential, potential + ns_, POT_HIGH);
    potential[start_i] = 0;

    int goal_i = toIndex(end_x, end_y);  //将目标坐标点转化成数组中的索引
    int cycle = 0;

    while (queue_.size() > 0 && cycle < cycles) {
        Index top = queue_[0];  //得到最小的F值
        std::pop_heap(queue_.begin(), queue_.end(), greater1());//将堆顶(所给范围的最前面)元素移动到所给范围的最后，并且将新的最大值置于所给范围的最前面
        queue_.pop_back(); //弹出最小的F，也就是从openlist移除

        int i = top.i;
        float cost=top.cost;

        if (i == goal_i)
            return true;

        add(costs, potential, potential[i], i + 1, end_x, end_y,1);  //当前点右边的点
        add(costs, potential, potential[i], i - 1, end_x, end_y,1);
        add(costs, potential, potential[i], i + nx_, end_x, end_y,1);//当前点下方的点
        add(costs, potential, potential[i], i - nx_, end_x, end_y,1);
        // add(costs, potential, potential[i], i -nx_-1, end_x, end_y,1);  //当前点右边的点
        // add(costs, potential, potential[i], i - nx_+1, end_x, end_y,1);
        // add(costs, potential, potential[i], i + nx_-1, end_x, end_y,1);//当前点下方的点
        // add(costs, potential, potential[i], i + nx_+1, end_x, end_y,1);

        cycle++;
    }

    return false;
}

void AStarExpansion::add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x,
                         int end_y,float factor) {
    
    if (next_i < 0 || next_i >= ns_){
        
        return;
    }
        

    if (potential[next_i] < POT_HIGH){//位于openlist不考虑
        
        return;
    }

    if(costs[next_i]>=lethal_cost_&& !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION)){//障碍物  lethal_cost_=253,unknown_=true
        
        return;
    }
   

    
	//p_calc_是PotentialCalculator类型，netural_cost_指的是移动一个格子的代价
    potential[next_i] = p_calc_->calculatePotential(potential, costs[next_i] + factor*neutral_cost_, next_i, prev_potential);
    int x = next_i % nx_, y = next_i / nx_;//得到下一个点的坐标值
    float distance = abs(end_x - x) + abs(end_y - y);
    queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_));//potential[next_i] + distance * neutral_cost_表示F=G+H
    std::push_heap(queue_.begin(), queue_.end(), greater1());//堆排序，从小到大
}

} //end namespace global_planner
