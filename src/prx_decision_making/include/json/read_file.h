/*
 * read_file.h
 *
 *  Created on: DFeb 1, 201BIN_SIZE
 *      Author: shaojun
 */

#ifndef READ_FILE_H_
#define READ_FILE_H_

#include <json/json.h>
#include <json/json-forwards.h>
#include <algorithm> // sort
#include <stdio.h>
#include <stdlib.h>


#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <cstdlib>

#define NUMBER_OF_ORDER 1000



const int BIN_SIZE = 5;

struct BinContents
{   
    std::string bin_A[BIN_SIZE];
    std::string bin_B[BIN_SIZE];
    std::string bin_C[BIN_SIZE];
    std::string bin_D[BIN_SIZE];
    std::string bin_E[BIN_SIZE];
    std::string bin_F[BIN_SIZE];
    std::string bin_G[BIN_SIZE];
    std::string bin_H[BIN_SIZE];
    std::string bin_I[BIN_SIZE];
    std::string bin_J[BIN_SIZE];
    std::string bin_K[BIN_SIZE];
    std::string bin_L[BIN_SIZE];
    std::string bin_M[BIN_SIZE];
    std::string bin_N[BIN_SIZE];
    std::string bin_O[BIN_SIZE];
    std::string bin_P[BIN_SIZE];
    std::string bin_Q[BIN_SIZE];
    std::string bin_R[BIN_SIZE];
    std::string bin_S[BIN_SIZE];
    std::string bin_T[BIN_SIZE];
    std::string bin_U[BIN_SIZE];
    std::string bin_V[BIN_SIZE];
    std::string bin_W[BIN_SIZE];
    std::string bin_X[BIN_SIZE];
    std::string bin_Y[BIN_SIZE];
    std::string bin_Z[BIN_SIZE];

    int item_counts[26];
};

enum PREFERENCE
{
    ROBOTIQ,UNIGRIPPER,ROBOTIQ_UNIGRIPPER,UNIGRIPPER_ROBOTIQ
};

class WorkOrder
{public: 
  std::string bin[NUMBER_OF_ORDER];
  std::string item[NUMBER_OF_ORDER];
  bool done[NUMBER_OF_ORDER];
  int item_counts[NUMBER_OF_ORDER];
  unsigned remaining_items;
  PREFERENCE gripper[NUMBER_OF_ORDER];
};

const int NUM_ITEMS = 25;

struct ItemPreferences
{
    std::string items[NUM_ITEMS];
    PREFERENCE gripper[NUM_ITEMS];
    int priority[NUM_ITEMS];
};

int read_file(std::string& file_name, WorkOrder& work_order, BinContents& bin_contents);
int read_preferences(std::string& file_name, ItemPreferences& i_pref);
void count_items_in_bins(BinContents& bin_contents,WorkOrder& work_order,int num_items);

#endif
