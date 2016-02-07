#!/usr/bin/env python
import ast
import os

##############################################################################################
# Order Bin Description                                                                      #
##############################################################################################

class OrderBinDescription(object):
    def __init__(self, file_name):
        file_name = str(os.environ['PRACSYS_PATH'])+"/../object_models/"+file_name
        f = open(file_name, 'r')
        str_desc = f.readline()
        initial_data = ast.literal_eval(str_desc)
        f.close()
        for key in initial_data:
           setattr(self, key, initial_data[key])    
    def __str__(self):
        return "{'x':%s, 'y':%s, 'z':%s, 'roll':%s, 'pitch':%s, 'yaw':%s, 'width':%s, 'height':%s}"  % (self.x, self.y, self.z, self.roll, self.pitch, self.yaw, self.width, self.height)
    
##############################################################################################
# Shelf Description                                                                          #
##############################################################################################

class ShelfDescription(object):
    def __init__(self, file_name):
        file_name = str(os.environ['PRACSYS_PATH'])+"/../object_models/"+file_name
        f = open(file_name, 'r')
        str_desc = f.readline()
        initial_data = ast.literal_eval(str_desc)
        f.close()
        for key in initial_data:
           setattr(self, key, initial_data[key])  
    def __str__(self):
        return "{'x':%s, 'y':%s, 'z':%s, 'roll':%s, 'pitch':%s, 'yaw':%s, 'width':%s, 'height':%s, 'rows':%s, 'cols':%s, 'a_col_width':%s, 'b_col_width':%s, 'c_col_width':%s, 'a_row_height':%s, 'd_row_height':%s, 'g_row_height':%s, 'j_row_height':%s, 'shelfs_lip_height':%s, 'support_column_width':%s}"  % (
                self.x, self.y, self.z, self.roll, self.pitch, self.yaw, 
                self.width, self.height, self.rows, self.cols, 
                self.a_col_width, self.b_col_width, self.c_col_width, 
                self.a_row_height, self.d_row_height, self.g_row_height, self.j_row_height, 
                self.shelfs_lip_height, self.support_column_width)

    def get_z_bin_center(self,bin):
        a_row = ['A', 'B', 'C']
        d_row = ['D', 'E', 'F']
        g_row = ['G', 'H', 'I']
        j_row = ['J', 'K', 'L']
        if a_row.count(bin) != 0: # bin is in the a row
            z_center_bin = self.z + 4*self.shelfs_lip_height+\
            self.j_row_height + self.g_row_height + self.d_row_height + \
            (self.a_row_height ) / 2.0
            bin_height = self.a_row_height
        elif d_row.count(bin) != 0: # bin is in the d row
            z_center_bin = self.z + 3*self.shelfs_lip_height+\
            self.j_row_height + self.g_row_height + \
            (self.d_row_height) / 2.0
            bin_height = self.d_row_height
        elif g_row.count(bin) != 0: # bin is in the g row
            z_center_bin = self.z + 2*self.shelfs_lip_height + \
            self.j_row_height + \
            (self.g_row_height ) / 2.0
            bin_height = self.g_row_height
        else: # bin is in the j row
            z_center_bin = self.z  + self.shelfs_lip_height + \
            (self.j_row_height) / 2.0
            bin_height = self.j_row_height
        
        return z_center_bin, bin_height

    def get_y_bin_center(self,bin):
        a_col = ['A', 'D', 'G', 'J']
        b_col = ['B', 'E', 'H', 'K']
        c_col = ['C', 'F', 'I', 'L']
        y_center_bin = 0.0
        if a_col.count(bin) != 0: # bin is in the a column
            y_center_bin = self.y + \
                self.b_col_width / 2.0 + \
                self.support_column_width +\
                (self.a_col_width) / 2.0
            bin_width = (self.a_col_width)
        elif b_col.count(bin) != 0: # bin is in the b column
            y_center_bin = self.y
            bin_width = self.b_col_width
        else: # bin is in the c column
            y_center_bin = self.y - \
            self.b_col_width / 2.0 - \
            self.support_column_width- \
            (self.c_col_width) / 2.0
            bin_width = self.c_col_width
        
        return y_center_bin, bin_width