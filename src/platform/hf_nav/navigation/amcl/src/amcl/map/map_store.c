/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/**************************************************************************
 * Desc: Global map storage functions
 * Author: Andrew Howard
 * Date: 6 Feb 2003
 * CVS: $Id: map_store.c 2951 2005-08-19 00:48:20Z gerkey $
 **************************************************************************/

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "amcl/map/map.h"

////////////////////////////////////////////////////////////////////////////
// Load an occupancy grid
// 加载占据栅格，输入为map_t格式，文件，地图的分辨率等
int map_load_occ(map_t *map, const char *filename, double scale, int negate) {
  FILE *file;
  char magic[3];
  int i, j;
  int ch, occ;
  int width, height, depth;
  map_cell_t *cell;

  // Open file
  file = fopen(filename, "r");
  if (file == NULL) {
    fprintf(stderr, "%s: %s\n", strerror(errno), filename);
    return -1;
  }

  // Read ppm header

  if ((fscanf(file, "%2s \n", magic) != 1) || (strcmp(magic, "P5") != 0)) {
    // PGM格式的图片，其实就是已经存在的栅格地图
    fprintf(stderr, "incorrect image format; must be PGM/binary");
    fclose(file);
    return -1;
  }

  // Ignore comments
  while ((ch = fgetc(file)) == '#')
    while (fgetc(file) != '\n')
      ;
  ungetc(ch, file);

  // Read image dimensions
  if (fscanf(file, " %d %d \n %d \n", &width, &height, &depth) != 3) {
    fprintf(stderr, "Failed ot read image dimensions");
    return -1;
  }

  // Allocate space in the map
  if (map->cells == NULL) {
    map->scale = scale;
    map->size_x = width;
    map->size_y = height;

    // 根据传入地图的长宽，创建map_t格式下map的size大小
    map->cells = calloc(width * height, sizeof(map->cells[0]));
  } else {
    if (width != map->size_x || height != map->size_y) {
      // PLAYER_ERROR("map dimensions are inconsistent with prior map
      // dimensions");
      return -1;
    }
  }

  // Read in the image
  for (j = height - 1; j >= 0; j--) {
    for (i = 0; i < width; i++) {
      /* 关于fgetc()函数
       在文件内部有一个位置指针，用来指向当前读写到的位置，也就是读写到第几个字节。
      在文件打开时，该指针总是指向文件的第一个字节。使用 fgetc() 函数后，
      该指针会向后移动一个字节，所以可以连续多次使用 fgetc() 读取多个字符。
      */
      ch = fgetc(file);

      // Black-on-white images
      if (!negate) {
        if (ch < depth / 4)
          occ = +1;
        else if (ch > 3 * depth / 4)
          occ = -1;
        else
          occ = 0;
      }

      // White-on-black images
      else {
        if (ch < depth / 4)
          occ = -1;
        else if (ch > 3 * depth / 4)
          occ = +1;
        else
          occ = 0;
      }

      if (!MAP_VALID(map, i, j))
        continue;
        
      //栅格单元
      cell = map->cells + MAP_INDEX(map, i, j);
      //栅格单元的状态填充
      cell->occ_state = occ;
    }
  }

  fclose(file);

  return 0;
}

////////////////////////////////////////////////////////////////////////////
// Load a wifi signal strength map
/*
int map_load_wifi(map_t *map, const char *filename, int index)
{
  FILE *file;
  char magic[3];
  int i, j;
  int ch, level;
  int width, height, depth;
  map_cell_t *cell;

  // Open file
  file = fopen(filename, "r");
  if (file == NULL)
  {
    fprintf(stderr, "%s: %s\n", strerror(errno), filename);
    return -1;
  }

  // Read ppm header
  fscanf(file, "%10s \n", magic);
  if (strcmp(magic, "P5") != 0)
  {
    fprintf(stderr, "incorrect image format; must be PGM/binary");
    return -1;
  }

  // Ignore comments
  while ((ch = fgetc(file)) == '#')
    while (fgetc(file) != '\n');
  ungetc(ch, file);

  // Read image dimensions
  fscanf(file, " %d %d \n %d \n", &width, &height, &depth);

  // Allocate space in the map
  if (map->cells == NULL)
  {
    map->size_x = width;
    map->size_y = height;
    map->cells = calloc(width * height, sizeof(map->cells[0]));
  }
  else
  {
    if (width != map->size_x || height != map->size_y)
    {
      //PLAYER_ERROR("map dimensions are inconsistent with prior map
dimensions"); return -1;
    }
  }

  // Read in the image
  for (j = height - 1; j >= 0; j--)
  {
    for (i = 0; i < width; i++)
    {
      ch = fgetc(file);

      if (!MAP_VALID(map, i, j))
        continue;

      if (ch == 0)
        level = 0;
      else
        level = ch * 100 / 255 - 100;

      cell = map->cells + MAP_INDEX(map, i, j);
      cell->wifi_levels[index] = level;
    }
  }

  fclose(file);

  return 0;
}
*/
