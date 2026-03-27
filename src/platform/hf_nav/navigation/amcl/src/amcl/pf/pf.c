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
 * Desc: Simple particle filter for localization.
 * Author: Andrew Howard
 * Date: 10 Dec 2002
 * CVS: $Id: pf.c 6345 2008-04-17 01:36:39Z gerkey $
 *************************************************************************/

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

#include "amcl/pf/pf.h"
#include "amcl/pf/pf_kdtree.h"
#include "amcl/pf/pf_pdf.h"
#include "portable_utils.hpp"

// Compute the required number of samples, given that there are k bins
// with samples in them.
// pf粒子滤波器是粒子sample集的来源，k是至少填充了一个粒子的直方图的位数
static int pf_resample_limit(pf_t *pf, int k);

// Create a new filter
// 创建一个粒子滤波器，输入为我们关心的粒子sample集的最小数目，最大数目，
// alpha_slow, alpha_fast，粒子滤波器初始化模型，随机生成数据的函数。
pf_t *pf_alloc(int min_samples, int max_samples, double alpha_slow,
               double alpha_fast, pf_init_model_fn_t random_pose_fn,
               void *random_pose_data) {
  //在函数内部对粒子滤波器,粒子sample集，粒子sample进行定义声明初始化。
  int i, j;
  pf_t *pf;
  //粒子sample集
  pf_sample_set_t *set;
  //粒子sample
  pf_sample_t *sample;

  srand48(time(NULL));

  //根据粒子滤波器的类型创建pf

  pf = calloc(1, sizeof(pf_t));

  //对粒子滤波器的随机位姿生成模型进行初始化
  pf->random_pose_fn = random_pose_fn;
  pf->random_pose_data = random_pose_data;

  //粒子sample集的最小数目, 最大数目
  pf->min_samples = min_samples;
  pf->max_samples = max_samples;

  // Control parameters for the population size calculation.  [err] is
  // the max error between the true distribution and the estimated
  // distribution.  [z] is the upper standard normal quantile for (1 -
  // p), where p is the probability that the error on the estimated
  // distrubition will be less than [err].
  // 接下来用到的几个参数:pop_err,pop_z，dist_threshold均与KLD算法有关。

  // pop_error代表两个概率分布差的最大测度（真实分布与估计分布）
  pf->pop_err = 0.01;

  // pop_z代表标准正态分布上的1-sigma分位点
  pf->pop_z = 3;
  pf->dist_threshold = 0.5;

  // Number of leaf nodes is never higher than the max number of samples
  // 叶子节点的数目不能高过粒子样本集的最大数目
  pf->limit_cache = calloc(max_samples, sizeof(int));

  // 关于粒子滤波器的两个粒子sample集：current_set和set
  // set包括两个集合, current_set 指定这两个集合中谁是活跃的
  //将pf->current_set设置为0
  pf->current_set = 0;

  //使用一个for循环，遍历两次：从而刷新两个set
  for (j = 0; j < 2; j++) {
    // set是指针，这里使用j依次读取pf->sets的元素set
    set = pf->sets + j;

    //设置粒子集set的sample集最大数目
    //给粒子sample集set以粒子sample集的最大数目分配内存
    set->sample_count = max_samples;
    set->samples = calloc(max_samples, sizeof(pf_sample_t));

    //这里使用一个for循环遍历该粒子sample集set,从而读取到每一个sample对象
    for (i = 0; i < set->sample_count; i++) {
      // sample是指针，这里使用i依次读取set->samples的元素sample
      sample = set->samples + i;

      //对粒子sample的位姿和权重进行初始化
      sample->pose.v[0] = 0.0;
      sample->pose.v[1] = 0.0;
      sample->pose.v[2] = 0.0;
      sample->weight = 1.0 / max_samples;
    }

    // HACK: is 3 times max_samples enough?
    //粒子sample集set的kdtree数据结构，用三倍于粒子sample集的最大数目来分配内存：
    //实际上这里需要计算复杂度，很显然代码的作者并没有计算，而是简单粗暴分配三倍内存
    set->kdtree = pf_kdtree_alloc(3 * max_samples);

    //对粒子sample集set的簇cluster进行初始化
    set->cluster_count = 0;
    set->cluster_max_count = max_samples;
    set->clusters = calloc(set->cluster_max_count, sizeof(pf_cluster_t));

    //对粒子sample集set的均值和协方差进行初始化
    set->mean = pf_vector_zero();
    set->cov = pf_matrix_zero();
  }

  pf->w_slow = 0.0;
  pf->w_fast = 0.0;

  pf->alpha_slow = alpha_slow;
  pf->alpha_fast = alpha_fast;

  // set converged to 0
  //最后，将粒子sample集set收敛于0
  pf_init_converged(pf);

  return pf;
}

// Free an existing filter
//销毁一个粒子滤波器
void pf_free(pf_t *pf) {
  int i;

  free(pf->limit_cache);

  for (i = 0; i < 2; i++) {
    free(pf->sets[i].clusters);
    pf_kdtree_free(pf->sets[i].kdtree);
    free(pf->sets[i].samples);
  }
  free(pf);

  return;
}

// 初始化粒子滤波器
// Initialize the filter using a gaussian
// 使用高斯模型（均值，协方差）初始化滤波器
void pf_init(pf_t *pf, pf_vector_t mean, pf_matrix_t cov) {
  int i;

  //创建粒子sample集set,粒子sample对象，概率密度函数pdf
  pf_sample_set_t *set;
  pf_sample_t *sample;
  pf_pdf_gaussian_t *pdf;

  //初始化粒子sample集set
  set = pf->sets + pf->current_set;

  // Create the kd tree for adaptive sampling
  //创建kdtree为了选择性采样
  pf_kdtree_clear(set->kdtree);

  //粒子sample集set的sample数目设定
  set->sample_count = pf->max_samples;

  //使用高斯模型(均值，协方差)创建pdf
  pdf = pf_pdf_gaussian_alloc(mean, cov);

  // Compute the new sample poses
  //计算新的粒子sample对象的位姿
  //使用for循环遍历粒子sample集set的每个sample对象
  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;

    //对粒子sample的权重进行初始化
    sample->weight = 1.0 / pf->max_samples;

    //对粒子sample的位姿使用pdf模型产生
    sample->pose = pf_pdf_gaussian_sample(pdf);

    // Add sample to histogram
    //以kdtree数据结构的方式添加/存储粒子sample
    //或者说以kdtree数据结构的方式表达直方图
    pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
  }

  pf->w_slow = pf->w_fast = 0.0;

  //销毁释放pdf占用的内存
  pf_pdf_gaussian_free(pdf);

  // Re-compute cluster statistics
  //再次计算粒子sample集set的簇cluster的统计特性，输入为粒子滤波器pf,和set
  pf_cluster_stats(pf, set);

  // set converged to 0
  pf_init_converged(pf);

  return;
}

// Initialize the filter using some model
// 第二种方法, 使用pf_init_model_fn_t模型初始化
void pf_init_model(pf_t *pf, pf_init_model_fn_t init_fn, void *init_data) {
  int i;
  //创建粒子sample集set,粒子sample对象
  pf_sample_set_t *set;
  pf_sample_t *sample;

  //初始化粒子sample集set
  set = pf->sets + pf->current_set;

  // Create the kd tree for adaptive sampling
  // 创建kdtree为了选择性采样
  pf_kdtree_clear(set->kdtree);

  set->sample_count = pf->max_samples;

  // Compute the new sample poses
  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;
    sample->weight = 1.0 / pf->max_samples;

    //看这里！生成粒子位姿的方式不一样，不一样！
    //对粒子sample的位姿使用pf_init_model_fn_t模型生成
    sample->pose = (*init_fn)(init_data);

    // Add sample to histogram
    //以kdtree数据结构的方式添加/存储粒子sample
    //或者说以kdtree数据结构的方式表达直方图
    pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
  }

  pf->w_slow = pf->w_fast = 0.0;

  // Re-compute cluster statistics
  pf_cluster_stats(pf, set);

  // set converged to 0
  pf_init_converged(pf);

  return;
}

// 粒子滤波器初始化收敛
void pf_init_converged(pf_t *pf) {
  pf_sample_set_t *set;
  set = pf->sets + pf->current_set;

  //就是这么简单，将converged标志符改成0
  set->converged = 0;
  pf->converged = 0;
}

// 粒子滤波器更新收敛
//这里的关键在于dist_threshold,
//粒子的位姿pose在x,y方向上的值分别减去平均值mean_x,mean_y,
//得到的差值与dist_threshold比较，
//如果差值大于dist_threshold那就显示没收敛，粒子还处于发散的状态中
int pf_update_converged(pf_t *pf) {
  int i;

  //创建粒子sample集set,粒子sample对象
  pf_sample_set_t *set;
  pf_sample_t *sample;
  double total;

  //初始化粒子sample集set
  set = pf->sets + pf->current_set;
  double mean_x = 0, mean_y = 0;

  //计算新的粒子sample对象的位姿
  //使用for循环遍历粒子sample集set的每个sample对象
  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;

    //计算粒子sample对象位姿pose的x,y方向上的和
    mean_x += sample->pose.v[0];
    mean_y += sample->pose.v[1];
  }

  //计算粒子sample对象位姿pose的x,y方向上的均值mean
  mean_x /= set->sample_count;
  mean_y /= set->sample_count;

  //使用for循环遍历粒子sample集set的每个sample对象
  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;

    //这里的关键在于dist_threshold,粒子的位姿pose在x,y方向上的值分别减去平均值mean_x,
    // mean_y,得到的结果与dist_threshold比较，如果结果大于dist_threshold那就显示没收敛，
    //粒子还处于发散的状态中，至少dist_threshold表示不满意！
    // dist_threshold在代码中设置为 0.5
    if (fabs(sample->pose.v[0] - mean_x) > pf->dist_threshold ||
        fabs(sample->pose.v[1] - mean_y) > pf->dist_threshold) {
      set->converged = 0; //发散情况下，converged设置为0
      pf->converged = 0;  //发散情况下，converged设置为0
      return 0;
    }
  }
  set->converged = 1; //收敛情况下，converged设置为1
  pf->converged = 1;  //收敛情况下，converged设置为1
  return 1;
}

// 粒子滤波器随运动和观测更新

// Update the filter with some new action
// 首先是根据运动来更新粒子滤波器，其实是更新粒子的位姿
void pf_update_action(pf_t *pf, pf_action_model_fn_t action_fn,
                      void *action_data) {
  //创建粒子sample集set
  pf_sample_set_t *set;

  //初始化粒子sample集set
  set = pf->sets + pf->current_set;

  //引入运动模型action_fn，举个例子，这里我选择差分模型：odom_diff
  // 配置文件选择全向 omni
  (*action_fn)(action_data, set);

  return;
}

// Update the filter with some new sensor observation
// 然后是根据观测来更新粒子滤波器,其实是更新粒子的权重
void pf_update_sensor(pf_t *pf, pf_sensor_model_fn_t sensor_fn,
                      void *sensor_data) {
  int i;

  //创建粒子sample集set,粒子sample对象
  pf_sample_set_t *set;
  pf_sample_t *sample;

  //想想看这个total是干什么的呢？
  double total;

  //初始化粒子sample集set
  set = pf->sets + pf->current_set;

  // Compute the sample weights
  //引入sensor_fn，这里我选择似然域模型:likelihood_filed
  //来计算粒子sample集set的权重
  total = (*sensor_fn)(sensor_data, set);

  //又一个重要参数n_eff
  set->n_effective = 0;

  //如果呢，观测模型计算出的粒子sample集set的权值(或者说概率probability)大于0，那么：
  if (total > 0.0) {

    // Normalize weights
    // 粒子sample集sample对象的权重归一化，设置w_avg
    double w_avg = 0.0;

    //使用for循环遍历粒子sample集set的每个sample对象
    for (i = 0; i < set->sample_count; i++) {
      sample = set->samples + i;

      //利用粒子sample集sample对象的权重对w_avg赋值
      w_avg += sample->weight;

      //粒子集sample对象的权重归一化
      sample->weight /= total;

      //利用粒子sample集sample对象的权重的平方对粒子集set的n_eff参数赋值
      set->n_effective += sample->weight * sample->weight;
    }

    // Update running averages of likelihood of samples (Prob Rob p258)
    //将w_avg除以粒子集sample数目，得到新的w_avg
    w_avg /= set->sample_count;

    //重新刷新w_slow的值，怎么刷新呢？
    //如果w_slow等于0，那么将w_avg的值赋给它
    if (pf->w_slow == 0.0)
      pf->w_slow = w_avg;
    else //如果w_slow不等于0，那么它将获得alpha_slow*(w_avg-w_slow)
      pf->w_slow += pf->alpha_slow * (w_avg - pf->w_slow);

    //重新刷新w_fast的值，怎么刷新呢？
    //如果w_fast等于0，那么将w_avg的值赋给它
    if (pf->w_fast == 0.0)
      pf->w_fast = w_avg;
    else //如果w_fast不等于0，那么它将获得alpha_fast*(w_avg-w_fast)
      pf->w_fast += pf->alpha_fast * (w_avg - pf->w_fast);
    // printf("w_avg: %e slow: %e fast: %e\n",
    // w_avg, pf->w_slow, pf->w_fast);
  } else {

    //如果呢，观测模型计算出的粒子sample集set的权值(或者说概率probability)不大于0，那该怎么办呢？
    // Handle zero total
    //使用for循环遍历粒子sample集set的每个sample对象，并将每个sample的权重设置得一模一样的
    //都是weight = 1.0 / sample_count;
    for (i = 0; i < set->sample_count; i++) {
      sample = set->samples + i;
      sample->weight = 1.0 / set->sample_count;
    }
  }

  //最后，将粒子sample集set的n_eff参数设置为：1/n_eff
  set->n_effective = 1.0 / set->n_effective;
  return;
}

// copy set a to set b
// 复制粒子sample集set
void copy_set(pf_sample_set_t *set_a, pf_sample_set_t *set_b) {
  int i;
  double total;

  //创建粒子sample对象sample_a,sample_b
  pf_sample_t *sample_a, *sample_b;

  // Clean set b's kdtree
  //清除粒子sample集set_b的kdtree
  pf_kdtree_clear(set_b->kdtree);

  // Copy samples from set a to create set b
  //从set_a那里复制sample来创建set_b
  total = 0;

  //将set_b的sample数目初始化为0
  set_b->sample_count = 0;

  //使用for循环遍历粒子sample集set_a的每个sample对象,再copy到set_b里
  for (i = 0; i < set_a->sample_count; i++) {
    sample_b = set_b->samples + set_b->sample_count++;

    sample_a = set_a->samples + i;

    assert(sample_a->weight > 0);

    // Copy sample a to sample b
    // 不仅仅是sample的位姿，权重也要一并复制
    sample_b->pose = sample_a->pose;
    sample_b->weight = sample_a->weight;

    //一直在累积粒子sample_b的权重
    total += sample_b->weight;

    // Add sample to histogram
    // //将粒子sample_b加入到直方图里，这里用kdtree的数据结构表示，结果就是插入到粒子sample集set_b里
    pf_kdtree_insert(set_b->kdtree, sample_b->pose, sample_b->weight);
  }

  // Normalize weights
  //将粒子sample集set_b的权重归一化
  for (i = 0; i < set_b->sample_count; i++) {
    sample_b = set_b->samples + i;
    sample_b->weight /= total;
  }

  //将粒子sample集set_b的收敛状态设置得跟粒子sample集set_a一致
  set_b->converged = set_a->converged;
}

// Resample the distribution
// 对分布进行重采样
void pf_update_resample(pf_t *pf) {
  int i;
  double total;

  //创建粒子sample集set_a,set_b
  //创建粒子sample对象sample_a,sample_b
  pf_sample_set_t *set_a, *set_b;
  pf_sample_t *sample_a, *sample_b;

  // double r,c,U;
  // int m;
  // double count_inv;
  double *c;

  // Attention:新的参数:w_diff!
  double w_diff;

  //初始化粒子sample集set_a,set_b，但是两个有点细微的区别
  set_a = pf->sets + pf->current_set;
  set_b = pf->sets + (pf->current_set + 1) % 2;

  //如果选择性采样标志不等于0，那么：
  if (pf->selective_resampling != 0) {
    //如果粒子sample集set_a的n_eff > 0.5 * (set_a的粒子sample数目)，那么：
    if (set_a->n_effective > 0.5 * (set_a->sample_count)) {
      // copy set a to b
      copy_set(set_a, set_b);

      // Re-compute cluster statistics
      // //再次计算簇的统计特性，输入为pf和粒子sample集set_b
      pf_cluster_stats(pf, set_b);

      // Use the newly created sample set
      //使用新创建的粒子sample集，其实就是对current_set的值刷新一下
      pf->current_set = (pf->current_set + 1) % 2;
      return;
    }
  }

  //结束选择性采样标识的判断，下面我们进入重头戏部分。这里弃用了低方差采样，改用传统的采样方法

  // Build up cumulative probability table for resampling.
  // TODO: Replace this with a more efficient procedure
  // (e.g.,
  // http://www.network-theory.co.uk/docs/gslref/GeneralDiscreteDistributions.html)

  //用粒子sample集set_a的粒子sample数目来创建c的内存
  c = (double *)malloc(sizeof(double) * (set_a->sample_count + 1));
  c[0] = 0.0;

  //根据粒子sample集set_a的粒子sample数目，遍历，
  //从而将粒子sample集set_a的粒子sample对象的权重依次取出来，存在c里
  for (i = 0; i < set_a->sample_count; i++)
    c[i + 1] = c[i] + set_a->samples[i].weight;

  // Create the kd tree for adaptive sampling
  //创建kdtree为选择性采样做准备
  pf_kdtree_clear(set_b->kdtree);

  // Draw samples from set a to create set b.
  //从粒子sample集set_a中draw samples去创造粒子sample集set_b
  total = 0;
  set_b->sample_count = 0;

  w_diff = 1.0 - pf->w_fast / pf->w_slow;

  //如果w_diff小于0，那就将w_diff置为0咯！
  if (w_diff < 0.0)
    w_diff = 0.0;
  // printf("w_diff: %9.6f\n", w_diff);

  //由于呢，KLD适应性采样不是那么容易用低方差采样实现，所以这里采用传统方法实现
  // Can't (easily) combine low-variance sampler with KLD adaptive
  // sampling, so we'll take the more traditional route.
  //虽然这么说，代码的作者还是贴上了低方差采样原理的出处，当然是在《概率机器人》里了
  /*
  // Low-variance resampler, taken from Probabilistic Robotics, p110
  count_inv = 1.0/set_a->sample_count;
  r = drand48() * count_inv;
  c = set_a->samples[0].weight;
  i = 0;
  m = 0;
  */

  //当粒子sample集set_b的sample数目小于粒子滤波器的sample的最大数目
  while (set_b->sample_count < pf->max_samples) {

    //逐步从粒子sample集的set_b中取出sample对象
    sample_b = set_b->samples + set_b->sample_count++;

    //当随机数小于w_diff;w_diff的赋值在上面实现过了
    if (drand48() < w_diff)
      //使用随机生成位姿的模型生成sample_b的位姿；真够传统的方法呀，拍拍手。
      sample_b->pose = (pf->random_pose_fn)(pf->random_pose_data);

    //当随机数大于w_diff;那就复杂了，怎么生成sample_b的位姿呢？进入else吗？这里注释掉了
    //低方差重采样的代码实现，谁也没跑过，摊摊我的小手，我也不知道。
    else {
      // Can't (easily) combine low-variance sampler with KLD adaptive
      // sampling, so we'll take the more traditional route.
      /*
      //低方差重采样实现，实际上没用哦
      // Low-variance resampler, taken from Probabilistic Robotics, p110
      U = r + m * count_inv;
      while(U>c)
      {
        i++;
        // Handle wrap-around by resetting counters and picking a new random
        // number
        if(i >= set_a->sample_count)
        {
          r = drand48() * count_inv;
          c = set_a->samples[0].weight;
          i = 0;
          m = 0;
          U = r + m * count_inv;
          continue;
        }
        c += set_a->samples[i].weight;
      }
      m++;
      */

      // Naive discrete event sampler
      //简单采样器
      double r;
      r = drand48();

      //遍历粒子sample集set_a，把c拉出来跟r做比较；
      for (i = 0; i < set_a->sample_count; i++) {
        if ((c[i] <= r) && (r < c[i + 1]))
          break;
      }
      assert(i < set_a->sample_count);

      //遍历粒子sample集set_a，将sample_a取出来
      sample_a = set_a->samples + i;

      assert(sample_a->weight > 0);

      // Add sample to list
      //把sample_a的位姿赋给sample_b
      sample_b->pose = sample_a->pose;
    }

    //将sample_b的权重初始化为1
    sample_b->weight = 1.0;

    //计算sample_b所在set的权重和
    total += sample_b->weight;

    // Add sample to histogram
    //把粒子sample_b添加到kdtree里，其实也就是添加到粒子sample集set_b里
    pf_kdtree_insert(set_b->kdtree, sample_b->pose, sample_b->weight);

    // See if we have enough samples yet
    //检查看是否有足够的粒子sample了呢？
    //这里使用pf_resample_limit函数，输入为粒子sample集set_b的kdtree的叶子节点个数，这也代表
    //着直方图的k个bin。
    if (set_b->sample_count > pf_resample_limit(pf, set_b->kdtree->leaf_count))
      break;
  }

  //重设w_slow和w_fast
  // Reset averages, to avoid spiraling off into complete randomness.
  if (w_diff > 0.0)
    pf->w_slow = pf->w_fast = 0.0;

  // fprintf(stderr, "\n\n");

  // Normalize weights
  // 使用for循环，遍历粒子sample集set_b,将sample_b权重归一化
  for (i = 0; i < set_b->sample_count; i++) {
    sample_b = set_b->samples + i;
    sample_b->weight /= total;
  }

  // Re-compute cluster statistics
  // 重新计算粒子sample集set_b的簇的统计特性
  pf_cluster_stats(pf, set_b);

  // Use the newly created sample set
  //重新设置current_set的值
  pf->current_set = (pf->current_set + 1) % 2;

  pf_update_converged(pf);

  free(c);
  return;
}

// 给定直方图的bins的数目k，计算所需的粒子sample的数目，这个可真难呀！
// Compute the required number of samples, given that there are k bins
// with samples in them.  This is taken directly from Fox et al.
int pf_resample_limit(pf_t *pf, int k) {
  double a, b, c, x;
  int n;

  // Return max_samples in case k is outside expected range, this shouldn't
  // happen, but is added to prevent any runtime errors
  if (k < 1 || k > pf->max_samples)
    return pf->max_samples;

  // Return value if cache is valid, which means value is non-zero positive
  if (pf->limit_cache[k - 1] > 0)
    return pf->limit_cache[k - 1];

  if (k == 1) {
    pf->limit_cache[k - 1] = pf->max_samples;
    return pf->max_samples;
  }

  a = 1;
  b = 2 / (9 * ((double)k - 1));
  c = sqrt(2 / (9 * ((double)k - 1))) * pf->pop_z;
  x = a - b + c;

  n = (int)ceil((k - 1) / (2 * pf->pop_err) * x * x * x);

  if (n < pf->min_samples) {
    pf->limit_cache[k - 1] = pf->min_samples;
    return pf->min_samples;
  }
  if (n > pf->max_samples) {
    pf->limit_cache[k - 1] = pf->max_samples;
    return pf->max_samples;
  }

  pf->limit_cache[k - 1] = n;
  return n;
}

// Re-compute the cluster statistics for a sample set
void pf_cluster_stats(pf_t *pf, pf_sample_set_t *set) {
  int i, j, k, cidx;
  pf_sample_t *sample;
  pf_cluster_t *cluster;

  // Workspace
  double m[4], c[2][2];
  size_t count;
  double weight;

  // Cluster the samples
  pf_kdtree_cluster(set->kdtree);

  // Initialize cluster stats
  set->cluster_count = 0;

  for (i = 0; i < set->cluster_max_count; i++) {
    cluster = set->clusters + i;
    cluster->count = 0;
    cluster->weight = 0;
    cluster->mean = pf_vector_zero();
    cluster->cov = pf_matrix_zero();

    for (j = 0; j < 4; j++)
      cluster->m[j] = 0.0;
    for (j = 0; j < 2; j++)
      for (k = 0; k < 2; k++)
        cluster->c[j][k] = 0.0;
  }

  // Initialize overall filter stats
  count = 0;
  weight = 0.0;
  set->mean = pf_vector_zero();
  set->cov = pf_matrix_zero();
  for (j = 0; j < 4; j++)
    m[j] = 0.0;
  for (j = 0; j < 2; j++)
    for (k = 0; k < 2; k++)
      c[j][k] = 0.0;

  // Compute cluster stats
  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;

    // printf("%d %f %f %f\n", i, sample->pose.v[0], sample->pose.v[1],
    // sample->pose.v[2]);

    // Get the cluster label for this sample
    cidx = pf_kdtree_get_cluster(set->kdtree, sample->pose);
    assert(cidx >= 0);
    if (cidx >= set->cluster_max_count)
      continue;
    if (cidx + 1 > set->cluster_count)
      set->cluster_count = cidx + 1;

    cluster = set->clusters + cidx;

    cluster->count += 1;
    cluster->weight += sample->weight;

    count += 1;
    weight += sample->weight;

    // Compute mean
    cluster->m[0] += sample->weight * sample->pose.v[0];
    cluster->m[1] += sample->weight * sample->pose.v[1];
    cluster->m[2] += sample->weight * cos(sample->pose.v[2]);
    cluster->m[3] += sample->weight * sin(sample->pose.v[2]);

    m[0] += sample->weight * sample->pose.v[0];
    m[1] += sample->weight * sample->pose.v[1];
    m[2] += sample->weight * cos(sample->pose.v[2]);
    m[3] += sample->weight * sin(sample->pose.v[2]);

    // Compute covariance in linear components
    for (j = 0; j < 2; j++)
      for (k = 0; k < 2; k++) {
        cluster->c[j][k] +=
            sample->weight * sample->pose.v[j] * sample->pose.v[k];
        c[j][k] += sample->weight * sample->pose.v[j] * sample->pose.v[k];
      }
  }

  // Normalize
  for (i = 0; i < set->cluster_count; i++) {
    cluster = set->clusters + i;

    cluster->mean.v[0] = cluster->m[0] / cluster->weight;
    cluster->mean.v[1] = cluster->m[1] / cluster->weight;
    cluster->mean.v[2] = atan2(cluster->m[3], cluster->m[2]);

    cluster->cov = pf_matrix_zero();

    // Covariance in linear components
    for (j = 0; j < 2; j++)
      for (k = 0; k < 2; k++)
        cluster->cov.m[j][k] = cluster->c[j][k] / cluster->weight -
                               cluster->mean.v[j] * cluster->mean.v[k];

    // Covariance in angular components; I think this is the correct
    // formula for circular statistics.
    cluster->cov.m[2][2] = -2 * log(sqrt(cluster->m[2] * cluster->m[2] +
                                         cluster->m[3] * cluster->m[3]));

    // printf("cluster %d %d %f (%f %f %f)\n", i, cluster->count,
    // cluster->weight, cluster->mean.v[0], cluster->mean.v[1],
    // cluster->mean.v[2]);
    // pf_matrix_fprintf(cluster->cov, stdout, "%e");
  }

  assert(fabs(weight) >= DBL_EPSILON);
  if (fabs(weight) < DBL_EPSILON) {
    printf("ERROR : divide-by-zero exception : weight is zero\n");
    return;
  }
  // Compute overall filter stats
  set->mean.v[0] = m[0] / weight;
  set->mean.v[1] = m[1] / weight;
  set->mean.v[2] = atan2(m[3], m[2]);

  // Covariance in linear components
  for (j = 0; j < 2; j++)
    for (k = 0; k < 2; k++)
      set->cov.m[j][k] = c[j][k] / weight - set->mean.v[j] * set->mean.v[k];

  // Covariance in angular components; I think this is the correct
  // formula for circular statistics.
  set->cov.m[2][2] = -2 * log(sqrt(m[2] * m[2] + m[3] * m[3]));

  return;
}

void pf_set_selective_resampling(pf_t *pf, int selective_resampling) {
  pf->selective_resampling = selective_resampling;
}

// Compute the CEP statistics (mean and variance).
void pf_get_cep_stats(pf_t *pf, pf_vector_t *mean, double *var) {
  int i;
  double mn, mx, my, mrr;
  pf_sample_set_t *set;
  pf_sample_t *sample;

  set = pf->sets + pf->current_set;

  mn = 0.0;
  mx = 0.0;
  my = 0.0;
  mrr = 0.0;

  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;

    mn += sample->weight;
    mx += sample->weight * sample->pose.v[0];
    my += sample->weight * sample->pose.v[1];
    mrr += sample->weight * sample->pose.v[0] * sample->pose.v[0];
    mrr += sample->weight * sample->pose.v[1] * sample->pose.v[1];
  }

  assert(fabs(mn) >= DBL_EPSILON);
  if (fabs(mn) < DBL_EPSILON) {
    printf("ERROR : divide-by-zero exception : mn is zero\n");
    return;
  }

  mean->v[0] = mx / mn;
  mean->v[1] = my / mn;
  mean->v[2] = 0.0;

  *var = mrr / mn - (mx * mx / (mn * mn) + my * my / (mn * mn));

  return;
}

// Get the statistics for a particular cluster.
//获得一个粒子簇的统计特性：均值/协方差/权重
int pf_get_cluster_stats(pf_t *pf, int clabel, double *weight,
                         pf_vector_t *mean, pf_matrix_t *cov) {
  pf_sample_set_t *set;
  pf_cluster_t *cluster;

  set = pf->sets + pf->current_set;

  if (clabel >= set->cluster_count)
    return 0;

  //注意这里的clabel哦！
  cluster = set->clusters + clabel;

  //这里取出簇的权重，均值和协方差
  *weight = cluster->weight;
  *mean = cluster->mean;
  *cov = cluster->cov;

  return 1;
}
