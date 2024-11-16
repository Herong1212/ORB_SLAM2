/**
 * @file LoopClosing.cc
 * @author guoqing (1337841346@qq.com)
 * @brief 回环检测线程
 * @version 0.1
 * @date 2019-05-05
 *
 * @copyright Copyright (c) 2019
 *
 */

/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#include "LoopClosing.h"

#include "Sim3Solver.h"

#include "Converter.h"

#include "Optimizer.h"

#include "ORBmatcher.h"

#include <mutex>
#include <thread>

namespace ORB_SLAM2
{

    // 构造函数
    LoopClosing::LoopClosing(Map *pMap, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale) : mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
                                                                                                            mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mpMatchedKF(NULL), mLastLoopKFid(0), mbRunningGBA(false), mbFinishedGBA(true),
                                                                                                            mbStopGBA(false), mpThreadGBA(NULL), mbFixScale(bFixScale), mnFullBAIdx(0)
    {
        // 连续性阈值（表示“连续组”的一致性阈值），阈值越高，检测的鲁棒性越强。
        mnCovisibilityConsistencyTh = 3;
    }

    // 设置追踪线程句柄
    void LoopClosing::SetTracker(Tracking *pTracker)
    {
        mpTracker = pTracker;
    }
    // 设置局部建图线程的句柄
    void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper)
    {
        mpLocalMapper = pLocalMapper;
    }

    // note：回环线程主函数：实现主循环，持续检测闭环，并在检测到闭环时执行优化。
    void LoopClosing::Run()
    {
        // 表示闭环检测线程正在运行
        // 当线程结束时，会将 mbFinished 设置为 true。
        mbFinished = false;

        // 线程主循环
        while (1)
        {
            // Step 1 查看闭环检测队列 mlpLoopKeyFrameQueue 中有没有关键帧进来
            // Loopclosing 中的关键帧是 LocalMapping 发送过来的，LocalMapping 是 Tracking 中发过来的
            // 在 LocalMapping 中通过 InsertKeyFrame 将关键帧插入闭环检测队列 mlpLoopKeyFrameQueue
            if (CheckNewKeyFrames())
            {
                // Detect loop candidates and check covisibility consistency
                if (DetectLoop()) // 闭环检测的核心功能
                {
                    // Compute similarity transformation [sR|t]
                    // In the stereo/RGBD case s=1
                    if (ComputeSim3())
                    {
                        // Perform loop fusion and pose graph optimization
                        CorrectLoop();
                    }
                }
            }

            // 查看是否有外部线程请求复位当前线程
            ResetIfRequested();

            // 查看外部线程是否有终止当前线程的请求，如果有的话就跳出这个线程的主函数的主循环
            if (CheckFinish())
                break;

            // usleep(5000);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        // 运行到这里说明有外部线程请求终止当前线程，在这个函数中执行终止当前线程的一些操作
        SetFinish();
    }

    // 将某个关键帧加入到回环检测的过程中，由局部建图线程调用
    void LoopClosing::InsertKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexLoopQueue);
        // ! 这里第0个关键帧不能够参与到回环检测的过程中，因为第0关键帧定义了整个地图的世界坐标系
        if (pKF->mnId != 0)
            mlpLoopKeyFrameQueue.push_back(pKF);
    }

    /**
     * 查看列表中是否有等待被插入的关键帧
     * @return 如果存在，返回true
     */
    bool LoopClosing::CheckNewKeyFrames()
    {
        unique_lock<mutex> lock(mMutexLoopQueue);

        return (!mlpLoopKeyFrameQueue.empty());
    }

    // TODO 作用：闭环线程的第 1 阶段 ——> 闭环检测
    bool LoopClosing::DetectLoop()
    {
        {
            // Step 1 从队列中取出一个关键帧，作为当前检测闭环关键帧
            unique_lock<mutex> lock(mMutexLoopQueue);

            // 从队列头开始取，也就是先取早进来的关键帧
            mpCurrentKF = mlpLoopKeyFrameQueue.front();
            // 取出关键帧后从队列里弹出该关键帧
            mlpLoopKeyFrameQueue.pop_front();
            // 设置当前关键帧不要在优化的过程中被删除 
            mpCurrentKF->SetNotErase();
        }

        //  Step 2：如果距离上次闭环没多久（小于 10 帧），或者 map 中关键帧总共还没有 10 帧，则不进行闭环检测
        //  后者的体现是当 mLastLoopKFid 为 0 的时候
        if (mpCurrentKF->mnId < mLastLoopKFid + 10) // 比如，当前帧 id 为 12，而上一帧为 15，则 12＜15 + 10，就不进行闭环检测了
        {
            mpKeyFrameDB->add(mpCurrentKF);
            mpCurrentKF->SetErase();
            return false;
        }

        // Step 3：遍历当前回环关键帧所有连接（ > 15个共视地图点）关键帧，计算当前关键帧与每个共视关键帧的词袋相似度得分，并得到最低得分 minScore
        const vector<KeyFrame *> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
        const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;
        float minScore = 1;

        for (size_t i = 0; i < vpConnectedKeyFrames.size(); i++)
        {
            KeyFrame *pKF = vpConnectedKeyFrames[i];

            if (pKF->isBad())
                continue;

            const DBoW2::BowVector &BowVec = pKF->mBowVec;

            // 计算两个关键帧的相似度得分；得分越低，相似度越低
            float score = mpORBVocabulary->score(CurrentBowVec, BowVec);

            // 更新最低得分
            if (score < minScore)
                minScore = score;
        }

        // Step 4：在所有关键帧中找出闭环候选帧（注意：不和当前帧连接）
        // minScore 的作用：认为和当前关键帧具有回环关系的关键帧，不应该低于当前关键帧的相邻关键帧的最低的相似度 minScore
        // 得到的这些关键帧，和当前关键帧具有较多的公共单词，并且相似度评分都挺高
        vector<KeyFrame *> vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidates(mpCurrentKF, minScore);

        // 如果没有闭环候选帧，只需把关键帧添加到关键帧数据库，并返回 false
        if (vpCandidateKFs.empty())
        {
            mpKeyFrameDB->add(mpCurrentKF);
            mvConsistentGroups.clear();
            mpCurrentKF->SetErase();
            return false;
        }

        // Step 5：在候选帧中检测具有连续性的候选帧
        // 1、每个候选帧将与自己相连的关键帧构成一个【子候选组 spCandidateGroup】， vpCandidateKFs --> spCandidateGroup
        // 2、检测“子候选组”中每一个关键帧是否存在于“连续组”，如果存在 nCurrentConsistency++，则将该“子候选组”放入“当前连续组vCurrentConsistentGroups”
        // 3、如果 nCurrentConsistency 大于等于3，那么该”子候选组“代表的候选帧过关，进入 mvpEnoughConsistentCandidates

        // 相关的概念说明：（为方便理解，见视频里的图示）
        // 【组(group)】：对于某个关键帧，其和其具有共视关系的关键帧组成了一个"组"；
        // 【子候选组(CandidateGroup)】：对于某个候选的回环关键帧，其和其具有共视关系的关键帧组成的一个"组"；
        // 【连续性(Consistency)】：不同的组之间如果共同拥有【一个及以上】的关键帧，那么称这两个组之间具有连续关系；
        // 【连续长度】：表示累计的连续的链的长度：A--B 为 1, A--B--C--D 为 3 等；具体反映在数据类型 ConsistentGroup.second 上；
        // 【连续组(Consistent group)】：mvConsistentGroups 存储了上次执行回环检测时, 新的被检测出来的具有连续性的多个组的集合。由于组之间的连续关系是个网状结构，因此可能存在
        //                              一个组因为和不同的连续组链都具有连续关系，而被添加两次的情况(当然连续性度量是不相同的)；
        // 【连续组链】：自造的称呼，类似于菊花链 A--B--C--D 这样形成了一条连续组链。对于这个例子中，由于可能 E，F 都和 D 有连续关系，因此连续组链会产生分叉；为了简化计算，连续组中将只会保存
        //              最后形成连续关系的连续组们(见下面的连续组的更新)；
        // 【子连续组】: 上面的连续组中的一个组；
        // 【连续组的初始值】: 在遍历某个候选帧的过程中，如果该子候选组没有能够和任何一个上次的子连续组产生连续关系,那么就将添加自己组为连续组,并且连续性为0(相当于新开了一个连续链)；
        // 【连续组的更新】: 当前次回环检测过程中，所有被检测到和之前的连续组链有连续的关系的组，都将在对应的连续组链后面+1，这些子候选组(可能有重复,见上)都将会成为新的连续组，
        //                  换而言之，连续组 mvConsistentGroups 中只保存连续组链中末尾的组；

        // 记录最终筛选后得到的闭环帧，先清空
        mvpEnoughConsistentCandidates.clear();

        // ConsistentGroup 数据类型为 pair<set<KeyFrame*>,int>
        // ConsistentGroup.first 对应每个“连续组”中的关键帧集合，ConsistentGroup.second 为每个“连续组”的已连续几个的序号（连续长度）

        vector<ConsistentGroup> vCurrentConsistentGroups;

        // mvConsistentGroups 记录上次闭环检测的连续组链
        // vConsistentGroups 记录上次闭环连续组链中子连续组是否和当前候选组相连（有共同关键帧）
        // 这个下标是每个"子连续组"的下标, bool 表示当前的候选组中是否有和该组相同的一个关键帧
        vector<bool> vbConsistentGroup(mvConsistentGroups.size(), false);

        // Step 5.1：遍历刚才得到的每一个候选关键帧
        for (size_t i = 0, iend = vpCandidateKFs.size(); i < iend; i++)
        {
            KeyFrame *pCandidateKF = vpCandidateKFs[i];

            // Step 5.2：将候选关键帧及其相连的关键帧构成一个【子候选组】
            set<KeyFrame *> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
            // 把候选关键帧也加进去
            spCandidateGroup.insert(pCandidateKF);

            // 连续性达标的标志
            bool bEnoughConsistent = false;
            // 是否产生了连续关系
            bool bConsistentForSomeGroup = false;

            // Step 5.3：遍历前一次闭环检测到的连续组链
            // 上一次闭环检测的连续组链 ConsistentGroup 的数据结构 ——> std::vector<ConsistentGroup> mvConsistentGroups
            // 其中 ConsistentGroup 的定义：typedef pair<set<KeyFrame*>,int> ConsistentGroup
            // 其中 ConsistentGroup.first： 对应每个“子连续组”中的关键帧集合，ConsistentGroup.second： 为每个“连续组”的连续长度
            for (size_t iG = 0, iendG = mvConsistentGroups.size(); iG < iendG; iG++)
            {
                // 取出上次闭环检测中的一个子连续组中的关键帧集合
                set<KeyFrame *> sPreviousGroup = mvConsistentGroups[iG].first;

                // Step 5.4：遍历每个“子候选组”，检测子候选组中每一个关键帧在“子连续组”中是否存在
                // 如果有一帧共同存在于“子候选组”与之前的“子连续组”，那么“子候选组”与该“子连续组”连续
                bool bConsistent = false;
                for (set<KeyFrame *>::iterator sit = spCandidateGroup.begin(), send = spCandidateGroup.end(); sit != send; sit++)
                {
                    if (sPreviousGroup.count(*sit))
                    {
                        // 如果存在，该“子候选组”与该“子连续组”相连
                        bConsistent = true;
                        // 该“子候选组”至少与一个”子连续组“相连，跳出循环
                        bConsistentForSomeGroup = true;
                        break;
                    }
                }

                if (bConsistent)
                {
                    // Step 5.5：如果判定为连续，接下来判断是否达到连续的条件
                    // 取出和当前的子候选组发生"连续"关系的上次闭环中的子连续组的"已连续次数"（连续长度）
                    int nPreviousConsistency = mvConsistentGroups[iG].second;
                    // 将当前子候选组的连续长度在上次闭环的子连续组的连续长度基础上 +1，即👇
                    int nCurrentConsistency = nPreviousConsistency + 1;
                    // 如果上述连续关系还未记录到 vCurrentConsistentGroups，那么记录一下
                    // 注意，这里 spCandidateGroup 可能放置在 vbConsistentGroup 中其他索引(iG)下
                    if (!vbConsistentGroup[iG])
                    {
                        // 将该“子候选组”的该关键帧打上连续编号加入到“当前连续组”
                        ConsistentGroup cg = make_pair(spCandidateGroup, nCurrentConsistency);
                        // 放入本次闭环检测的连续组 vCurrentConsistentGroups 里
                        vCurrentConsistentGroups.push_back(cg);
                        //  标记一下，防止重复添加到同一个索引 iG
                        //  但是 spCandidateGroup 可能重复添加到不同的索引 iG 对应的 vbConsistentGroup 中
                        vbConsistentGroup[iG] = true;
                    }
                    // 如果连续长度满足要求【 >= 3 】，且还没有其它子候选组达到连续长度要求，那么当前的这个候选关键帧是足够靠谱的，即成功连续。
                    // 连续性阈值 mnCovisibilityConsistencyTh = 3
                    // 足够连续的标记 bEnoughConsistent
                    if (nCurrentConsistency >= mnCovisibilityConsistencyTh && !bEnoughConsistent)
                    {
                        // 记录达到连续条件的子候选组
                        mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                        //  标记一下，防止重复添加
                        bEnoughConsistent = true;

                        // ? 这里可以 break 掉结束当前 for 循环吗？
                        // 回答：不行。因为虽然 pCandidateKF 达到了连续性要求
                        // 但 spCandidateGroup 还可以和 mvConsistentGroups 中其他的子连续组进行连接
                    }
                }
            }

            // Step 5.6：如果该“子候选组”的所有关键帧都和上次闭环无关（不连续），则 vCurrentConsistentGroups 不新添加连续关系
            // 于是，就把“子候选组”全部拷贝到 vCurrentConsistentGroups， 用于更新 mvConsistentGroups，连续性计数器设为 0
            if (!bConsistentForSomeGroup)
            {
                ConsistentGroup cg = make_pair(spCandidateGroup, 0);
                vCurrentConsistentGroups.push_back(cg);
            }
        } // 遍历得到的初级的候选关键帧

        // 更新连续组链
        mvConsistentGroups = vCurrentConsistentGroups;

        // 当前闭环检测的关键帧添加到关键帧数据库中
        mpKeyFrameDB->add(mpCurrentKF);

        if (mvpEnoughConsistentCandidates.empty())
        {
            // 未检测到闭环，返回 false
            mpCurrentKF->SetErase();
            return false;
        }
        else
        {
            // 成功检测到闭环，返回 true
            return true;
        }

        // 多余的代码，执行不到
        mpCurrentKF->SetErase();

        return false;
    }

    // TODO 作用：闭环线程的第 2 阶段 ——> 计算 Sim(3)
    /**
     * @brief 计算当前关键帧和上一步闭环候选帧的Sim3变换
     * 1. 遍历闭环候选帧集，筛选出与当前帧的匹配特征点数大于20的候选帧集合，并为每一个候选帧构造一个Sim3Solver
     * 2. 对每一个候选帧进行 Sim3Solver 迭代匹配，直到有一个候选帧匹配成功，或者全部失败
     * 3. 取出闭环匹配上关键帧的相连关键帧，得到它们的地图点放入 mvpLoopMapPoints
     * 4. 将闭环匹配上关键帧以及相连关键帧的地图点投影到当前关键帧进行投影匹配
     * 5. 判断当前帧与检测出的所有闭环关键帧是否有足够多的地图点匹配
     * 6. 清空mvpEnoughConsistentCandidates
     * @return true         只要有一个候选关键帧通过Sim3的求解与优化，就返回true
     * @return false        所有候选关键帧与当前关键帧都没有有效Sim3变换
     */
    bool LoopClosing::ComputeSim3()
    {
        // Sim3 计算流程说明：
        // 1. 通过 Bow 加速描述子的匹配，利用 RANSAC 粗略地计算出当前帧与闭环帧的 Sim3（当前帧---闭环帧）
        // 2. 根据估计的 Sim3，对3D点进行投影找到更多匹配，通过优化的方法计算更精确的 Sim3（当前帧---闭环帧）
        // 3. 将闭环帧以及闭环帧相连的关键帧的地图点与当前帧的点进行匹配（当前帧---闭环帧+相连关键帧）
        // 注意以上匹配的结果均都存在成员变量 mvpCurrentMatchedPoints 中，实际的更新步骤见 CorrectLoop() 步骤3
        // 对于双目或者是 RGB-D 输入的情况,计算得到的 尺度因子 =1

        //  准备工作
        // 对每个（上一步得到的具有足够连续关系的）闭环候选帧都准备算一个 Sim3
        const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

        // We compute first ORB matches for each candidate
        // If enough matches are found, we setup a Sim3Solver
        ORBmatcher matcher(0.75, true);

        // 存储每一个候选帧的Sim3Solver求解器
        vector<Sim3Solver *> vpSim3Solvers;
        vpSim3Solvers.resize(nInitialCandidates);

        // 存储每个候选帧的匹配地图点信息
        vector<vector<MapPoint *>> vvpMapPointMatches;
        vvpMapPointMatches.resize(nInitialCandidates);

        // 存储每个候选帧应该被放弃(True）或者 保留(False)
        vector<bool> vbDiscarded;
        vbDiscarded.resize(nInitialCandidates);

        // 完成 Step 1 的匹配后，被保留的候选帧数量
        int nCandidates = 0;

        // Step 1. 遍历闭环候选帧集，初步筛选出与当前关键帧的匹配特征点数大于20的候选帧集合，并为每一个候选帧构造一个Sim3Solver
        for (int i = 0; i < nInitialCandidates; i++)
        {
            // Step 1.1 从筛选的闭环候选帧中取出一帧有效关键帧pKF
            KeyFrame *pKF = mvpEnoughConsistentCandidates[i];

            // 避免在LocalMapping中KeyFrameCulling函数将此关键帧作为冗余帧剔除
            pKF->SetNotErase();

            // 如果候选帧质量不高，直接PASS
            if (pKF->isBad())
            {
                vbDiscarded[i] = true;
                continue;
            }

            // Step 1.2 将当前帧 mpCurrentKF 与闭环候选关键帧pKF匹配
            // 通过bow加速得到 mpCurrentKF 与 pKF 之间的匹配特征点
            // vvpMapPointMatches 是匹配特征点对应的地图点,本质上来自于候选闭环帧
            int nmatches = matcher.SearchByBoW(mpCurrentKF, pKF, vvpMapPointMatches[i]);

            // 粗筛：匹配的特征点数太少，该候选帧剔除
            if (nmatches < 20)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                // Step 1.3 为保留的候选帧构造Sim3求解器
                // 如果 mbFixScale（是否固定尺度） 为 true，则是6 自由度优化（双目 RGBD）
                // 如果是false，则是7 自由度优化（单目）
                Sim3Solver *pSolver = new Sim3Solver(mpCurrentKF, pKF, vvpMapPointMatches[i], mbFixScale);

                // Sim3Solver Ransac 过程置信度0.99，至少20个inliers 最多300次迭代
                pSolver->SetRansacParameters(0.99, 20, 300);
                vpSim3Solvers[i] = pSolver;
            }

            // 保留的候选帧数量
            nCandidates++;
        }

        // 用于标记是否有一个候选帧通过Sim3Solver的求解与优化
        bool bMatch = false;

        // Step 2 对每一个候选帧用Sim3Solver 迭代匹配，直到有一个候选帧匹配成功，或者全部失败
        while (nCandidates > 0 && !bMatch)
        {
            // 遍历每一个候选帧
            for (int i = 0; i < nInitialCandidates; i++)
            {
                if (vbDiscarded[i])
                    continue;

                KeyFrame *pKF = mvpEnoughConsistentCandidates[i];

                // 内点（Inliers）标志
                // 即标记经过RANSAC sim3 求解后,vvpMapPointMatches中的哪些作为内点
                vector<bool> vbInliers;

                // 内点（Inliers）数量
                int nInliers;

                // 是否到达了最优解
                bool bNoMore;

                // Step 2.1 取出从 Step 1.3 中为当前候选帧构建的 Sim3Solver 并开始迭代
                Sim3Solver *pSolver = vpSim3Solvers[i];

                // 最多迭代5次，返回的Scm是候选帧pKF到当前帧mpCurrentKF的Sim3变换（T12）
                cv::Mat Scm = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

                // If Ransac reachs max. iterations discard keyframe
                // 总迭代次数达到最大限制还没有求出合格的Sim3变换，该候选帧剔除
                if (bNoMore)
                {
                    vbDiscarded[i] = true;
                    nCandidates--;
                }

                // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
                // 如果计算出了Sim3变换，继续匹配出更多点并优化。因为之前 SearchByBoW 匹配可能会有遗漏
                if (!Scm.empty())
                {
                    // 取出经过Sim3Solver 后匹配点中的内点集合
                    vector<MapPoint *> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPoint *>(NULL));
                    for (size_t j = 0, jend = vbInliers.size(); j < jend; j++)
                    {
                        // 保存内点
                        if (vbInliers[j])
                            vpMapPointMatches[j] = vvpMapPointMatches[i][j];
                    }

                    // Step 2.2 通过上面求取的Sim3变换引导关键帧匹配，弥补Step 1中的漏匹配
                    // 候选帧pKF到当前帧mpCurrentKF的R（R12），t（t12），变换尺度s（s12）
                    cv::Mat R = pSolver->GetEstimatedRotation();
                    cv::Mat t = pSolver->GetEstimatedTranslation();
                    const float s = pSolver->GetEstimatedScale();

                    // 查找更多的匹配（成功的闭环匹配需要满足足够多的匹配特征点数，之前使用SearchByBoW进行特征点匹配时会有漏匹配）
                    // 通过Sim3变换，投影搜索pKF1的特征点在pKF2中的匹配，同理，投影搜索pKF2的特征点在pKF1中的匹配
                    // 只有互相都成功匹配的才认为是可靠的匹配
                    matcher.SearchBySim3(mpCurrentKF, pKF, vpMapPointMatches, s, R, t, 7.5);

                    // Step 2.3 用新的匹配来优化 Sim3，只要有一个候选帧通过Sim3的求解与优化，就跳出停止对其它候选帧的判断
                    // OpenCV的Mat矩阵转成Eigen的Matrix类型
                    // gScm：候选关键帧到当前帧的Sim3变换
                    g2o::Sim3 gScm(Converter::toMatrix3d(R), Converter::toVector3d(t), s);

                    // 如果mbFixScale为true，则是6 自由度优化（双目 RGBD），如果是false，则是7 自由度优化（单目）
                    // 优化mpCurrentKF与pKF对应的MapPoints间的Sim3，得到优化后的量gScm
                    const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10, mbFixScale);

                    // 如果优化成功，则停止while循环遍历闭环候选
                    if (nInliers >= 20)
                    {
                        // 为True时将不再进入 while循环
                        bMatch = true;
                        // mpMatchedKF就是最终闭环检测出来与当前帧形成闭环的关键帧
                        mpMatchedKF = pKF;

                        // gSmw：从世界坐标系 w 到该候选帧 m 的Sim3变换，都在一个坐标系下，所以尺度 Scale=1
                        g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()), Converter::toVector3d(pKF->GetTranslation()), 1.0);

                        // 得到g2o优化后从世界坐标系到当前帧的Sim3变换
                        mg2oScw = gScm * gSmw;
                        mScw = Converter::toCvMat(mg2oScw);
                        mvpCurrentMatchedPoints = vpMapPointMatches;

                        // 只要有一个候选帧通过Sim3的求解与优化，就跳出停止对其它候选帧的判断
                        break;
                    }
                }
            }
        }

        // 退出上面 while 循环的原因有两种,一种是求解到了 bMatch 置位后出的,另外一种是 nCandidates 耗尽为 0
        if (!bMatch)
        {
            // 如果没有一个闭环匹配候选帧通过 Sim3 的求解与优化
            // 清空 mvpEnoughConsistentCandidates，这些候选关键帧以后都不会在再参加回环检测过程了
            for (int i = 0; i < nInitialCandidates; i++)
                mvpEnoughConsistentCandidates[i]->SetErase();
            // 当前关键帧也将不会再参加回环检测了
            mpCurrentKF->SetErase();
            // Sim3 计算失败，退出了
            return false;
        }

        // Step 3：取出与当前帧闭环匹配上的关键帧及其共视关键帧，以及这些共视关键帧的地图点
        // 注意是闭环检测出来与当前帧形成闭环的关键帧 mpMatchedKF
        // 将mpMatchedKF共视的关键帧全部取出来放入 vpLoopConnectedKFs
        // 将vpLoopConnectedKFs的地图点取出来放入mvpLoopMapPoints
        vector<KeyFrame *> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();

        // 包含闭环匹配关键帧本身,形成一个“闭环关键帧小组“
        vpLoopConnectedKFs.push_back(mpMatchedKF);
        mvpLoopMapPoints.clear();

        // 遍历这个组中的每一个关键帧
        for (vector<KeyFrame *>::iterator vit = vpLoopConnectedKFs.begin(); vit != vpLoopConnectedKFs.end(); vit++)
        {
            KeyFrame *pKF = *vit;
            vector<MapPoint *> vpMapPoints = pKF->GetMapPointMatches();

            // 遍历其中一个关键帧的所有有效地图点
            for (size_t i = 0, iend = vpMapPoints.size(); i < iend; i++)
            {
                MapPoint *pMP = vpMapPoints[i];
                if (pMP)
                {
                    // mnLoopPointForKF 用于标记，避免重复添加
                    if (!pMP->isBad() && pMP->mnLoopPointForKF != mpCurrentKF->mnId)
                    {
                        mvpLoopMapPoints.push_back(pMP);
                        // 标记一下
                        pMP->mnLoopPointForKF = mpCurrentKF->mnId;
                    }
                }
            }
        }

        // Find more matches projecting with the computed Sim3
        // Step 4：将闭环关键帧及其连接关键帧的所有地图点投影到当前关键帧进行投影匹配
        // 根据投影查找更多的匹配（成功的闭环匹配需要满足足够多的匹配特征点数）
        // 根据 Sim3 变换，将每个 mvpLoopMapPoints 投影到 mpCurrentKF上，搜索新的匹配对
        // mvpCurrentMatchedPoints 是前面经过 SearchBySim3 得到的已经匹配的点对，这里就忽略不再匹配了
        // 搜索范围系数为【10】
        matcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints, 10);

        // Step 5: 统计当前帧与闭环关键帧的匹配地图点数目，超过【40】个说明成功闭环，否则失败
        int nTotalMatches = 0;
        for (size_t i = 0; i < mvpCurrentMatchedPoints.size(); i++)
        {
            if (mvpCurrentMatchedPoints[i])
                nTotalMatches++;
        }

        if (nTotalMatches >= 40)
        {
            // 如果当前回环可靠,保留当前待闭环关键帧，其他闭环候选全部删掉以后不用了
            for (int i = 0; i < nInitialCandidates; i++)
                if (mvpEnoughConsistentCandidates[i] != mpMatchedKF)
                    mvpEnoughConsistentCandidates[i]->SetErase();
            return true;
        }
        else
        {
            // 闭环不可靠，闭环候选及当前待闭环帧全部删除
            for (int i = 0; i < nInitialCandidates; i++)
                mvpEnoughConsistentCandidates[i]->SetErase();
            mpCurrentKF->SetErase();
            return false;
        }
    }

    // TODO 作用：闭环线程的第 2 阶段 ——> 闭环矫正
    /**
     * @brief 闭环矫正
     * 1. 通过求解的 Sim3 以及相对姿态关系，调整与当前帧相连的关键帧位姿以及这些关键帧观测到的地图点位置（相连关键帧---当前帧）
     * 2. 将闭环帧以及闭环帧相连的关键帧的地图点和与当前帧相连的关键帧的点进行匹配（当前帧 + 相连关键帧---闭环帧 + 相连关键帧）
     * 3. 通过 MapPoints 的匹配关系更新这些帧之间的连接关系，即更新 covisibility graph
     * 4. 对 Essential Graph（Pose Graph）进行优化，MapPoints 的位置则根据优化后的位姿做相对应的调整
     * 5. 创建线程进行全局 Bundle Adjustment
     */
    void LoopClosing::CorrectLoop()
    {

        cout << "Loop detected!" << endl;
        // Step 0：结束局部地图线程、全局 BA，为闭环矫正做准备
        // Step 1：根据【共视关系】更新当前帧与其它关键帧之间的连接
        // Step 2：通过【位姿传播】，得到 Sim3 优化后，与当前帧相连的关键帧的位姿，以及它们的 MapPoints
        // Step 3：检查当前帧的 MapPoints 与闭环匹配后该帧的 MapPoints 是否存在冲突，对冲突的 MapPoints 进行替换或填补
        // Step 4：通过将闭环相连关键帧组中的 所有地图点(mvpLoopMapPoints) 投影到当前关键帧组中，进行 MapPoints 检查与替换
        // Step 5：更新当前关键帧之间的共视相连关系，得到因闭环时 MapPoints 融合而新得到的连接关系
        // Step 6：进行 EssentialGraph 优化，LoopConnections 是形成闭环后新生成的连接关系，不包括步骤 7 中当前帧与闭环匹配帧之间的连接关系
        // Step 7：添加当前帧与闭环匹配帧之间的边（这个连接关系不优化）
        // Step 8：新建一个线程用于全局BA优化

        // g2oSic： 当前关键帧 mpCurrentKF 到其共视关键帧 pKFi 的 Sim3 相对变换
        // mg2oScw: 世界坐标系到当前关键帧的 Sim3 变换
        // g2oCorrectedSiw：世界坐标系到当前关键帧共视关键帧的 Sim3 变换

        // Step 0：结束局部地图线程、全局BA，为闭环矫正做准备
        // 请求局部地图停止，防止在回环矫正时局部地图线程中 InsertKeyFrame 函数插入新的关键帧
        mpLocalMapper->RequestStop();

        if (isRunningGBA())
        {
            // 如果有全局 BA 在运行，终止掉，迎接新的全局 BA
            unique_lock<mutex> lock(mMutexGBA);
            mbStopGBA = true;
            // 记录全局 BA 次数
            mnFullBAIdx++;
            if (mpThreadGBA)
            {
                // 停止全局 BA 线程
                mpThreadGBA->detach();
                delete mpThreadGBA;
            }
        }

        // 一直等到局部地图线程结束再继续
        while (!mpLocalMapper->isStopped())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // Step 1：根据【共视关系】更新当前关键帧与其它关键帧之间的连接关系
        // Ensure current keyframe is updated
        // 因为之前闭环检测、计算 Sim3 中改变了该关键帧的地图点，所以需要更新
        mpCurrentKF->UpdateConnections();

        // Step 2：通过【位姿传播】，得到 Sim3 优化后，与当前帧相连的关键帧的位姿，以及它们的地图点
        // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
        // 当前帧与世界坐标系之间的 Sim 变换在 ComputeSim3 函数中已经确定并优化，
        // 通过相对位姿关系，可以确定这些相连的关键帧与世界坐标系之间的 Sim3 变换

        // 取出当前关键帧及其共视关键帧，称为【当前关键帧组】
        mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
        mvpCurrentConnectedKFs.push_back(mpCurrentKF);

        // CorrectedSim3：存放【闭环 g2o 优化】后【当前关键帧】的【共视关键帧】的世界坐标系下 Sim3 变换
        // NonCorrectedSim3：存放【没有矫正】的【当前关键帧】的【共视关键帧】的世界坐标系下 Sim3 变换
        KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;

        // 先将 mpCurrentKF 的 Sim3 变换存入，认为是准的，所以固定不动
        CorrectedSim3[mpCurrentKF] = mg2oScw;

        // 当前关键帧到世界坐标系下的变换矩阵
        cv::Mat Twc = mpCurrentKF->GetPoseInverse();

        // 对地图点操作
        {
            // 锁定地图点
            unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

            // Boundary ------------------------------------------------------------------------------------------------------------------------------

            // todo 重要操作1：Sim3 位姿传播和矫正
            // Step 2.1：通过 mg2oScw（认为是准的）来进行位姿传播，得到当前关键帧的共视关键帧的世界坐标系下的 Sim3 位姿
            // 遍历"当前关键帧组""
            for (vector<KeyFrame *>::iterator vit = mvpCurrentConnectedKFs.begin(), vend = mvpCurrentConnectedKFs.end(); vit != vend; vit++)
            {
                KeyFrame *pKFi = *vit;
                cv::Mat Tiw = pKFi->GetPose();
                if (pKFi != mpCurrentKF) // 跳过当前关键帧，因为当前关键帧的位姿已经在前面优化过了，在这里是参考基准
                {
                    // 得到当前关键帧 mpCurrentKF 到其共视关键帧 pKFi 的相对变换
                    cv::Mat Tic = Tiw * Twc;
                    cv::Mat Ric = Tic.rowRange(0, 3).colRange(0, 3);
                    cv::Mat tic = Tic.rowRange(0, 3).col(3);

                    // g2oSic：当前关键帧 mpCurrentKF 到其共视关键帧 pKFi 的Sim3 相对变换
                    // 这里是 non-correct, 所以 scale = 1.0
                    g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric), Converter::toVector3d(tic), 1.0);

                    // 当前帧的位姿固定不动，其它的关键帧根据相对关系得到 Sim3 调整的位姿
                    // 基本原理：用准确的 mg2oScw 表示世界坐标系下 pKFi 的 Sim(3) 变换
                    g2o::Sim3 g2oCorrectedSiw = g2oSic * mg2oScw;

                    // Pose corrected with the Sim3 of the loop closure
                    // 存放闭环 g2o 优化后当前关键帧的共视关键帧的 Sim3 位姿
                    CorrectedSim3[pKFi] = g2oCorrectedSiw;
                }

                cv::Mat Riw = Tiw.rowRange(0, 3).colRange(0, 3);
                cv::Mat tiw = Tiw.rowRange(0, 3).col(3);
                g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw), Converter::toVector3d(tiw), 1.0);

                // 存放没有矫正的当前关键帧的共视关键帧的 Sim3 变换
                NonCorrectedSim3[pKFi] = g2oSiw;
            }

            // Boundary ------------------------------------------------------------------------------------------------------------------------------

            // todo 重要操作2：地图点坐标 传播和矫正
            // Step 2.2：得到矫正的当前关键帧的共视关键帧位姿后，修正这些共视关键帧的地图点
            // 遍历待矫正的共视关键帧（不包括当前关键帧）
            for (KeyFrameAndPose::iterator mit = CorrectedSim3.begin(), mend = CorrectedSim3.end(); mit != mend; mit++)
            {
                // 取出当前关键帧连接关键帧
                KeyFrame *pKFi = mit->first;
                // 取出【经过位姿传播】后的 Sim3 变换
                g2o::Sim3 g2oCorrectedSiw = mit->second;
                g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();
                // 取出【未经过位姿传播】的 Sim3 变换
                g2o::Sim3 g2oSiw = NonCorrectedSim3[pKFi];

                vector<MapPoint *> vpMPsi = pKFi->GetMapPointMatches();

                // 遍历待矫正共视关键帧中的每一个地图点
                for (size_t iMP = 0, endMPi = vpMPsi.size(); iMP < endMPi; iMP++)
                {
                    MapPoint *pMPi = vpMPsi[iMP];
                    // 跳过无效的地图点
                    if (!pMPi)
                        continue;
                    if (pMPi->isBad())
                        continue;
                    // 标记，防止重复矫正
                    if (pMPi->mnCorrectedByKF == mpCurrentKF->mnId)
                        continue;

                    // 矫正过程本质上也是基于当前关键帧的优化后的位姿展开的
                    // 将该未校正的 eigP3Dw 先从世界坐标系映射到【未校正】的 pKFi 相机坐标系，然后再反映射到【校正后】的世界坐标系下
                    cv::Mat P3Dw = pMPi->GetWorldPos();
                    // 地图点世界坐标系下坐标
                    Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(P3Dw);
                    // map(P) 内部做了相似变换 s*R*P +t

                    // 下面变换是：eigP3Dw： world → g2oSiw → i → g2oCorrectedSwi → world
                    Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

                    cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
                    pMPi->SetWorldPos(cvCorrectedP3Dw);

                    // 记录矫正该地图点的关键帧 id，防止重复
                    pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                    // 记录该地图点所在的关键帧 id
                    pMPi->mnCorrectedReference = pKFi->mnId;
                    // 因为地图点更新了，需要更新其平均观测方向以及观测距离范围
                    pMPi->UpdateNormalAndDepth();
                }

                // Step 2.3：将共视关键帧的 Sim3 转换为 SE3，根据更新的 Sim3，更新关键帧的位姿
                // 其实是现在已经有了更新后的关键帧组中关键帧的位姿,但是在上面的操作时只是暂时存储到了 KeyFrameAndPose 类型的变量中,还没有写回到关键帧对象中
                // 调用 toRotationMatrix 可以自动归一化旋转矩阵
                Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
                Eigen::Vector3d eigt = g2oCorrectedSiw.translation();

                double s = g2oCorrectedSiw.scale();

                // 平移向量中包含有尺度信息，还需要用尺度归一化
                eigt *= (1. / s);

                cv::Mat correctedTiw = Converter::toCvSE3(eigR, eigt);
                // 设置矫正后的新的 SE(3) 的 pose
                pKFi->SetPose(correctedTiw);

                // Step 2.4：根据共视关系更新当前帧与其它关键帧之间的连接
                // 地图点的位置改变了,可能会引起共视关系（权值）的改变
                pKFi->UpdateConnections();
            }

            // Boundary ------------------------------------------------------------------------------------------------------------------------------

            // Step 3：检查当前帧的地图点与经过闭环匹配后该帧的地图点是否存在冲突，对冲突的地图点进行替换或填补
            // mvpCurrentMatchedPoints 是当前关键帧和闭环关键帧组的所有地图点进行投影得到的匹配点
            for (size_t i = 0; i < mvpCurrentMatchedPoints.size(); i++)
            {
                if (mvpCurrentMatchedPoints[i])
                {
                    // 取出同一个索引对应的两种地图点，决定是否要替换
                    // 匹配投影得到的地图点
                    MapPoint *pLoopMP = mvpCurrentMatchedPoints[i];
                    // 原来的地图点
                    MapPoint *pCurMP = mpCurrentKF->GetMapPoint(i);
                    if (pCurMP)
                        // 如果有重复的 MapPoint，则用匹配的地图点代替现有的
                        // 因为匹配的地图点是经过一系列操作后比较精确的，现有的地图点很可能有累计误差
                        pCurMP->Replace(pLoopMP);
                    else
                    {
                        // 如果当前帧没有该 MapPoint，则直接添加
                        mpCurrentKF->AddMapPoint(pLoopMP, i);
                        pLoopMP->AddObservation(mpCurrentKF, i);
                        pLoopMP->ComputeDistinctiveDescriptors();
                    }
                }
            }
        }

        // Project MapPoints observed in the neighborhood of the loop keyframe
        // into the current keyframe and neighbors using corrected poses.
        // Fuse duplications.
        // Step 4：将闭环相连关键帧组中的所有地图点 mvpLoopMapPoints 投影到当前关键帧组中，进行匹配，融合，新增或替换当前关键帧组中 KF 的地图点
        // 因为 闭环相连关键帧组 mvpLoopMapPoints 在地图中时间比较久，经历了多次优化，认为是准确的
        // 而当前关键帧组中的关键帧的地图点是最近新计算的，可能有累积误差
        // CorrectedSim3：存放矫正后当前关键帧的共视关键帧，及其世界坐标系下 Sim3 变换
        SearchAndFuse(CorrectedSim3);

        // Step 5：更新当前关键帧组之间的两级共视相连关系，得到因闭环时地图点融合而新得到的连接关系
        // LoopConnections：存储因为闭环时地图点调整而新生成的连接关系
        map<KeyFrame *, set<KeyFrame *>> LoopConnections;

        // Step 5.1：遍历当前帧相连关键帧组（一级相连）
        for (vector<KeyFrame *>::iterator vit = mvpCurrentConnectedKFs.begin(), vend = mvpCurrentConnectedKFs.end(); vit != vend; vit++)
        {
            KeyFrame *pKFi = *vit;

            // Step 5.2：得到与当前帧相连关键帧的相连关键帧（二级相连）
            vector<KeyFrame *> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

            // Update connections. Detect new links.
            // Step 5.3：更新一级相连关键帧的连接关系 (会把当前关键帧添加进去, 因为地图点已经更新和替换了)
            pKFi->UpdateConnections();

            // Step 5.4：取出该帧更新后的连接关系
            LoopConnections[pKFi] = pKFi->GetConnectedKeyFrames();

            // Step 5.5：从连接关系中，去除闭环之前的二级连接关系，剩下的连接就是由闭环得到的连接关系
            for (vector<KeyFrame *>::iterator vit_prev = vpPreviousNeighbors.begin(), vend_prev = vpPreviousNeighbors.end(); vit_prev != vend_prev; vit_prev++)
            {
                LoopConnections[pKFi].erase(*vit_prev);
            }
            // Step 5.6：从连接关系中，去除闭环之前的一级连接关系，剩下的连接就是由闭环得到的连接关系
            for (vector<KeyFrame *>::iterator vit2 = mvpCurrentConnectedKFs.begin(), vend2 = mvpCurrentConnectedKFs.end(); vit2 != vend2; vit2++)
            {
                LoopConnections[pKFi].erase(*vit2);
            }
        }

        // Step 6：进行本质图优化，优化本质图中所有关键帧的位姿和地图点
        // LoopConnections 是形成闭环后新生成的连接关系，不包括步骤 7 中当前帧与闭环匹配帧之间的连接关系
        Optimizer::OptimizeEssentialGraph(mpMap, mpMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, mbFixScale);

        // Add loop edge
        // Step 7：添加当前帧与闭环匹配帧之间的边（这个连接关系不优化）
        // 它在下一次的本质图优化里面使用
        mpMatchedKF->AddLoopEdge(mpCurrentKF);
        mpCurrentKF->AddLoopEdge(mpMatchedKF);

        // Step 8：新建一个线程用于全局BA优化
        // OptimizeEssentialGraph 只是优化了一些主要关键帧的位姿，这里进行全局 BA 可以全局优化所有位姿和 MapPoints
        mbRunningGBA = true;
        mbFinishedGBA = false;
        mbStopGBA = false;
        mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment, this, mpCurrentKF->mnId);

        // Loop closed. Release Local Mapping.
        mpLocalMapper->Release();

        cout << "Loop Closed!" << endl;

        mLastLoopKFid = mpCurrentKF->mnId;
    }

    /**
     * @brief 将闭环相连关键帧组mvpLoopMapPoints 投影到当前关键帧组中，进行匹配，新增或替换当前关键帧组中KF的地图点
     * 因为 闭环相连关键帧组mvpLoopMapPoints 在地图中时间比较久经历了多次优化，认为是准确的
     * 而当前关键帧组中的关键帧的地图点是最近新计算的，可能有累积误差
     *
     * @param[in] CorrectedPosesMap         矫正的当前KF对应的共视关键帧及Sim3变换
     */
    void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap)
    {

        // 定义ORB匹配器
        ORBmatcher matcher(0.8);

        // Step 1 遍历待矫正的当前KF的相连关键帧
        for (KeyFrameAndPose::const_iterator mit = CorrectedPosesMap.begin(), mend = CorrectedPosesMap.end(); mit != mend; mit++)
        {
            KeyFrame *pKF = mit->first;
            // 矫正过的Sim 变换
            g2o::Sim3 g2oScw = mit->second;
            cv::Mat cvScw = Converter::toCvMat(g2oScw);

            // Step 2 将mvpLoopMapPoints投影到pKF帧匹配，检查地图点冲突并融合
            // mvpLoopMapPoints：与当前关键帧闭环匹配上的关键帧及其共视关键帧组成的地图点
            vector<MapPoint *> vpReplacePoints(mvpLoopMapPoints.size(), static_cast<MapPoint *>(NULL));
            // vpReplacePoints：存储mvpLoopMapPoints投影到pKF匹配后需要替换掉的新增地图点,索引和mvpLoopMapPoints一致，初始化为空
            // 搜索区域系数为4
            matcher.Fuse(pKF, cvScw, mvpLoopMapPoints, 4, vpReplacePoints);

            // Get Map Mutex
            // 之所以不在上面 Fuse 函数中进行地图点融合更新的原因是需要对地图加锁
            unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
            const int nLP = mvpLoopMapPoints.size();
            // Step 3 遍历闭环帧组的所有的地图点，替换掉需要替换的地图点
            for (int i = 0; i < nLP; i++)
            {
                MapPoint *pRep = vpReplacePoints[i];
                if (pRep)
                {
                    // 如果记录了需要替换的地图点
                    // 用mvpLoopMapPoints替换掉vpReplacePoints里记录的要替换的地图点
                    pRep->Replace(mvpLoopMapPoints[i]);
                }
            }
        }
    }

    // 由外部线程调用，请求复位当前线程
    void LoopClosing::RequestReset()
    {
        // 标志置位
        {
            unique_lock<mutex> lock(mMutexReset);
            mbResetRequested = true;
        }

        // 堵塞,直到回环检测线程复位完成
        while (1)
        {
            {
                unique_lock<mutex> lock2(mMutexReset);
                if (!mbResetRequested)
                    break;
            }
            // usleep(5000);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    // 当前线程调用，检查是否有外部线程请求复位当前线程，如果有的话就复位回环检测线程
    void LoopClosing::ResetIfRequested()
    {
        unique_lock<mutex> lock(mMutexReset);
        // 如果有来自于外部的线程的复位请求,那么就复位当前线程
        if (mbResetRequested)
        {
            mlpLoopKeyFrameQueue.clear(); // 清空参与和进行回环检测的关键帧队列
            mLastLoopKFid = 0;            // 上一次没有和任何关键帧形成闭环关系
            mbResetRequested = false;     // 复位请求标志复位
        }
    }

    /**
     * @brief 全局 BA 优化线程，并更新所有关键帧位姿和地图点坐标，这个是这个线程的主函数
     *
     * @param[in] nLoopKF 看上去是闭环关键帧 id, 但是在调用的时候给的其实是【当前关键帧】的 id
     */
    // todo 作用：完成闭环矫正后的最后一步 —— 对所有地图点和关键帧位姿进行全局 BA 优化👇
    void LoopClosing::RunGlobalBundleAdjustment(unsigned long nLoopKF)
    {
        cout << "Starting Global Bundle Adjustment" << endl;

        // 记录 GBA 已经迭代次数,用来检查全局 BA 过程是否是因为意外结束的
        int idx = mnFullBAIdx;
        // mbStopGBA 直接传引用过去了,这样当有外部请求的时候这个优化函数能够及时响应并且结束掉
        // 提问：进行完这个过程后我们能够获得哪些信息?
        // 回答：能够得到全部关键帧优化后的位姿,以及优化后的地图点

        // Step 1 执行全局 BA，优化所有的关键帧位姿和地图中地图点
        Optimizer::GlobalBundleAdjustemnt(mpMap,      // 地图点对象
                                          10,         // 迭代次数
                                          &mbStopGBA, // 外界控制 GBA 停止的标志
                                          nLoopKF,    // 形成了闭环的当前关键帧的id
                                          false);     // 不使用鲁棒核函数

        // Update all MapPoints and KeyFrames
        // Local Mapping was active during BA, that means that there might be new keyframes
        // not included in the Global BA and they are not consistent with the updated map.
        // We need to propagate the correction through the spanning tree
        // 更新所有的地图点和关键帧
        // 在 global BA过程中 local mapping 线程仍然在工作，这意味着在 global BA 时可能有新的关键帧产生，但是并未包括在 GBA 里，可能会造成更新后的地图并不连续。
        // 需要通过 spanning tree 来传播
        {
            unique_lock<mutex> lock(mMutexGBA);

            // 如果全局BA过程是因为意外结束的,那么直接退出GBA
            if (idx != mnFullBAIdx)
                return;

            // 如果当前 GBA 没有中断请求，更新位姿和地图点
            // 这里和上面那句话的功能还有些不同,因为如果一次全局优化被中断,往往意味又要重新开启一个新的全局 BA;为了中断当前正在执行的优化过程 mbStopGBA 将会被置位,同时会有一定的时间
            // 使得该线程进行响应; 而在开启一个新的全局优化进程之前 mbStopGBA 将会被置为 False
            // 因此,如果被强行中断的线程退出时已经有新的线程启动了, mbStopGBA=false, 为了避免进行后面的程序,所以有了上面的程序;
            // 而如果被强行中断的线程退出时新的线程还没有启动,那么上面的条件就不起作用了(虽然概率很小,前面的程序中 mbStopGBA 置位后很快 mnFullBAIdx 就 ++ 了,保险起见),所以这里要再判断一次
            if (!mbStopGBA)
            {
                cout << "Global Bundle Adjustment finished" << endl;
                cout << "Updating map ..." << endl;
                mpLocalMapper->RequestStop();

                // 等待直到 local mapping 结束才会继续后续操作
                while (!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())
                {
                    // usleep(1000);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }

                // 后续要更新地图，所以要上锁
                unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

                // 从第一个关键帧开始矫正关键帧。刚开始只保存了初始化第一个关键帧
                list<KeyFrame *> lpKFtoCheck(mpMap->mvpKeyFrameOrigins.begin(), mpMap->mvpKeyFrameOrigins.end());

                // 问：GBA里锁住第一个关键帧位姿没有优化，其对应的pKF->mTcwGBA是不变的吧？那后面调整位姿的意义何在？
                // 回答：注意在前面essential graph BA里只锁住了回环帧，没有锁定第1个初始化关键帧位姿。所以第1个初始化关键帧位姿已经更新了
                // 在GBA里锁住第一个关键帧位姿没有优化，其对应的pKF->mTcwGBA应该是essential BA结果，在这里统一更新了
                // Step 2 遍历并更新全局地图中的所有 spanning tree 中的关键帧
                while (!lpKFtoCheck.empty())
                {
                    KeyFrame *pKF = lpKFtoCheck.front();
                    const set<KeyFrame *> sChilds = pKF->GetChilds();
                    cv::Mat Twc = pKF->GetPoseInverse();
                    // 遍历当前关键帧的子关键帧
                    for (set<KeyFrame *>::const_iterator sit = sChilds.begin(); sit != sChilds.end(); sit++)
                    {
                        KeyFrame *pChild = *sit;
                        // mnBAGlobalForKF 记录是由于哪个闭环匹配关键帧触发的全局 BA,并且已经经过了 GBA 的优化。
                        if (pChild->mnBAGlobalForKF != nLoopKF)
                        {
                            // 从父关键帧到当前子关键帧的位姿变换 T_child_farther
                            cv::Mat Tchildc = pChild->GetPose() * Twc;
                            // 再利用优化后的父关键帧的位姿，转换到世界坐标系下，相当于更新了子关键帧的位姿
                            // 这种最小生成树中除了根节点，其他的节点都会作为其他关键帧的子节点，这样做可以使得最终所有的关键帧都得到了优化
                            pChild->mTcwGBA = Tchildc * pKF->mTcwGBA;
                            // 做个标记，避免重复
                            pChild->mnBAGlobalForKF = nLoopKF;
                        }
                        lpKFtoCheck.push_back(pChild);
                    }
                    // 记录未矫正的关键帧的位姿
                    pKF->mTcwBefGBA = pKF->GetPose();
                    // 记录已经矫正的关键帧的位姿
                    pKF->SetPose(pKF->mTcwGBA);
                    // 从列表中移除
                    lpKFtoCheck.pop_front();
                }

                // Correct MapPoints
                const vector<MapPoint *> vpMPs = mpMap->GetAllMapPoints();

                // Step 3 遍历每一个地图点，并用更新的关键帧位姿来更新地图点位置
                for (size_t i = 0; i < vpMPs.size(); i++)
                {
                    MapPoint *pMP = vpMPs[i];

                    if (pMP->isBad())
                        continue;

                    // 如果这个地图点直接参与到了全局 BA 优化的过程,那么就直接重新设置其位姿即可
                    if (pMP->mnBAGlobalForKF == nLoopKF)
                    {
                        // If optimized by Global BA, just update
                        pMP->SetWorldPos(pMP->mPosGBA);
                    }
                    else
                    {
                        // 如这个地图点并没有直接参与到全局 BA 优化的过程中,那么就使用其【参考关键帧】的新位姿来优化自己的坐标
                        // Update according to the correction of its reference keyframe
                        KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();

                        // 如果参考关键帧并没有经过此次全局 BA 优化，就跳过
                        if (pRefKF->mnBAGlobalForKF != nLoopKF)
                            continue;

                        // 未矫正位姿的相机坐标系下的三维点👇
                        cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0, 3).colRange(0, 3);
                        cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0, 3).col(3);
                        // 转换到其参考关键帧相机坐标系下的坐标👇
                        cv::Mat Xc = Rcw * pMP->GetWorldPos() + tcw;

                        // 然后使用已经纠正过的参考关键帧的位姿,再将该地图点变换到世界坐标系下
                        cv::Mat Twc = pRefKF->GetPoseInverse();
                        cv::Mat Rwc = Twc.rowRange(0, 3).colRange(0, 3);
                        cv::Mat twc = Twc.rowRange(0, 3).col(3);

                        pMP->SetWorldPos(Rwc * Xc + twc);
                    }
                }

                // 释放局部建图线程
                mpLocalMapper->Release();

                cout << "Map updated!" << endl;
            }

            mbFinishedGBA = true;
            mbRunningGBA = false;
        }
    }

    // 由外部线程调用，请求终止当前线程
    void LoopClosing::RequestFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    // 当前线程调用，查看是否有外部线程请求当前线程
    bool LoopClosing::CheckFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);

        return mbFinishRequested;
    }

    // 有当前线程调用，执行完成该函数之后线程主函数退出，线程销毁
    void LoopClosing::SetFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);

        mbFinished = true;
    }

    // 由外部线程调用，判断当前回环检测线程是否已经正确终止了
    bool LoopClosing::isFinished()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

} // namespace ORB_SLAM
